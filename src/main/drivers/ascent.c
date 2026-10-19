/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * ASCENT: SOC 黑盒子驱动。BF 为 SPI 主机，SOC（现用 ESP32-S3）为从机，
 * 链路承载 MSP v2 帧，SOC 对外提供逻辑 flash 语义（擦除后读 0xFF）。
 * 接线与事务契约见 tools/SPI_LOG/ESP32blackboxtest/README.txt，
 * 命令 ID 改动须与 ESP32blackboxtest/main/ascent_slave.c 同步。
 * Runtime writes/erases use a DMA state machine. Blocking flash APIs are
 * serviced only during initialization or while disarmed.
 */
#include <string.h>

#include "platform.h"

#ifdef USE_ASCENT

#include "blackbox/blackbox.h"
#include "build/debug.h"
#include "common/maths.h"
#include "drivers/ascent.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"
#include "fc/init.h"
#include "fc/runtime_config.h"
#include "pg/bus_spi.h"

extern int16_t debug[];

/* MSP v2 自定义命令，与 ascent_slave.c 一致 */
#define ASCENT_MSP_PING          0x5A01
#define ASCENT_MSP_GEOMETRY      0x5A02
#define ASCENT_MSP_READ          0x5A03
#define ASCENT_MSP_WRITE         0x5A04
#define ASCENT_MSP_ERASE_SECTOR  0x5A05
#define ASCENT_MSP_ERASE_ALL     0x5A06
#define ASCENT_MSP_STATUS        0x5A07

#define ASCENT_JEDEC_ID          0x41534301  // "ASC1"
#define ASCENT_CHUNK             256         // 单帧数据上限，须 <= 从机 READ_MAX
#define ASCENT_BURST             512         // 应答轮询时钟量，须 <= 从机 TRANS_BYTES
#define ASCENT_SPI_SPEED_HZ      8000000     // F405 clock divider selects 5.25 MHz
#define ASCENT_RSP_DELAY_US      2000        // 请求后等 SOC 处理
#define ASCENT_RETRY_POLLS       4
#define ASCENT_TRANSACTION_US    100000
#define ASCENT_ERASE_TIMEOUT_US  300000000
#define ASCENT_STATUS_DELAY_US   10000
#define ASCENT_PING_RETRY_MS     100
#define ASCENT_PING_TIMEOUT_MS   2000

#define MSP_V2_HDR               9

static extDevice_t ascentDevInstance;
static extDevice_t *ascentDev;
static bool ascentAvailable;
static flashDevice_t *ascentFlash;

typedef enum {
    ASCENT_IDLE,
    ASCENT_REQUEST,
    ASCENT_REQUEST_DMA,
    ASCENT_DELAY,
    ASCENT_RESPONSE_DMA,
} ascentState_e;

typedef enum {
    ASCENT_SYNC,
    ASCENT_WRITE,
    ASCENT_ERASE,
    ASCENT_STATUS,
} ascentOperation_e;

static struct {
    ascentState_e state;
    ascentOperation_e operation;
    uint16_t command;
    uint16_t frameLength;
    uint16_t responseLength;
    uint8_t sequence;
    uint8_t polls;
    uint32_t deadline;
    uint32_t nextTransfer;
    bool success;
} transaction;
static bool erasePending;
static uint32_t eraseDeadline;
static uint32_t nextStatus;
static busSegment_t segments[2];

/* 帧缓冲须 DMA 可用 */
STATIC_DMA_DATA_AUTO uint8_t frameBuf[MSP_V2_HDR + 4 + ASCENT_CHUNK + 3];
STATIC_DMA_DATA_AUTO uint8_t pollTxBuf[ASCENT_BURST];
STATIC_DMA_DATA_AUTO uint8_t pollRxBuf[ASCENT_BURST];
static uint8_t chunkBuf[ASCENT_CHUNK];

static uint8_t crc8_dvb_s2(uint8_t crc, const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
        }
    }
    return crc;
}

static void writeU32(uint8_t *p, uint32_t v)
{
    p[0] = v & 0xFF;
    p[1] = (v >> 8) & 0xFF;
    p[2] = (v >> 16) & 0xFF;
    p[3] = (v >> 24) & 0xFF;
}

static uint32_t readU32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int buildFrame(uint8_t *out, uint16_t cmd, const uint8_t *payload, uint16_t len)
{
    out[0] = '$';
    out[1] = 'X';
    out[2] = '<';
    out[3] = 0;
    out[4] = cmd & 0xFF;
    out[5] = cmd >> 8;
    out[6] = len & 0xFF;
    out[7] = len >> 8;
    if (len) {
        memcpy(&out[8], payload, len);
    }
    out[8 + len] = crc8_dvb_s2(0, &out[3], 5 + len);
    return MSP_V2_HDR + len;
}

/* 在一段时钟数据内扫描完整应答帧（帧保证不跨 CS 突发） */
static bool scanResponse(uint16_t expectCmd, uint8_t *payload, uint16_t *payloadLen)
{
    enum { STATE_HDR, STATE_PAY } state = STATE_HDR;
    uint8_t crc = 0;
    uint16_t cmd = 0, len = 0, got = 0;

    for (int i = 0; i < ASCENT_BURST; i++) {
        uint8_t b = pollRxBuf[i];
        switch (state) {
        case STATE_HDR:
            if (b == '$' && i + MSP_V2_HDR <= ASCENT_BURST && pollRxBuf[i + 1] == 'X'
                && pollRxBuf[i + 2] == '>') {
                if (pollRxBuf[i + 3] != transaction.sequence) {
                    continue;
                }
                crc = crc8_dvb_s2(0, &pollRxBuf[i + 3], 5);
                cmd = pollRxBuf[i + 4] | (pollRxBuf[i + 5] << 8);
                len = pollRxBuf[i + 6] | (pollRxBuf[i + 7] << 8);
                if (len > ASCENT_CHUNK || i + MSP_V2_HDR + len > ASCENT_BURST) {
                    return false;
                }
                got = 0;
                state = STATE_PAY;
                i += 7;
            }
            break;
        case STATE_PAY:
            if (got < len) {
                payload[got++] = b;
                crc = crc8_dvb_s2(crc, &b, 1);
            } else if (b == crc && cmd == expectCmd) {
                *payloadLen = len;
                return true;
            } else {
                return false;
            }
            break;
        }
    }
    return false;
}

static bool ascentStart(uint16_t cmd, const uint8_t *payload, uint16_t len, ascentOperation_e operation)
{
    if (!ascentAvailable || transaction.state != ASCENT_IDLE || len > 4 + ASCENT_CHUNK) {
        return false;
    }
    int length = buildFrame(frameBuf, cmd, payload, len);
    if (++transaction.sequence == 0) {
        transaction.sequence = 1;
    }
    frameBuf[3] = transaction.sequence;
    frameBuf[length - 1] = crc8_dvb_s2(0, frameBuf + 3, length - 4);
    transaction.frameLength = (length + 3) & ~3;
    memset(frameBuf + length, 0xFF, transaction.frameLength - length);
    transaction.command = cmd;
    transaction.operation = operation;
    transaction.responseLength = 0;
    transaction.polls = 0;
    transaction.success = false;
    transaction.deadline = micros() + (ARMING_FLAG(ARMED) ? ASCENT_TRANSACTION_US : 10 * ASCENT_TRANSACTION_US);
    if (cmd == ASCENT_MSP_PING && !(systemState & SYSTEM_STATE_READY)) {
        transaction.deadline = micros() + ASCENT_TRANSACTION_US;
    }
    transaction.state = ASCENT_REQUEST;
    return true;
}

static void ascentComplete(bool success)
{
    const ascentOperation_e operation = transaction.operation;
    transaction.success = success;
    transaction.state = ASCENT_IDLE;
    transaction.nextTransfer = micros() + ASCENT_RSP_DELAY_US;
    if (!success) {
        ascentAvailable = false;
        erasePending = false;
        debug[3]++;
    }
    if (operation == ASCENT_WRITE) {
        void (*callback)(uintptr_t) = ascentFlash->callback;
        ascentFlash->callback = NULL;
        if (success) {
            ascentFlash->currentWriteAddress += ascentFlash->bytesWritten;
        }
        if (callback) {
            callback(success ? ascentFlash->bytesWritten : 0);
        }
    } else if (operation == ASCENT_ERASE || operation == ASCENT_STATUS) {
        if (success && operation == ASCENT_STATUS) {
            if (transaction.responseLength != 1 || chunkBuf[0] > 1) {
                ascentAvailable = false;
                erasePending = false;
            } else {
                erasePending = chunkBuf[0] != 0;
            }
        }
        nextStatus = micros() + ASCENT_STATUS_DELAY_US;
    }
}

static void ascentTransfer(uint8_t *tx, uint8_t *rx, unsigned length)
{
    segments[0] = (busSegment_t) {
        .u.buffers = { tx, rx }, .len = length, .negateCS = true,
    };
    // Force DMA even for a short request; init may still use the polled backend.
    segments[1] = (busSegment_t) { .len = 0, .negateCS = false };
    spiSequence(ascentDev, segments);
}

void ascentUpdate(void)
{
    if (!ascentAvailable) {
        return;
    }
    const uint32_t now = micros();
    if (erasePending && cmp32(now, eraseDeadline) >= 0) {
        ascentComplete(false);
        return;
    }
    if (transaction.state == ASCENT_IDLE) {
        if (erasePending && cmp32(now, nextStatus) >= 0) {
            ascentStart(ASCENT_MSP_STATUS, NULL, 0, ASCENT_STATUS);
        }
        return;
    }
    if (cmp32(now, transaction.deadline) >= 0) {
        // Never reuse DMA buffers after timeout; the device stays disabled.
        ascentComplete(false);
        return;
    }
    if (spiIsBusy(ascentDev)) {
        return;
    }
    if ((systemState & SYSTEM_STATE_READY) && !spiUseDMA(ascentDev)) {
        ascentComplete(false);
        return;
    }
    switch (transaction.state) {
    case ASCENT_REQUEST:
        if (cmp32(now, transaction.nextTransfer) >= 0) {
            ascentTransfer(frameBuf, pollRxBuf, transaction.frameLength);
            transaction.state = ASCENT_REQUEST_DMA;
        }
        break;
    case ASCENT_REQUEST_DMA:
        transaction.nextTransfer = now + ASCENT_RSP_DELAY_US;
        transaction.state = ASCENT_DELAY;
        break;
    case ASCENT_DELAY:
        if (cmp32(now, transaction.nextTransfer) >= 0) {
            ascentTransfer(pollTxBuf, pollRxBuf, ASCENT_BURST);
            transaction.state = ASCENT_RESPONSE_DMA;
        }
        break;
    case ASCENT_RESPONSE_DMA:
        if (scanResponse(transaction.command, chunkBuf, &transaction.responseLength)) {
            const uint16_t expectedLength = transaction.command == ASCENT_MSP_PING || transaction.command == ASCENT_MSP_GEOMETRY ? 8
                : transaction.command == ASCENT_MSP_READ ? readU32(frameBuf + 12)
                : transaction.command == ASCENT_MSP_STATUS ? 1 : 0;
            const bool valid = transaction.responseLength == expectedLength
                && (transaction.command != ASCENT_MSP_PING || memcmp(chunkBuf, "ASCENT01", 8) == 0);
            ascentComplete(valid);
        } else {
            // The SOC caches replies by sequence, so lost requests/ACKs can be replayed safely.
            transaction.nextTransfer = now + ASCENT_RSP_DELAY_US;
            transaction.state = ++transaction.polls % ASCENT_RETRY_POLLS == 0 ? ASCENT_REQUEST : ASCENT_DELAY;
        }
        break;
    default:
        break;
    }
}

bool ascentIsAvailable(void)
{
    return ascentAvailable;
}

/* flashVTable has a synchronous read API. It must never wait while armed. */
static bool ascentTransaction(uint16_t cmd, const uint8_t *payload, uint16_t len,
    uint8_t *respPayload, uint16_t *respLen)
{
    if (ARMING_FLAG(ARMED) || !ascentStart(cmd, payload, len, ASCENT_SYNC)) {
        return false;
    }
    while (transaction.state != ASCENT_IDLE) {
        ascentUpdate();
    }
    if (transaction.success) {
        if (respPayload && respPayload != chunkBuf && transaction.responseLength) {
            memcpy(respPayload, chunkBuf, transaction.responseLength);
        }
        if (respLen) {
            *respLen = transaction.responseLength;
        }
    }
    return transaction.success;
}

/* ---- flashVTable ---- */

static void ascent_configure(flashDevice_t *fdevice, uint32_t configurationFlags)
{
    UNUSED(fdevice);
    UNUSED(configurationFlags);
}

static bool ascent_isReady(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
    ascentUpdate();
    // Release legacy synchronous flashfs loops on failure without accepting writes.
    return !ascentAvailable || (transaction.state == ASCENT_IDLE && !erasePending);
}

static bool ascent_waitForReady(flashDevice_t *fdevice)
{
    if (ARMING_FLAG(ARMED)) {
        return ascent_isReady(fdevice);
    }
    while (!ascent_isReady(fdevice));
    return ascentAvailable;
}

static void ascent_eraseSector(flashDevice_t *fdevice, uint32_t address)
{
    if (ARMING_FLAG(ARMED) || !ascent_waitForReady(fdevice)) {
        return;
    }
    uint8_t payload[4];
    writeU32(payload, address);
    if (address <= fdevice->geometry.totalSize - fdevice->geometry.sectorSize
        && address % fdevice->geometry.sectorSize == 0
        && ascentStart(ASCENT_MSP_ERASE_SECTOR, payload, 4, ASCENT_ERASE)) {
        erasePending = true;
        eraseDeadline = micros() + ASCENT_ERASE_TIMEOUT_US;
    }
}

static void ascent_eraseCompletely(flashDevice_t *fdevice)
{
    if (!ARMING_FLAG(ARMED) && ascent_waitForReady(fdevice)
        && ascentStart(ASCENT_MSP_ERASE_ALL, NULL, 0, ASCENT_ERASE)) {
        erasePending = true;
        eraseDeadline = micros() + ASCENT_ERASE_TIMEOUT_US;
    }
}

static void ascent_pageProgramBegin(flashDevice_t *fdevice, uint32_t address, void (*callback)(uintptr_t arg))
{
    fdevice->callback = callback;
    fdevice->currentWriteAddress = address;
}

static uint32_t ascent_pageProgramContinue(flashDevice_t *fdevice, uint8_t const **buffers, const uint32_t *bufferSizes, uint32_t bufferCount)
{
    if (!ascentAvailable || !ascent_isReady(fdevice)) {
        return 0;
    }
    uint32_t consumed = 0;
    uint32_t bufIdx = 0;
    uint32_t bufOff = 0;
    uint8_t payload[4 + ASCENT_CHUNK];

    while (bufIdx < bufferCount && consumed < ASCENT_CHUNK) {
        uint32_t take = MIN(bufferSizes[bufIdx] - bufOff, (uint32_t)ASCENT_CHUNK - consumed);
        memcpy(&payload[4 + consumed], &buffers[bufIdx][bufOff], take);
        consumed += take;
        bufOff += take;
        if (bufOff == bufferSizes[bufIdx]) {
            bufIdx++;
            bufOff = 0;
        }
    }

    if (consumed) {
        writeU32(payload, fdevice->currentWriteAddress);
        if (fdevice->currentWriteAddress > fdevice->geometry.totalSize
            || consumed > fdevice->geometry.totalSize - fdevice->currentWriteAddress
            || !ascentStart(ASCENT_MSP_WRITE, payload, 4 + consumed, ASCENT_WRITE)) {
            consumed = 0;
        }
    }
    fdevice->bytesWritten = consumed;
    return consumed;
}

static void ascent_pageProgramFinish(flashDevice_t *fdevice)
{
    if (!fdevice->bytesWritten) {
        fdevice->callback = NULL;
    }
}

static void ascent_pageProgram(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, uint32_t length, void (*callback)(uintptr_t arg))
{
    if (!callback && ARMING_FLAG(ARMED)) {
        return;
    }
    ascent_pageProgramBegin(fdevice, address, callback);
    if (callback) {
        ascent_pageProgramContinue(fdevice, &data, &length, 1);
        ascent_pageProgramFinish(fdevice);
        return;
    }
    while (length) {
        uint8_t const *buffers[1] = { data };
        uint32_t sizes[1] = { MIN(length, (uint32_t)ASCENT_CHUNK) };
        uint32_t written = ascent_pageProgramContinue(fdevice, buffers, sizes, 1);
        if (!written) {
            break;
        }
        data += written;
        length -= written;
        if (!ascent_waitForReady(fdevice)) {
            break;
        }
    }
    ascent_pageProgramFinish(fdevice);
}

static void ascent_flush(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
}

static int ascent_readBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, uint32_t length)
{
    if (ARMING_FLAG(ARMED) || address > fdevice->geometry.totalSize || !ascent_waitForReady(fdevice)) {
        return 0;
    }
    length = MIN(length, fdevice->geometry.totalSize - address);

    uint32_t done = 0;
    while (done < length) {
        uint32_t take = MIN(length - done, (uint32_t)ASCENT_CHUNK);
        uint8_t payload[8];
        writeU32(payload, address + done);
        writeU32(payload + 4, take);

        uint16_t got = 0;
        if (!ascentTransaction(ASCENT_MSP_READ, payload, 8, chunkBuf, &got)
            || got != take) {
            break;
        }
        memcpy(&buffer[done], chunkBuf, take);
        done += take;
    }
    return done;
}

static const flashGeometry_t *ascent_getGeometry(flashDevice_t *fdevice)
{
    return &fdevice->geometry;
}

static const flashVTable_t ascentVTable = {
    .configure = ascent_configure,
    .isReady = ascent_isReady,
    .waitForReady = ascent_waitForReady,
    .eraseSector = ascent_eraseSector,
    .eraseCompletely = ascent_eraseCompletely,
    .pageProgramBegin = ascent_pageProgramBegin,
    .pageProgramContinue = ascent_pageProgramContinue,
    .pageProgramFinish = ascent_pageProgramFinish,
    .pageProgram = ascent_pageProgram,
    .flush = ascent_flush,
    .readBytes = ascent_readBytes,
    .getGeometry = ascent_getGeometry,
};

bool ascentInit(flashDevice_t *fdevice)
{
    ascentFlash = fdevice;
    transaction.state = ASCENT_IDLE;
    transaction.nextTransfer = micros() + ASCENT_RSP_DELAY_US;
    memset(pollTxBuf, 0xFF, sizeof(pollTxBuf));
    ascentDev = &ascentDevInstance;
    ascentDev->busType_u.spi.csnPin = IOGetByTag(IO_TAG(ASCENT_SPI_CS_PIN));

    debug[0] = 1;
    if (!IOIsFreeOrPreinit(ascentDev->busType_u.spi.csnPin)) {
        debug[0] = 2;
        return false;
    }

    const uint32_t spiDevice = SPI_DEV_TO_CFG(spiDeviceByInstance((const spiResource_t *)ASCENT_SPI_INSTANCE));
    if (!spiSetBusInstance(ascentDev, spiDevice)) {
        debug[0] = 3;
        return false;
    }
    debug[0] = 4;

    IOInit(ascentDev->busType_u.spi.csnPin, OWNER_ASCENT, 0);
    IOConfigGPIO(ascentDev->busType_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(ascentDev->busType_u.spi.csnPin);

    spiSetClkDivisor(ascentDev, spiCalculateDivider(ASCENT_SPI_SPEED_HZ));
    spiSetClkPhasePolarity(ascentDev, true);

    delay(50);

    ascentAvailable = true;

    /* 探活：限定时间内重试，SOC 缺席则本设备不可用 */
    uint32_t start = millis();
    bool pinged = false;
    int attempts = 0;
    do {
        pinged = ascentTransaction(ASCENT_MSP_PING, NULL, 0, NULL, NULL);
        attempts++;
        debug[1] = attempts;
        debug[2] = pinged;
        if (!pinged && !spiIsBusy(ascentDev)) {
            ascentAvailable = true;
            delay(ASCENT_PING_RETRY_MS);
        } else if (!pinged) {
            break;
        }
    } while (!pinged && (millis() - start) < ASCENT_PING_TIMEOUT_MS);

    if (!pinged) {
        debug[0] = 8;
        ascentAvailable = false;
        ioPreinitByTag(IO_TAG(ASCENT_SPI_CS_PIN), IOCFG_IPU, PREINIT_PIN_STATE_HIGH);
        return false;
    }

    uint8_t geometryRsp[8];
    uint16_t got = 0;
    if (!ascentTransaction(ASCENT_MSP_GEOMETRY, NULL, 0, geometryRsp, &got) || got != 8) {
        ascentAvailable = false;
        return false;
    }
    uint32_t totalSize = readU32(geometryRsp);
    uint32_t sectorSize = readU32(geometryRsp + 4);

    if (!totalSize || !sectorSize || (sectorSize % 256) || (totalSize % sectorSize)
        || sectorSize / 256 > UINT16_MAX || totalSize / sectorSize > UINT16_MAX) {
        ascentAvailable = false;
        return false;
    }

    fdevice->io.mode = FLASHIO_SPI;
    fdevice->vTable = &ascentVTable;
    fdevice->geometry.flashType = FLASH_TYPE_NOR;
    fdevice->geometry.pageSize = 256;
    fdevice->geometry.pagesPerSector = sectorSize / 256;
    fdevice->geometry.sectorSize = sectorSize;
    fdevice->geometry.sectors = totalSize / sectorSize;
    fdevice->geometry.totalSize = totalSize;
    fdevice->geometry.jedecId = ASCENT_JEDEC_ID;
    fdevice->geometry.maxReadClkSPIHz = ASCENT_SPI_SPEED_HZ;
    fdevice->geometry.bbReplacementBlocks = 0;
    fdevice->geometry.bufReadModeSet = 0;
    fdevice->isLargeFlash = false;
    fdevice->couldBeBusy = false;

    ascentAvailable = true;
    debug[0] = 7;
    return true;
}
#endif // USE_ASCENT
