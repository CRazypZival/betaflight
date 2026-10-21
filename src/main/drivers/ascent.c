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

#include <string.h>

#include "platform.h"

#ifdef USE_ASCENT

#include "build/debug.h"
#include "common/maths.h"
#include "drivers/ascent.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"
#include "pg/bus_spi.h"

extern int16_t debug[];

#define ASCENT_MSP_DATA          0x5A10

#define ASCENT_SPI_SPEED_HZ      8000000
#define ASCENT_FRAME_GAP_US      3000
#define ASCENT_DMA_TIMEOUT_US    100000
#define ASCENT_RING_MASK         (ASCENT_RING_SIZE - 1)

#define MSP_V2_HDR               9

static extDevice_t ascentDevInstance;
static extDevice_t *ascentDev;
static bool ascentAvailable;

typedef enum {
    ASCENT_IDLE,
    ASCENT_SENDING,
} ascentState_e;

static struct {
    ascentState_e state;
    uint16_t frameLength;
    uint8_t sequence;
    uint32_t nextFrameUs;
    uint32_t deadlineUs;
} transaction;

STATIC_DMA_DATA_AUTO uint8_t frameBuf[MSP_V2_HDR + ASCENT_CHUNK + 3];
// The DMA ISR retains this list after ascentUpdate returns.
static busSegment_t segments[2];
static uint8_t chunkBuf[ASCENT_CHUNK];

static uint8_t ringBuf[ASCENT_RING_SIZE];
static volatile uint16_t ringHead;
static volatile uint16_t ringTail;

static uint8_t crc8_dvb_s2(uint8_t crc, const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static int buildFrame(uint8_t *out, uint8_t sequence, uint16_t cmd, const uint8_t *payload, uint16_t len)
{
    out[0] = '$';
    out[1] = 'X';
    out[2] = '<';
    out[3] = sequence;
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

void ascentUpdate(void)
{
    if (!ascentAvailable) {
        return;
    }

    const uint32_t now = micros();

    if (transaction.state == ASCENT_SENDING) {
        if (!spiIsBusy(ascentDev)) {
            transaction.state = ASCENT_IDLE;
            transaction.nextFrameUs = now + ASCENT_FRAME_GAP_US;
        } else if (cmp32(now, transaction.deadlineUs) >= 0) {
            // Leave the DMA buffers untouched; a late interrupt may still use them.
            ascentAvailable = false;
            debug[0] = 8;
        }
        return;
    }

    if (cmp32(now, transaction.nextFrameUs) < 0 || ringHead == ringTail || spiIsBusy(ascentDev)) {
        return;
    }

    if (!spiUseSDO_DMA(ascentDev)) {
        ascentAvailable = false;
        debug[0] = 9;
        return;
    }

    debug[2]++;

    uint16_t used = (ringHead - ringTail) & ASCENT_RING_MASK;
    uint16_t take = MIN(used, (uint16_t)ASCENT_CHUNK);
    uint16_t contiguous = MIN(take, (uint16_t)(ASCENT_RING_SIZE - ringTail));
    memcpy(chunkBuf, &ringBuf[ringTail], contiguous);

    int length = buildFrame(frameBuf, ++transaction.sequence, ASCENT_MSP_DATA, chunkBuf, contiguous);
    transaction.frameLength = (length + 3) & ~3;
    memset(frameBuf + length, 0xFF, transaction.frameLength - length);

    ringTail = (ringTail + contiguous) & ASCENT_RING_MASK;

    segments[0] = (busSegment_t) {
        .u.buffers = { frameBuf, NULL }, .len = transaction.frameLength, .negateCS = true,
    };
    segments[1] = (busSegment_t) { .len = 0, .negateCS = false };
    transaction.state = ASCENT_SENDING;
    transaction.deadlineUs = now + ASCENT_DMA_TIMEOUT_US;
    spiSequence(ascentDev, segments);
}

void ascentWrite(const uint8_t *data, int len)
{
    if (!ascentAvailable) {
        return;
    }
    for (int i = 0; i < len; i++) {
        uint16_t next = (ringHead + 1) & ASCENT_RING_MASK;
        if (next == ringTail) {
            debug[3]++;
            return;
        }
        ringBuf[ringHead] = data[i];
        ringHead = next;
        debug[1] = ringHead;
    }
}

int ascentWriteFree(void)
{
    return ascentAvailable ? (ringTail - ringHead - 1) & ASCENT_RING_MASK : 0;
}

bool ascentFlushed(void)
{
    return !ascentAvailable || (transaction.state == ASCENT_IDLE && ringHead == ringTail);
}

bool ascentIsAvailable(void)
{
    return ascentAvailable;
}

void ascentInit(void)
{
    ascentDev = &ascentDevInstance;
    ascentDev->busType_u.spi.csnPin = IOGetByTag(IO_TAG(ASCENT_SPI_CS_PIN));

    debug[0] = 1;
    if (!IOIsFreeOrPreinit(ascentDev->busType_u.spi.csnPin)) {
        debug[0] = 2;
        return;
    }

    const uint32_t spiDevice = SPI_DEV_TO_CFG(spiDeviceByInstance((const spiResource_t *)ASCENT_SPI_INSTANCE));
    if (!spiSetBusInstance(ascentDev, spiDevice)) {
        debug[0] = 3;
        return;
    }
    debug[0] = 4;

    IOInit(ascentDev->busType_u.spi.csnPin, OWNER_ASCENT, 0);
    IOConfigGPIO(ascentDev->busType_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(ascentDev->busType_u.spi.csnPin);

    spiSetClkDivisor(ascentDev, spiCalculateDivider(ASCENT_SPI_SPEED_HZ));
    spiSetClkPhasePolarity(ascentDev, true);

    transaction.state = ASCENT_IDLE;
    transaction.nextFrameUs = micros();
    ascentAvailable = true;
    debug[0] = 7;
}
#endif
