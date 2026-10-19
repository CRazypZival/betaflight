#include <assert.h>
#include <stdio.h>
#include "../../main/drivers/ascent.c"

uint8_t armingFlags, systemState;
int16_t debug[8];
static uint32_t clockUs, dmaUntil;
static bool dmaEnabled, offline, corrupt;
static unsigned requests, polls, callbacks;
static unsigned writes;
static uint8_t lastSequence;
static bool dropRequest;
static uintptr_t completedBytes;
static uint8_t response[512];
static uint8_t storage[8192];
static uint8_t statusByte;
static uint32_t slaveReadyAt;

uint32_t micros(void) { return clockUs++; }
uint32_t millis(void) { return clockUs / 1000; }
void delay(unsigned ms) { clockUs += ms * 1000; }
bool spiIsBusy(const extDevice_t *dev) { UNUSED(dev); return cmp32(clockUs, dmaUntil) < 0; }
bool spiUseDMA(const extDevice_t *dev) { UNUSED(dev); return dmaEnabled; }

void spiSequence(const extDevice_t *dev, busSegment_t *s)
{
    UNUSED(dev);
    assert(!spiIsBusy(dev));
    assert(s[0].negateCS && !s[1].len && !s[1].negateCS);
    uint8_t *tx = s[0].u.buffers.txData;
    uint8_t *rx = s[0].u.buffers.rxData;
    memset(rx, 0xFF, s[0].len);
    if (tx[0] == '$') {
        requests++;
        if (dropRequest) {
            dropRequest = false;
            return;
        }
        if (tx[3] == lastSequence) {
            dmaUntil = clockUs + 100;
            return;
        }
        lastSequence = tx[3];
        assert(s[0].len % 4 == 0);
        const uint16_t cmd = tx[4] | tx[5] << 8;
        const uint16_t len = tx[6] | tx[7] << 8;
        assert(crc8_dvb_s2(0, tx + 3, len + 5) == tx[len + 8]);
        uint8_t data[256];
        uint16_t n = 0;
        if (cmd == ASCENT_MSP_PING) {
            memcpy(data, "ASCENT01", 8); n = 8;
        } else if (cmd == ASCENT_MSP_GEOMETRY) {
            writeU32(data, sizeof(storage)); writeU32(data + 4, 4096); n = 8;
        } else if (cmd == ASCENT_MSP_READ) {
            n = readU32(tx + 12); memcpy(data, storage + readU32(tx + 8), n);
        } else if (cmd == ASCENT_MSP_WRITE) {
            writes++;
            memcpy(storage + readU32(tx + 8), tx + 12, len - 4);
        } else if (cmd == ASCENT_MSP_STATUS) {
            data[0] = statusByte; n = 1;
        }
        memset(response, 0xFF, sizeof(response));
        buildFrame(response, cmd, data, n);
        response[2] = '>';
        response[3] = tx[3];
        response[8 + n] = crc8_dvb_s2(0, response + 3, 5 + n);
        if (corrupt) response[8 + n] ^= 1;
    } else {
        polls++;
        if (!offline && clockUs >= slaveReadyAt) memcpy(rx, response, s[0].len);
    }
    dmaUntil = clockUs + 100;
}

static void callback(uintptr_t n) { callbacks++; completedBytes = n; }
static void tick(void) { clockUs += 1000; ascentUpdate(); }
static void drain(void)
{
    for (int i = 0; i < 2000 && transaction.state != ASCENT_IDLE; i++) tick();
    assert(transaction.state == ASCENT_IDLE);
}
static void reset(flashDevice_t *device)
{
    clockUs = dmaUntil = 0;
    armingFlags = 0; systemState = 0;
    dmaEnabled = true; offline = corrupt = false;
    requests = polls = callbacks = 0; completedBytes = 999;
    writes = 0; lastSequence = 0; dropRequest = false;
    erasePending = false; statusByte = 0;
    memset(storage, 0xFF, sizeof(storage));
    memset(device, 0, sizeof(*device));
    assert(ascentInit(device));
    assert(device->geometry.totalSize == sizeof(storage));
    systemState = SYSTEM_STATE_READY;
}

int main(void)
{
    flashDevice_t device;
    reset(&device);
    uint8_t data[256];
    for (unsigned i = 0; i < sizeof(data); i++) data[i] = i * 7;
    const uint8_t *buffers[] = { data, data + 100 };
    const uint32_t sizes[] = { 100, 156 };
    armingFlags = ARMED;
    ascent_pageProgramBegin(&device, 128, callback);
    uint32_t before = clockUs;
    assert(ascent_pageProgramContinue(&device, buffers, sizes, 2) == 256);
    ascent_pageProgramFinish(&device);
    assert(clockUs - before < 20 && callbacks == 0);
    assert(device.currentWriteAddress == 128);
    drain();
    assert(callbacks == 1 && completedBytes == 256);
    assert(device.currentWriteAddress == 384);
    assert(memcmp(storage + 128, data, 256) == 0);
    assert(ascent_readBytes(&device, 128, data, 256) == 0);
    armingFlags = 0;
    uint8_t readback[256];
    assert(ascent_readBytes(&device, 128, readback, 256) == 256);
    assert(memcmp(readback, data, 256) == 0);
    puts("PASS DMA write, split buffers, callback byte count, armed read guard, readback");

    // Headers at the end of a burst must never read beyond the DMA buffer.
    uint16_t n;
    memset(pollRxBuf, 0xFF, sizeof(pollRxBuf));
    memcpy(pollRxBuf + 509, "$X>", 3);
    assert(!scanResponse(ASCENT_MSP_READ, readback, &n));
    buildFrame(pollRxBuf, ASCENT_MSP_READ, data, 256); pollRxBuf[2] = '>';
    pollRxBuf[3] = transaction.sequence;
    pollRxBuf[264] = crc8_dvb_s2(0, pollRxBuf + 3, 261);
    assert(scanResponse(ASCENT_MSP_READ, readback, &n) && n == 256);
    pollRxBuf[20] ^= 1;
    assert(!scanResponse(ASCENT_MSP_READ, readback, &n));
    puts("PASS payload CRC and truncated response bounds (ASan/UBSan)");

    reset(&device); armingFlags = ARMED; offline = true;
    unsigned baseline = requests;
    ascent_pageProgramBegin(&device, 0, callback);
    assert(ascent_pageProgramContinue(&device, buffers, sizes, 2) == 256);
    drain();
    assert(!ascentIsAvailable() && callbacks == 1 && completedBytes == 0);
    assert(requests > baseline + 1 && polls > 1 && writes == 1 && device.currentWriteAddress == 0);
    assert(ascent_isReady(&device));
    puts("PASS offline timeout, deduplicated retries, failed write preserves cursor");

    reset(&device); armingFlags = ARMED; dropRequest = true;
    ascent_pageProgramBegin(&device, 0, callback);
    assert(ascent_pageProgramContinue(&device, buffers, sizes, 2) == 256);
    drain();
    assert(ascentIsAvailable() && callbacks == 1 && completedBytes == 256 && writes == 1);
    puts("PASS lost request recovers through sequence-preserving retry");

    reset(&device); dmaEnabled = false; armingFlags = ARMED;
    baseline = requests;
    ascent_pageProgramBegin(&device, 0, callback);
    assert(ascent_pageProgramContinue(&device, buffers, sizes, 2) == 256);
    drain();
    assert(!ascentIsAvailable() && requests == baseline && completedBytes == 0);
    puts("PASS missing DMA disables runtime recording without polled fallback");

    reset(&device); statusByte = 1;
    ascent_eraseCompletely(&device); drain();
    assert(erasePending && !ascent_isReady(&device));
    clockUs += ASCENT_STATUS_DELAY_US; ascentUpdate(); drain();
    assert(erasePending);
    clockUs += 140000000;
    ascentUpdate(); drain();
    assert(erasePending && ascentIsAvailable());
    statusByte = 0;
    clockUs += ASCENT_STATUS_DELAY_US; ascentUpdate(); drain();
    assert(!erasePending && ascent_isReady(&device));
    puts("PASS erase readiness follows SOC completion");

    reset(&device); armingFlags = ARMED;
    baseline = requests;
    ascent_pageProgram(&device, 0, data, sizeof(data), NULL);
    assert(requests == baseline && transaction.state == ASCENT_IDLE);
    puts("PASS synchronous program is rejected while armed");

    reset(&device);
    systemState = 0; clockUs = dmaUntil = 0; slaveReadyAt = 600000;
    assert(ascentInit(&device));
    assert(debug[1] > 1 && clockUs < 2100000);
    slaveReadyAt = 0;
    puts("PASS bounded startup retries tolerate delayed slave boot");

    reset(&device); corrupt = true;
    assert(ascent_readBytes(&device, 0, readback, 256) == 0);
    assert(!ascentIsAvailable());
    puts("PASS corrupt reply disables device without copying unverified data");
    return 0;
}
