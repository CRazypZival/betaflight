#include <assert.h>
#include <stdio.h>
#include "../../main/drivers/ascent.c"

uint8_t armingFlags, systemState;
int16_t debug[8];
static uint32_t clockUs, dmaUntil;
static unsigned frames;
static uint8_t stream[16384];
static uint32_t writePtr;
static uint8_t lastSeq;
static busSegment_t *pendingSegments;
static bool dmaEnabled = true;
static uint32_t sharedBusyUntil;

uint32_t micros(void) { return clockUs++; }
uint32_t millis(void) { return clockUs / 1000; }
void delay(unsigned ms) { clockUs += ms * 1000; }
bool spiIsBusy(const extDevice_t *dev) { UNUSED(dev); return pendingSegments || cmp32(clockUs, sharedBusyUntil) < 0; }
bool spiUseSDO_DMA(const extDevice_t *dev) { UNUSED(dev); return dmaEnabled; }

void spiSequence(const extDevice_t *dev, busSegment_t *s)
{
    UNUSED(dev);
    assert(!spiIsBusy(dev));
    pendingSegments = s;
    dmaUntil = clockUs + 100;
}

static void completeDMA(void)
{
    busSegment_t *s = pendingSegments;
    pendingSegments = NULL;
    assert(s[0].negateCS && !s[1].len && !s[1].negateCS);
    uint8_t *tx = s[0].u.buffers.txData;
    assert(s[0].len % 4 == 0);
    assert(tx[0] == '$' && tx[1] == 'X' && tx[2] == '<');
    assert(tx[3] != lastSeq);                       // 序列号必须递增
    lastSeq = tx[3];
    const uint16_t cmd = tx[4] | tx[5] << 8;
    const uint16_t len = tx[6] | tx[7] << 8;
    assert(len <= ASCENT_CHUNK);
    assert(crc8_dvb_s2(0, tx + 3, len + 5) == tx[len + 8]);
    assert(cmd == ASCENT_MSP_DATA);
    memcpy(stream + writePtr, tx + 8, len);
    writePtr += len;
    frames++;
}

static void tick(void)
{
    clockUs += 1000;
    if (pendingSegments && cmp32(clockUs, dmaUntil) >= 0) {
        completeDMA();
    }
    ascentUpdate();
}
static void drain(unsigned ticks)
{
    for (unsigned i = 0; i < ticks; i++) tick();
    assert(transaction.state == ASCENT_IDLE);
}
static void reset(void)
{
    clockUs = dmaUntil = 0;
    pendingSegments = NULL; sharedBusyUntil = 0; dmaEnabled = true;
    armingFlags = 0; systemState = SYSTEM_STATE_READY;
    frames = 0; writePtr = 0; lastSeq = 0;
    memset(stream, 0, sizeof(stream));
    ringHead = ringTail = 0;
    ascentAvailable = false;
    transaction.state = ASCENT_IDLE;
    transaction.nextFrameUs = 0;
    ascentInit();
    assert(ascentIsAvailable());
}

int main(void)
{
    uint8_t data[600];
    for (unsigned i = 0; i < sizeof(data); i++) data[i] = i * 7;

    puts("PASS init enables device");

    // 流式推送：600B 跨 3 帧，按序完整到达
    reset();
    ascentWrite(data, sizeof(data));
    assert(!ascentFlushed());
    drain(2000);
    assert(ascentFlushed() && frames == 3 && writePtr == sizeof(data));
    assert(memcmp(stream, data, sizeof(data)) == 0);
    puts("PASS streamed bytes arrive in order across chunked DATA frames");

    // 帧间隔：相邻帧起点间隔不小于 GAP
    reset();
    ascentWrite(data, 256);
    drain(100);
    assert(frames == 1);
    puts("PASS frame pacing honours gap");

    // 缓冲满：丢弃新数据，已入队前缀完整
    reset();
    uint8_t big[5000];
    memset(big, 0x5A, sizeof(big));
    ascentWrite(big, sizeof(big));
    assert(ascentWriteFree() == 0);                 // 恰好收满 RING-1 字节
    drain(60000);
    assert(writePtr == ASCENT_RING_SIZE - 1);
    assert(memcmp(stream, big, writePtr) == 0);
    puts("PASS ring overflow drops newest bytes and preserves enqueued prefix");

    // A DMA completion delayed by less than the deadline does not block taskMain.
    reset();
    ascentWrite(data, 256);
    tick();
    assert(transaction.state == ASCENT_SENDING);
    dmaUntil = clockUs + 1000000000;
    ascentWrite(data, 10);
    for (int i = 0; i < 50; i++) tick();            // 挂死期间：不产出也不阻塞
    assert(writePtr == 0);
    dmaUntil = clockUs - 1;
    for (int i = 0; i < 100 && writePtr != 266; i++) tick();
    assert(writePtr == 266);
    puts("PASS delayed DMA completion keeps buffers alive and does not block");

    reset();
    sharedBusyUntil = clockUs + 10000;
    ascentWrite(data, 256);
    for (int i = 0; i < 5; i++) tick();
    assert(!pendingSegments && ringTail == 0);
    drain(100);
    assert(writePtr == 256 && memcmp(stream, data, 256) == 0);
    puts("PASS shared SPI bus remains untouched while busy");

    reset(); dmaEnabled = false;
    ascentWrite(data, 256); tick();
    assert(!ascentIsAvailable() && !pendingSegments && frames == 0);
    assert(ascentFlushed() && ascentWriteFree() == 0);
    puts("PASS missing DMA stops logging without a blocking fallback");

    reset();
    ascentWrite(data, 256); tick();
    busSegment_t *inFlight = pendingSegments;
    uint8_t savedFrame[sizeof(frameBuf)];
    memcpy(savedFrame, frameBuf, sizeof(savedFrame));
    dmaUntil = clockUs + 1000000000;
    for (int i = 0; i < 110; i++) tick();
    assert(!ascentIsAvailable() && ascentFlushed());
    ascentWrite(data, 256); tick();
    assert(pendingSegments == inFlight && memcmp(frameBuf, savedFrame, sizeof(savedFrame)) == 0);
    dmaUntil = clockUs - 1; tick();
    assert(frames == 1 && !ascentIsAvailable());
    puts("PASS DMA timeout disables logging and preserves buffers for a late ISR");

    // 写指针观测自洽
    reset();
    for (int i = 0; i < 10; i++) {
        ascentWrite(data, 100);
        assert((int)ascentWriteFree() == ASCENT_RING_SIZE - 1 - 100 * (i + 1));
    }
    puts("PASS write free space accounting");
    return 0;
}
