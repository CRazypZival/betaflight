#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define USE_ASCENT
#define UNUSED(x) (void)(x)
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define STATIC_DMA_DATA_AUTO static
#define ARMING_FLAG(x) (armingFlags & (x))
#define ARMED 1
#define SYSTEM_STATE_READY 128
#define ASCENT_SPI_CS_PIN 14
#define ASCENT_SPI_INSTANCE ((void *)3)
#define IO_TAG(x) (x)
#define SPI_DEV_TO_CFG(x) ((x) + 1)
#define OWNER_ASCENT 1
#define SPI_IO_CS_CFG 0
#define IOCFG_IPU 0
#define PREINIT_PIN_STATE_HIGH 1
#define FLASHIO_SPI 0
#define FLASH_TYPE_NOR 0

typedef void spiResource_t;
typedef struct {
    struct { struct { int csnPin; } spi; } busType_u;
} extDevice_t;
typedef struct {
    union { struct { uint8_t *txData, *rxData; } buffers; } u;
    int len;
    bool negateCS;
} busSegment_t;
typedef struct {
    uint32_t flashType, pageSize, pagesPerSector, sectorSize, sectors, totalSize;
    uint32_t jedecId, maxReadClkSPIHz, bbReplacementBlocks, bufReadModeSet;
} flashGeometry_t;
typedef struct flashDevice_s flashDevice_t;
typedef struct {
    void (*configure)(flashDevice_t *, uint32_t);
    bool (*isReady)(flashDevice_t *);
    bool (*waitForReady)(flashDevice_t *);
    void (*eraseSector)(flashDevice_t *, uint32_t);
    void (*eraseCompletely)(flashDevice_t *);
    void (*pageProgramBegin)(flashDevice_t *, uint32_t, void (*)(uintptr_t));
    uint32_t (*pageProgramContinue)(flashDevice_t *, const uint8_t **, const uint32_t *, uint32_t);
    void (*pageProgramFinish)(flashDevice_t *);
    void (*pageProgram)(flashDevice_t *, uint32_t, const uint8_t *, uint32_t, void (*)(uintptr_t));
    void (*flush)(flashDevice_t *);
    int (*readBytes)(flashDevice_t *, uint32_t, uint8_t *, uint32_t);
    const flashGeometry_t *(*getGeometry)(flashDevice_t *);
} flashVTable_t;
struct flashDevice_s {
    const flashVTable_t *vTable;
    flashGeometry_t geometry;
    struct { int mode; } io;
    bool isLargeFlash, couldBeBusy;
    uint32_t currentWriteAddress, bytesWritten;
    void (*callback)(uintptr_t);
};

extern uint8_t armingFlags, systemState;
uint32_t micros(void);
uint32_t millis(void);
void delay(unsigned ms);
static inline int32_t cmp32(uint32_t a, uint32_t b) { return (int32_t)(a - b); }
bool spiIsBusy(const extDevice_t *dev);
bool spiUseDMA(const extDevice_t *dev);
void spiSequence(const extDevice_t *dev, busSegment_t *segments);
static inline int IOGetByTag(int tag) { return tag; }
static inline bool IOIsFreeOrPreinit(int pin) { UNUSED(pin); return true; }
static inline int spiDeviceByInstance(const spiResource_t *p) { UNUSED(p); return 2; }
static inline bool spiSetBusInstance(extDevice_t *d, uint32_t i) { UNUSED(d); UNUSED(i); return true; }
static inline void IOInit(int p, int o, int i) { UNUSED(p); UNUSED(o); UNUSED(i); }
static inline void IOConfigGPIO(int p, int c) { UNUSED(p); UNUSED(c); }
static inline void IOHi(int p) { UNUSED(p); }
static inline void ioPreinitByTag(int p, int c, int s) { UNUSED(p); UNUSED(c); UNUSED(s); }
static inline uint16_t spiCalculateDivider(unsigned f) { UNUSED(f); return 1; }
static inline void spiSetClkDivisor(extDevice_t *d, unsigned v) { UNUSED(d); UNUSED(v); }
static inline void spiSetClkPhasePolarity(extDevice_t *d, bool p) { UNUSED(d); UNUSED(p); }
