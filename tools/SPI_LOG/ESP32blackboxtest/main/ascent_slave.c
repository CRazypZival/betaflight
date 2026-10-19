/*
 * ASCENT 测试固件 - ESP32-S3 SPI 从机端
 * MSP v2 over SPI + persistent flash partition
 * 协议契约见同目录 README.txt
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_slave.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_attr.h"

/* 接线（BF 主机 ↔ 本机从机），可改 */
#define PIN_MOSI 10  /* BF→ESP */
#define PIN_MISO 11  /* ESP→BF */
#define PIN_SCK  12
#define PIN_CS   13

#define TRANS_BYTES   512
#define FLASH_SECTOR  4096
#define MSP_MAX_PAYLOAD 1024
#define READ_MAX      256

/* 自定义命令 ID，最终分配后全局替换 */
#define ASCENT_PING         0x5A01
#define ASCENT_GEOMETRY     0x5A02
#define ASCENT_READ         0x5A03
#define ASCENT_WRITE        0x5A04
#define ASCENT_ERASE_SECTOR 0x5A05
#define ASCENT_ERASE_ALL    0x5A06
#define ASCENT_STATUS       0x5A07

static const char *TAG = "ascent";

static const esp_partition_t *store;
static uint32_t capacity;
static portMUX_TYPE eraseLock = portMUX_INITIALIZER_UNLOCKED;
static uint8_t eraseStatus; // 0 ready, 1 busy, 2 failed
static bool startErase;
static bool eraseLaunched;

/* DMA 缓冲必须常驻内部 RAM */
DMA_ATTR static uint8_t rxbuf[TRANS_BYTES];
DMA_ATTR static uint8_t txbuf[TRANS_BYTES];

/* ---------- MSP v2 ---------- */

enum { MSP_IDLE, MSP_MAGIC, MSP_DIR, MSP_FLAG, MSP_CMD_LO, MSP_CMD_HI,
       MSP_LEN_LO, MSP_LEN_HI, MSP_PAYLOAD, MSP_CRC };

static struct {
    int state;
    uint8_t crc;
    uint8_t flag;
    uint16_t cmd, len, got;
    uint8_t payload[MSP_MAX_PAYLOAD];
} msp;

static uint8_t crc8_dvb_s2(uint8_t crc, const uint8_t *d, int n)
{
    for (int i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    return crc;
}

static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static void wr32(uint8_t *p, uint32_t v)
{
    p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}

static int msp_build(uint8_t *out, uint16_t cmd, const uint8_t *data, uint16_t len)
{
    out[0] = '$'; out[1] = 'X'; out[2] = '>'; out[3] = msp.flag;
    out[4] = cmd & 0xFF; out[5] = cmd >> 8;
    out[6] = len & 0xFF; out[7] = len >> 8;
    if (len)
        memcpy(&out[8], data, len);
    out[8 + len] = crc8_dvb_s2(0, &out[3], 5 + len);
    return 9 + len;
}

static void erase_all_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "erase_all start");
    esp_err_t err = ESP_OK;
    for (uint32_t address = 0; address < capacity && err == ESP_OK; address += FLASH_SECTOR) {
        err = esp_partition_erase_range(store, address, FLASH_SECTOR);
        vTaskDelay(1);
    }
    portENTER_CRITICAL(&eraseLock);
    eraseStatus = err == ESP_OK ? 0 : 2;
    portEXIT_CRITICAL(&eraseLock);
    ESP_LOGI(TAG, "erase_all %s", esp_err_to_name(err));
    vTaskDelete(NULL);
}

/* 返回应答帧长度；0 = 不应答 */
static int handle_cmd(uint16_t cmd, const uint8_t *p, uint16_t len, uint8_t *resp)
{
    portENTER_CRITICAL(&eraseLock);
    const uint8_t status = eraseStatus;
    portEXIT_CRITICAL(&eraseLock);
    if (status && cmd != ASCENT_STATUS && cmd != ASCENT_PING && cmd != ASCENT_GEOMETRY) {
        return 0;
    }
    switch (cmd) {
    case ASCENT_PING:
        if (len != 0)
            return 0;
        ESP_LOGI(TAG, "PING");
        return msp_build(resp, cmd, (const uint8_t *)"ASCENT01", 8);

    case ASCENT_GEOMETRY: {
        if (len != 0)
            return 0;
        uint8_t b[8];
        wr32(b, capacity);
        wr32(b + 4, FLASH_SECTOR);
        ESP_LOGI(TAG, "GEOMETRY %u/%u", (unsigned)capacity, (unsigned)FLASH_SECTOR);
        return msp_build(resp, cmd, b, 8);
    }

    case ASCENT_READ: {
        if (len != 8)
            return 0;
        uint32_t addr = rd32(p), n = rd32(p + 4);
        if (n > READ_MAX || addr > capacity || n > capacity - addr) {
            ESP_LOGW(TAG, "READ oob addr=%u n=%u", (unsigned)addr, (unsigned)n);
            return 0;
        }
        static uint8_t rdata[READ_MAX];
        esp_err_t err = esp_partition_read(store, addr, rdata, n);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "READ fail addr=%u %s", (unsigned)addr, esp_err_to_name(err));
            return 0;
        }
        ESP_LOGD(TAG, "READ addr=%u n=%u", (unsigned)addr, (unsigned)n);
        return msp_build(resp, cmd, rdata, n);
    }

    case ASCENT_WRITE: {
        if (len < 4)
            return 0;
        uint32_t addr = rd32(p);
        uint32_t n = len - 4;
        if (n > READ_MAX || addr > capacity || n > capacity - addr) {
            ESP_LOGW(TAG, "WRITE oob addr=%u n=%u", (unsigned)addr, (unsigned)n);
            return 0;
        }
        esp_err_t err = esp_partition_write(store, addr, p + 4, n);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "WRITE fail addr=%u %s", (unsigned)addr, esp_err_to_name(err));
            return 0;
        }
        ESP_LOGD(TAG, "WRITE addr=%u n=%u", (unsigned)addr, (unsigned)n);
        return msp_build(resp, cmd, NULL, 0);
    }

    case ASCENT_ERASE_SECTOR: {
        if (len != 4)
            return 0;
        uint32_t addr = rd32(p);
        if (addr >= capacity || addr % FLASH_SECTOR) {
            ESP_LOGW(TAG, "ERASE oob addr=%u", (unsigned)addr);
            return 0;
        }
        esp_err_t err = esp_partition_erase_range(store, addr, FLASH_SECTOR);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ERASE fail addr=%u %s", (unsigned)addr, esp_err_to_name(err));
            return 0;
        }
        ESP_LOGI(TAG, "ERASE_SECTOR addr=%u", (unsigned)addr);
        return msp_build(resp, cmd, NULL, 0);
    }

    case ASCENT_ERASE_ALL:
        if (len != 0)
            return 0;
        ESP_LOGI(TAG, "ERASE_ALL (background)");
        portENTER_CRITICAL(&eraseLock);
        eraseStatus = 1;
        portEXIT_CRITICAL(&eraseLock);
        eraseLaunched = false;
        return msp_build(resp, cmd, NULL, 0);

    case ASCENT_STATUS:
        return len == 0 ? msp_build(resp, cmd, &status, 1) : 0;

    default:
        ESP_LOGW(TAG, "unknown cmd 0x%04x", cmd);
        return 0;
    }
}

static uint8_t respbuf[MSP_MAX_PAYLOAD + 16];
static int resp_len;
static struct {
    bool valid;
    uint8_t flag;
    uint16_t cmd, len;
    uint8_t payload[MSP_MAX_PAYLOAD];
    uint8_t response[TRANS_BYTES];
    int responseLength;
} lastRequest;

/* 喂一个收到的字节；完整且 CRC 正确的请求帧在此产生应答 */
static void msp_feed(uint8_t b)
{
    switch (msp.state) {
    case MSP_IDLE:
        if (b == '$') {
            msp.state = MSP_MAGIC;
            msp.crc = 0;
        }
        break;
    case MSP_MAGIC:
        msp.state = (b == 'X') ? MSP_DIR : MSP_IDLE;
        break;
    case MSP_DIR:
        msp.state = (b == '<') ? MSP_FLAG : MSP_IDLE;
        break;
    case MSP_FLAG:
        msp.flag = b;
        msp.crc = crc8_dvb_s2(msp.crc, &b, 1);
        msp.state = MSP_CMD_LO;
        break;
    case MSP_CMD_LO:
        msp.cmd = b;
        msp.crc = crc8_dvb_s2(msp.crc, &b, 1);
        msp.state = MSP_CMD_HI;
        break;
    case MSP_CMD_HI:
        msp.cmd |= b << 8;
        msp.crc = crc8_dvb_s2(msp.crc, &b, 1);
        msp.state = MSP_LEN_LO;
        break;
    case MSP_LEN_LO:
        msp.len = b;
        msp.crc = crc8_dvb_s2(msp.crc, &b, 1);
        msp.state = MSP_LEN_HI;
        break;
    case MSP_LEN_HI:
        msp.len |= b << 8;
        msp.crc = crc8_dvb_s2(msp.crc, &b, 1);
        if (msp.len > MSP_MAX_PAYLOAD) {
            ESP_LOGW(TAG, "payload too big %u", msp.len);
            msp.state = MSP_IDLE;
        } else {
            msp.got = 0;
            msp.state = msp.len ? MSP_PAYLOAD : MSP_CRC;
        }
        break;
    case MSP_PAYLOAD:
        msp.payload[msp.got++] = b;
        msp.crc = crc8_dvb_s2(msp.crc, &b, 1);
        if (msp.got == msp.len)
            msp.state = MSP_CRC;
        break;
    case MSP_CRC:
        if (b == msp.crc) {
            if (lastRequest.valid && lastRequest.flag == msp.flag && lastRequest.cmd == msp.cmd
                && lastRequest.len == msp.len && memcmp(lastRequest.payload, msp.payload, msp.len) == 0) {
                resp_len = lastRequest.responseLength;
                memcpy(respbuf, lastRequest.response, resp_len);
            } else {
                resp_len = handle_cmd(msp.cmd, msp.payload, msp.len, respbuf);
                if (resp_len > 0) {
                    lastRequest.valid = true;
                    lastRequest.flag = msp.flag;
                    lastRequest.cmd = msp.cmd;
                    lastRequest.len = msp.len;
                    memcpy(lastRequest.payload, msp.payload, msp.len);
                    lastRequest.responseLength = resp_len;
                    memcpy(lastRequest.response, respbuf, resp_len);
                }
            }
        } else
            ESP_LOGW(TAG, "crc err cmd=0x%04x", msp.cmd);
        msp.state = MSP_IDLE;
        break;
    }
}

/* ---------- 上电自检 ---------- */

static bool selftest(void)
{
    const uint32_t addr = store->size - FLASH_SECTOR;
    static uint8_t sec[FLASH_SECTOR];
    static uint8_t cmp[FLASH_SECTOR];

    if (esp_partition_erase_range(store, addr, FLASH_SECTOR) != ESP_OK)
        return false;
    if (esp_partition_read(store, addr, sec, FLASH_SECTOR) != ESP_OK)
        return false;
    for (uint32_t i = 0; i < FLASH_SECTOR; i++)
        if (sec[i] != 0xFF)
            return false;

    for (uint32_t i = 0; i < FLASH_SECTOR; i++)
        sec[i] = (uint8_t)(i * 7);
    if (esp_partition_write(store, addr, sec, FLASH_SECTOR) != ESP_OK)
        return false;
    if (esp_partition_read(store, addr, cmp, FLASH_SECTOR) != ESP_OK)
        return false;
    if (memcmp(sec, cmp, FLASH_SECTOR) != 0)
        return false;

    if (esp_partition_erase_range(store, addr, FLASH_SECTOR) != ESP_OK)
        return false;
    return true;
}

void app_main(void)
{
    store = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, (esp_partition_type_t)0x40, "storage");
    if (!store) {
        ESP_LOGE(TAG, "storage partition not found");
        return;
    }
    ESP_LOGI(TAG, "storage: %s @0x%lx size=%u", store->label, (unsigned long)store->address, (unsigned)store->size);
    capacity = store->size - FLASH_SECTOR;

    if (!selftest()) {
        ESP_LOGE(TAG, "selftest FAIL");
        return;
    }
    ESP_LOGI(TAG, "selftest PASS, logical capacity=%u", (unsigned)capacity);

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "spi slave ready: MOSI=%d MISO=%d SCK=%d CS=%d",
             PIN_MOSI, PIN_MISO, PIN_SCK, PIN_CS);

    spi_slave_transaction_t t = {0}, *ret;
    resp_len = 0;

    for (;;) {
        memset(txbuf, 0xFF, TRANS_BYTES);
        if (resp_len > 0) {
            int n = resp_len < TRANS_BYTES ? resp_len : TRANS_BYTES;
            memcpy(txbuf, respbuf, n);
            resp_len = 0;
        }

        memset(rxbuf, 0, TRANS_BYTES);
        t.length = TRANS_BYTES * 8;
        t.tx_buffer = txbuf;
        t.rx_buffer = rxbuf;
        t.user = NULL;

        ESP_ERROR_CHECK(spi_slave_queue_trans(SPI2_HOST, &t, portMAX_DELAY));
        // Queue the next receive before starting flash work, after the erase ACK was clocked out.
        if (startErase) {
            startErase = false;
            if (xTaskCreate(erase_all_task, "erase_all", 4096, NULL, 3, NULL) != pdPASS) {
                portENTER_CRITICAL(&eraseLock);
                eraseStatus = 2;
                portEXIT_CRITICAL(&eraseLock);
            }
        }
        ESP_ERROR_CHECK(spi_slave_get_trans_result(SPI2_HOST, &ret, portMAX_DELAY));

        int n = ret->trans_len / 8;
        if (n >= 9 && txbuf[0] == '$' && txbuf[4] == (ASCENT_ERASE_ALL & 0xFF)
            && txbuf[5] == (ASCENT_ERASE_ALL >> 8)) {
            // Only the original ACK may launch an erase; duplicate ACKs reuse the cached result.
            portENTER_CRITICAL(&eraseLock);
            if (eraseStatus == 1 && !eraseLaunched) {
                startErase = true;
                eraseLaunched = true;
            }
            portEXIT_CRITICAL(&eraseLock);
        }
        msp.state = MSP_IDLE;
        for (int i = 0; i < n; i++)
            msp_feed(rxbuf[i]);
    }
}
