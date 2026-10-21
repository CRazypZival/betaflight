/*
 * ASCENT 测试固件 - ESP32-S3 SPI 从机端（v2 流式）
 * 接收 BF 推送的原始 log 字节流（MSP v2 DATA 帧），追加存储到自带 flash 分区。
 * 满了丢弃新数据（STATUS 可查）；数据断电保留；导出 = 读分区 HEADER_SIZE 起的前 length 字节。
 * 协议契约见 packs/SPI_MSP协议.txt
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_slave.h"
#include "esp_log.h"
#include "esp_partition.h"

/* 接线（BF 主机 ↔ 本机从机），可改 */
#define PIN_MOSI 10  /* BF→ESP */
#define PIN_MISO 11  /* ESP→BF */
#define PIN_SCK  12
#define PIN_CS   13

#define TRANS_BYTES     512
#define FLASH_SECTOR    4096
#define MSP_MAX_PAYLOAD 1024
#define DATA_MAX        256
#define HEADER_SIZE     FLASH_SECTOR     /* 分区首扇区存头：magic + 已写长度 */
#define HDR_MAGIC       0x42435341u      /* "ASCB" */

/* 命令 ID，与 BF ascent.c 同步 */
#define ASCENT_PING   0x5A01
#define ASCENT_DATA   0x5A10
#define ASCENT_STATUS 0x5A07

static const char *TAG = "ascent";

static const esp_partition_t *store;
static uint32_t capacity;      /* 数据区容量 = 分区大小 - 头扇区 */
static uint32_t written;       /* 已写入字节数（持久化于头扇区） */
static bool storageFull;

/* DMA 缓冲必须常驻内部 RAM */
DMA_ATTR static uint8_t rxbuf[TRANS_BYTES];
DMA_ATTR static uint8_t txbuf[TRANS_BYTES];

/* ---------- 存储层 ---------- */

static uint8_t pendingChunks;   /* 距上次持久化长度的块数 */

static bool storage_load(void)
{
    struct { uint32_t magic, length; } hdr;
    if (esp_partition_read(store, 0, &hdr, sizeof(hdr)) != ESP_OK)
        return false;
    if (hdr.magic == HDR_MAGIC && hdr.length <= capacity) {
        written = hdr.length;
        return true;
    }
    esp_partition_erase_range(store, 0, HEADER_SIZE);
    const uint32_t init[2] = { HDR_MAGIC, 0 };
    if (esp_partition_write(store, 0, init, sizeof(init)) != ESP_OK)
        return false;
    written = 0;
    return true;
}

static void storage_persist_length(void)
{
    esp_partition_erase_range(store, 0, HEADER_SIZE);
    const uint32_t hdr[2] = { HDR_MAGIC, written };
    esp_partition_write(store, 0, hdr, sizeof(hdr));
}

static void storage_append(const uint8_t *data, uint16_t n)
{
    if (storageFull || written + n > capacity) {
        storageFull = true;
        ESP_LOGW(TAG, "full, dropping %u bytes", n);
        return;                          /* 满：丢弃，仍回 ACK */
    }
    if (esp_partition_write(store, HEADER_SIZE + written, data, n) != ESP_OK) {
        ESP_LOGW(TAG, "write fail at %u", (unsigned)written);
        return;
    }
    written += n;
    if (++pendingChunks >= 16) {         /* 每 4KB 数据持久化一次长度 */
        pendingChunks = 0;
        storage_persist_length();
    }
}

/* ---------- MSP v2 ---------- */

static uint8_t crc8_dvb_s2(uint8_t crc, const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    return crc;
}

static uint8_t respbuf[TRANS_BYTES];
static int resp_len;

static int msp_build(uint8_t *out, uint8_t flag, uint16_t cmd, const uint8_t *payload, uint16_t len)
{
    out[0] = '$'; out[1] = 'X'; out[2] = '>'; out[3] = flag;
    out[4] = cmd & 0xFF; out[5] = cmd >> 8;
    out[6] = len & 0xFF; out[7] = len >> 8;
    if (len)
        memcpy(&out[8], payload, len);
    out[8 + len] = crc8_dvb_s2(0, &out[3], 5 + len);
    return 9 + len;
}

static struct {
    int state;
    uint8_t crc, flag;
    uint16_t cmd, len, got;
    uint8_t payload[MSP_MAX_PAYLOAD];
} msp;

/* 按序列号缓存最近应答：BF 重发同一请求时直接回放缓存，不重复落盘 */
static struct {
    bool valid;
    uint8_t flag;
    uint16_t cmd, len;
    uint8_t payload[MSP_MAX_PAYLOAD];
    uint8_t response[TRANS_BYTES];
    int responseLength;
} lastRequest;

/* 返回应答帧长度；0 = 不应答 */
static int handle_cmd(uint16_t cmd, const uint8_t *p, uint16_t len, uint8_t *resp)
{
    switch (cmd) {
    case ASCENT_PING:
        if (len)
            return 0;
        ESP_LOGI(TAG, "PING written=%u%s", (unsigned)written, storageFull ? " (FULL)" : "");
        return msp_build(resp, msp.flag, cmd, (const uint8_t *)"ASCENT01", 8);

    case ASCENT_DATA:
        if (len < 1 || len > DATA_MAX)
            return 0;
        storage_append(p, len);
        return msp_build(resp, msp.flag, cmd, NULL, 0);

    case ASCENT_STATUS:
        if (len)
            return 0;
        return msp_build(resp, msp.flag, cmd, (const uint8_t[]){ storageFull ? 1 : 0 }, 1);

    default:
        ESP_LOGW(TAG, "unknown cmd 0x%04x", cmd);
        return 0;
    }
}

enum { P_IDLE, P_MAGIC, P_MAGIC2, P_DIR, P_FLAG, P_CMD_LO, P_CMD_HI,
       P_LEN_LO, P_LEN_HI, P_PAYLOAD, P_CRC };

/* 喂一个收到的字节；完整且 CRC 正确的请求帧在此产生应答 */
static void msp_feed(uint8_t b)
{
    switch (msp.state) {
    case P_IDLE:
        if (b == '$') { msp.state = P_MAGIC; msp.crc = 0; }
        break;
    case P_MAGIC:
        msp.state = (b == 'X') ? P_MAGIC2 : P_IDLE;
        break;
    case P_MAGIC2:
        msp.state = (b == '<') ? P_FLAG : P_IDLE;
        break;
    case P_FLAG:
        msp.flag = b;
        msp.crc = crc8_dvb_s2(msp.crc, &b, 1);
        msp.state = P_CMD_LO;
        break;
    case P_CMD_LO:
        msp.cmd = b;
        msp.crc = crc8_dvb_s2(msp.crc, &b, 1);
        msp.state = P_CMD_HI;
        break;
    case P_CMD_HI:
        msp.cmd |= b << 8;
        msp.crc = crc8_dvb_s2(msp.crc, &b, 1);
        msp.state = P_LEN_LO;
        break;
    case P_LEN_LO:
        msp.len = b;
        msp.crc = crc8_dvb_s2(msp.crc, &b, 1);
        msp.state = P_LEN_HI;
        break;
    case P_LEN_HI:
        msp.len |= b << 8;
        msp.crc = crc8_dvb_s2(msp.crc, &b, 1);
        if (msp.len > MSP_MAX_PAYLOAD) {
            ESP_LOGW(TAG, "payload too big %u", msp.len);
            msp.state = P_IDLE;
        } else {
            msp.got = 0;
            msp.state = msp.len ? P_PAYLOAD : P_CRC;
        }
        break;
    case P_PAYLOAD:
        msp.payload[msp.got++] = b;
        msp.crc = crc8_dvb_s2(msp.crc, &b, 1);
        if (msp.got == msp.len)
            msp.state = P_CRC;
        break;
    case P_CRC:
        if (b == msp.crc) {
            if (lastRequest.valid && lastRequest.flag == msp.flag && lastRequest.cmd == msp.cmd
                && lastRequest.len == msp.len && memcmp(lastRequest.payload, msp.payload, msp.len) == 0) {
                /* 序列号+内容相同的重发：回放缓存应答，不重复落盘 */
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
        } else {
            ESP_LOGW(TAG, "crc err cmd=0x%04x", msp.cmd);
        }
        msp.state = P_IDLE;
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

    esp_partition_erase_range(store, addr, FLASH_SECTOR);
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
    capacity = store->size - HEADER_SIZE - FLASH_SECTOR;   /* 末扇区留给自检 */

    if (!storage_load()) {
        ESP_LOGE(TAG, "storage header init fail");
        return;
    }
    if (!selftest()) {
        ESP_LOGE(TAG, "selftest FAIL");
        return;
    }
    ESP_LOGI(TAG, "selftest PASS, resuming at offset=%u, capacity=%u", (unsigned)written, (unsigned)capacity);

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
        ESP_ERROR_CHECK(spi_slave_get_trans_result(SPI2_HOST, &ret, portMAX_DELAY));

        int n = ret->trans_len / 8;
        msp.state = P_IDLE;
        for (int i = 0; i < n; i++)
            msp_feed(rxbuf[i]);
    }
}
