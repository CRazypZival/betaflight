#include <assert.h>
#include "../../../tools/SPI_LOG/ESP32blackboxtest/main/ascent_slave.c"

static uint8_t media[16384];
static esp_partition_t partition = { .size = sizeof(media) };
static unsigned writes;
esp_err_t esp_partition_read(const esp_partition_t *p, uint32_t a, void *d, uint32_t n)
{ assert(a <= p->size && n <= p->size - a); memcpy(d, media + a, n); return ESP_OK; }
esp_err_t esp_partition_write(const esp_partition_t *p, uint32_t a, const void *d, uint32_t n)
{ assert(a <= p->size && n <= p->size - a); memcpy(media + a, d, n); writes++; return ESP_OK; }
esp_err_t esp_partition_erase_range(const esp_partition_t *p, uint32_t a, uint32_t n)
{ assert(a <= p->size && n <= p->size - a); memset(media + a, 0xFF, n); return ESP_OK; }
const esp_partition_t *esp_partition_find_first(int t, esp_partition_type_t st, const char *n)
{ (void)t; (void)st; (void)n; return &partition; }
const char *esp_err_to_name(esp_err_t e) { (void)e; return "OK"; }
void vTaskDelete(void *p) { (void)p; }
void vTaskDelay(unsigned n) { (void)n; }
int xTaskCreate(void (*f)(void *), const char *n, unsigned s, void *p, unsigned pri, void *h)
{ (void)f; (void)n; (void)s; (void)p; (void)pri; (void)h; return pdPASS; }
int spi_slave_initialize(int h, const spi_bus_config_t *b, const spi_slave_interface_config_t *s, int d)
{ (void)h; (void)b; (void)s; (void)d; return 0; }
int spi_slave_queue_trans(int h, spi_slave_transaction_t *t, int d)
{ (void)h; (void)t; (void)d; return 0; }
int spi_slave_get_trans_result(int h, spi_slave_transaction_t **t, int d)
{ (void)h; (void)t; (void)d; return 0; }

static void request(uint8_t seq, uint16_t cmd, const uint8_t *p, uint16_t n)
{
    uint8_t frame[512];
    msp.flag = seq;
    int length = msp_build(frame, seq, cmd, p, n);
    frame[2] = '<';
    resp_len = 0;
    for (int i = 0; i < length; i++) msp_feed(frame[i]);
    if (resp_len) {
        assert(respbuf[3] == seq);
        assert(crc8_dvb_s2(0, respbuf + 3, resp_len - 4) == respbuf[resp_len - 1]);
    }
}

int main(void)
{
    store = &partition;
    capacity = sizeof(media) - HEADER_SIZE - FLASH_SECTOR;
    memset(media, 0xA5, sizeof(media));
    assert(storage_load());
    assert(written == 0);
    assert(((uint32_t *)media)[0] == HDR_MAGIC);
    puts("PASS storage header initialised");

    uint8_t payload[64];
    for (unsigned i = 0; i < sizeof(payload); i++) payload[i] = (uint8_t)(i * 3);
    printf("[dbg] full=%d written=%u capacity=%u HEADER=%d\n", storageFull, written, capacity, HEADER_SIZE);
    unsigned baseline = writes;
    request(1, ASCENT_DATA, payload, sizeof(payload));
    assert(resp_len == 9 && memcmp(media + HEADER_SIZE, payload, sizeof(payload)) == 0);
    request(1, ASCENT_DATA, payload, sizeof(payload));
    assert(resp_len == 9 && writes == baseline + 1);
    request(2, ASCENT_DATA, payload, sizeof(payload));
    assert(writes == baseline + 2 && written == 2 * sizeof(payload));
    puts("PASS DATA appends once per sequence, replay-safe");

    request(3, ASCENT_PING, NULL, 0);
    assert(resp_len == 17 && memcmp(respbuf + 8, "ASCENT01", 8) == 0);
    request(4, ASCENT_STATUS, NULL, 0);
    assert(resp_len == 10 && respbuf[8] == 0);
    puts("PASS PING magic and STATUS ready");

    storageFull = true;
    uint32_t before = written;
    request(5, ASCENT_DATA, payload, sizeof(payload));
    assert(resp_len == 9 && written == before);
    request(6, ASCENT_STATUS, NULL, 0);
    assert(resp_len == 10 && respbuf[8] == 1);
    puts("PASS full storage drops data, keeps ACKing, reports via STATUS");

    storageFull = false;
    for (unsigned i = 0; i < 16; i++) {
        storage_append(payload, 64);
    }
    assert(written == before + 16 * 64);
    storage_persist_length();
    uint32_t hdr[2];
    esp_partition_read(store, 0, hdr, sizeof(hdr));
    assert(hdr[0] == HDR_MAGIC && hdr[1] == written);
    puts("PASS length persisted to header sector");
    return 0;
}
