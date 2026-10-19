#include <assert.h>
#include "../../../tools/SPI_LOG/ESP32blackboxtest/main/ascent_slave.c"

static uint8_t media[8192];
static esp_partition_t partition = { .size = sizeof(media) };
static unsigned writes, erases;
esp_err_t esp_partition_read(const esp_partition_t *p, uint32_t a, void *d, uint32_t n)
{ assert(a <= p->size && n <= p->size - a); memcpy(d, media + a, n); return ESP_OK; }
esp_err_t esp_partition_write(const esp_partition_t *p, uint32_t a, const void *d, uint32_t n)
{ assert(a <= p->size && n <= p->size - a); memcpy(media + a, d, n); writes++; return ESP_OK; }
esp_err_t esp_partition_erase_range(const esp_partition_t *p, uint32_t a, uint32_t n)
{ assert(a <= p->size && n <= p->size - a); memset(media + a, 0xFF, n); erases++; return ESP_OK; }
const esp_partition_t *esp_partition_find_first(int t, esp_partition_type_t s, const char *n)
{ (void)t; (void)s; (void)n; return &partition; }
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
    int length = msp_build(frame, cmd, p, n);
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
    store = &partition; capacity = sizeof(media) - FLASH_SECTOR;
    memset(media, 0xA5, sizeof(media));
    assert(selftest());
    for (unsigned i = 0; i < capacity; i++) assert(media[i] == 0xA5);
    puts("PASS selftest never erases advertised storage");
    uint8_t payload[260] = {0};
    wr32(payload, 128);
    for (unsigned i = 4; i < sizeof(payload); i++) payload[i] = i;
    unsigned baseline = writes;
    request(1, ASCENT_WRITE, payload, sizeof(payload));
    assert(resp_len == 9 && writes == baseline + 1);
    request(1, ASCENT_WRITE, payload, sizeof(payload));
    assert(resp_len == 9 && writes == baseline + 1);
    request(2, ASCENT_WRITE, payload, sizeof(payload));
    assert(writes == baseline + 2);
    puts("PASS repeated write sequence replays ACK without writing again");
    wr32(payload, UINT32_MAX - 3); wr32(payload + 4, 16);
    request(3, ASCENT_READ, payload, 8); assert(resp_len == 0);
    request(4, ASCENT_WRITE, payload, 20); assert(resp_len == 0);
    wr32(payload, capacity);
    request(5, ASCENT_ERASE_SECTOR, payload, 4); assert(resp_len == 0);
    puts("PASS overflow and reserved-sector access rejected");
    request(6, ASCENT_ERASE_ALL, NULL, 0);
    assert(eraseStatus == 1 && resp_len == 9);
    eraseLaunched = true;
    request(6, ASCENT_ERASE_ALL, NULL, 0);
    assert(eraseLaunched && resp_len == 9);
    request(7, ASCENT_STATUS, NULL, 0);
    assert(resp_len == 10 && respbuf[8] == 1);
    request(8, ASCENT_WRITE, payload, 20); assert(resp_len == 0);
    erase_all_task(NULL);
    request(9, ASCENT_STATUS, NULL, 0);
    assert(resp_len == 10 && respbuf[8] == 0);
    for (unsigned i = 0; i < capacity; i++) assert(media[i] == 0xFF);
    puts("PASS erase deduplication, busy write rejection and completion status");
    return 0;
}
