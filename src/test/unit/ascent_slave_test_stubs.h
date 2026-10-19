#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#define DMA_ATTR
#define ESP_OK 0
#define ESP_LOGI(tag, ...) do { (void)(tag); if (0) printf(__VA_ARGS__); } while (0)
#define ESP_LOGD ESP_LOGI
#define ESP_LOGW ESP_LOGI
#define ESP_LOGE ESP_LOGI
#define ESP_ERROR_CHECK(x) assert((x) == ESP_OK)
#define portENTER_CRITICAL(p) (void)(p)
#define portEXIT_CRITICAL(p) (void)(p)
#define portMUX_INITIALIZER_UNLOCKED 0
#define portMAX_DELAY 0
#define pdPASS 1
#define ESP_PARTITION_TYPE_DATA 1
#define SPI2_HOST 2
#define SPI_DMA_CH_AUTO 0
typedef int esp_err_t;
typedef int esp_partition_type_t;
typedef int portMUX_TYPE;
typedef struct { uint32_t size, address; const char *label; } esp_partition_t;
typedef struct { int mosi_io_num, miso_io_num, sclk_io_num, quadwp_io_num, quadhd_io_num; } spi_bus_config_t;
typedef struct { int mode, spics_io_num, queue_size; } spi_slave_interface_config_t;
typedef struct { uint32_t length, trans_len; void *tx_buffer, *rx_buffer, *user; } spi_slave_transaction_t;
esp_err_t esp_partition_read(const esp_partition_t *, uint32_t, void *, uint32_t);
esp_err_t esp_partition_write(const esp_partition_t *, uint32_t, const void *, uint32_t);
esp_err_t esp_partition_erase_range(const esp_partition_t *, uint32_t, uint32_t);
const esp_partition_t *esp_partition_find_first(int, esp_partition_type_t, const char *);
const char *esp_err_to_name(esp_err_t);
void vTaskDelete(void *);
void vTaskDelay(unsigned);
int xTaskCreate(void (*)(void *), const char *, unsigned, void *, unsigned, void *);
int spi_slave_initialize(int, const spi_bus_config_t *, const spi_slave_interface_config_t *, int);
int spi_slave_queue_trans(int, spi_slave_transaction_t *, int);
int spi_slave_get_trans_result(int, spi_slave_transaction_t **, int);
