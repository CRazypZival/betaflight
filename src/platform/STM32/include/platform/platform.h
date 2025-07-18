/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#if defined(STM32G474xx)
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "system_stm32g4xx.h"

#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_ex.h"

// Chip Unique ID on G4
#define U_ID_0 (*(uint32_t*)UID_BASE)
#define U_ID_1 (*(uint32_t*)(UID_BASE + 4))
#define U_ID_2 (*(uint32_t*)(UID_BASE + 8))

#define SPI_TRAIT_AF_PIN 1
#define I2C_TRAIT_AF_PIN 1

#ifndef STM32G4
#define STM32G4
#endif

#elif defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H7A3xx) || defined(STM32H7A3xxQ) || defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H730xx)
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "system_stm32h7xx.h"

#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_ex.h"

// Chip Unique ID on H7
#define U_ID_0 (*(uint32_t*)UID_BASE)
#define U_ID_1 (*(uint32_t*)(UID_BASE + 4))
#define U_ID_2 (*(uint32_t*)(UID_BASE + 8))

#define SPI_TRAIT_AF_PIN 1
#define I2C_TRAIT_AF_PIN 1

#ifndef STM32H7
#define STM32H7
#endif

#elif defined(STM32F722xx) || defined(STM32F745xx) || defined(STM32F746xx) || defined(STM32F765xx)
#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include "system_stm32f7xx.h"

#include "stm32f7xx_ll_spi.h"
#include "stm32f7xx_ll_gpio.h"
#include "stm32f7xx_ll_dma.h"
#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_tim.h"
#include "stm32f7xx_ll_system.h"
#include "stm32f7xx_ll_ex.h"

// Chip Unique ID on F7
#define U_ID_0 (*(uint32_t*)UID_BASE)
#define U_ID_1 (*(uint32_t*)(UID_BASE + 4))
#define U_ID_2 (*(uint32_t*)(UID_BASE + 8))

#define SPI_TRAIT_AF_PIN 1

#ifndef STM32F7
#define STM32F7
#endif

#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F446xx)

#include "stm32f4xx.h"

// Chip Unique ID on F405
#define U_ID_0 (*(uint32_t*)0x1fff7a10)
#define U_ID_1 (*(uint32_t*)0x1fff7a14)
#define U_ID_2 (*(uint32_t*)0x1fff7a18)

#define SPI_TRAIT_AF_PORT 1

#define I2C_TRAIT_STATE 1
#define I2C_TRAIT_AF_PIN 1

#ifndef STM32F4
#define STM32F4
#endif

#endif

#ifdef STM32F4
#if defined(STM32F40_41xxx)
#define USE_FAST_DATA
#endif

#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_DYN_NOTCH_FILTER
#define USE_ADC_INTERNAL
#define USE_USB_CDC_HID
#define USE_USB_MSC
#define USE_PERSISTENT_MSC_RTC
#define USE_MCO
#define USE_MCO_DEVICE2
#define USE_DMA_SPEC
#define USE_PERSISTENT_OBJECTS
#define USE_LATE_TASK_STATISTICS

#if defined(STM32F40_41xxx) || defined(STM32F411xE)
#define USE_OVERCLOCK
#endif
#endif // STM32F4

#ifdef STM32F7
#define USE_ITCM_RAM
#define ITCM_RAM_OPTIMISATION "-O2", "-freorder-blocks-algorithm=simple"
#define USE_FAST_DATA
#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_DYN_NOTCH_FILTER
#define USE_OVERCLOCK
#define USE_ADC_INTERNAL
#define USE_USB_CDC_HID
#define USE_USB_MSC
#define USE_PERSISTENT_MSC_RTC
#define USE_MCO
#define USE_MCO_DEVICE2
#define USE_DMA_SPEC
#define USE_PERSISTENT_OBJECTS
#define USE_LATE_TASK_STATISTICS
#endif // STM32F7

#ifdef STM32H7

#ifdef USE_DSHOT
#define USE_DSHOT_CACHE_MGMT
#endif

#define USE_ITCM_RAM
#define USE_FAST_DATA
#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_DYN_NOTCH_FILTER
#define USE_ADC_INTERNAL
#define USE_USB_CDC_HID
#define USE_DMA_SPEC
#define USE_PERSISTENT_OBJECTS
#define USE_DMA_RAM
#define USE_USB_MSC
#define USE_RTC_TIME
#define USE_PERSISTENT_MSC_RTC
#define USE_LATE_TASK_STATISTICS
#endif

#ifdef STM32G4
#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_OVERCLOCK
#define USE_DYN_NOTCH_FILTER
#define USE_ADC_INTERNAL
#define USE_USB_MSC
#define USE_USB_CDC_HID
#define USE_MCO
#define USE_MCO_DEVICE1
#define USE_DMA_SPEC
#define USE_LATE_TASK_STATISTICS
#endif

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
#define TASK_GYROPID_DESIRED_PERIOD     125 // 125us = 8kHz
#define SCHEDULER_DELAY_LIMIT           10
#else
#define TASK_GYROPID_DESIRED_PERIOD     1000 // 1000us = 1kHz
#define SCHEDULER_DELAY_LIMIT           100
#endif

// Set the default cpu_overclock to the first level (108MHz) for F411
// Helps with looptime stability as the CPU is borderline when running native gyro sampling
#if defined(USE_OVERCLOCK) && defined(STM32F411xE)
#define DEFAULT_CPU_OVERCLOCK 1
#else
#define DEFAULT_CPU_OVERCLOCK 0
#endif

#if defined(STM32H7)
// Move ISRs to fast ram to avoid flash latency.
#define FAST_IRQ_HANDLER FAST_CODE
#else
#define FAST_IRQ_HANDLER
#endif

#if defined(STM32F4) || defined(STM32G4)
// F4 can't DMA to/from CCM (core coupled memory) SRAM (where the stack lives)
// On G4 there is no specific DMA target memory
#define DMA_DATA_ZERO_INIT
#define DMA_DATA
#define STATIC_DMA_DATA_AUTO        static
#elif defined(STM32F7)
// F7 has no cache coherency issues DMAing to/from DTCM, otherwise buffers must be cache aligned
#define DMA_DATA_ZERO_INIT          FAST_DATA_ZERO_INIT
#define DMA_DATA                    FAST_DATA
#define STATIC_DMA_DATA_AUTO        static DMA_DATA
#else
// DMA to/from any memory
#define DMA_DATA_ZERO_INIT          __attribute__ ((section(".dmaram_bss"), aligned(32)))
#define DMA_DATA                    __attribute__ ((section(".dmaram_data"), aligned(32)))
#define STATIC_DMA_DATA_AUTO        static DMA_DATA
#endif

#if defined(STM32F4) || defined(STM32H7)
// Data in RAM which is guaranteed to not be reset on hot reboot
#define PERSISTENT                  __attribute__ ((section(".persistent_data"), aligned(4)))
#endif

#ifdef USE_DMA_RAM
#if defined(STM32H7)
#define DMA_RAM __attribute__((section(".DMA_RAM"), aligned(32)))
#define DMA_RW_AXI __attribute__((section(".DMA_RW_AXI"), aligned(32)))
extern uint8_t _dmaram_start__;
extern uint8_t _dmaram_end__;
#elif defined(STM32G4)
#define DMA_RAM_R __attribute__((section(".DMA_RAM_R")))
#define DMA_RAM_W __attribute__((section(".DMA_RAM_W")))
#define DMA_RAM_RW __attribute__((section(".DMA_RAM_RW")))
#endif
#else
#define DMA_RAM
#define DMA_RW_AXI
#define DMA_RAM_R
#define DMA_RAM_W
#define DMA_RAM_RW
#endif

#define USE_TIMER_MGMT
#define USE_TIMER_AF

#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)

// speed is packed between modebits 4 and 1,
// 7       6        5        4         3         2        1        0
// 0 <pupd-1> <pupd-0> <mode-4> <speed-1> <speed-0> <mode-1> <mode-0>
// mode-4 is equivalent to STM32F4 otype (pushpull/od)
#define IO_CONFIG(mode, speed, pupd) ((mode) | ((speed) << 2) | ((pupd) << 5))

#define IOCFG_OUT_PP         IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_OUT_PP_UP      IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW,  GPIO_PULLUP)
#define IOCFG_OUT_PP_25      IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL)
#define IOCFG_OUT_OD         IO_CONFIG(GPIO_MODE_OUTPUT_OD, GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_AF_PP          IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_AF_PP_PD       IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLDOWN)
#define IOCFG_AF_PP_UP       IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLUP)
#define IOCFG_AF_OD          IO_CONFIG(GPIO_MODE_AF_OD,     GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_IPD            IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLDOWN)
#define IOCFG_IPU            IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLUP)
#define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_IPU_25         IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP)

#elif defined(STM32F4)

#define IO_CONFIG(mode, speed, otype, pupd) ((mode) | ((speed) << 2) | ((otype) << 4) | ((pupd) << 5))

#define IOCFG_OUT_PP         IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_PP, GPIO_PuPd_NOPULL)  // TODO
#define IOCFG_OUT_PP_UP      IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_PP, GPIO_PuPd_UP)
#define IOCFG_OUT_PP_25      IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define IOCFG_OUT_OD         IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL)
#define IOCFG_AF_PP          IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define IOCFG_AF_PP_PD       IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_PP, GPIO_PuPd_DOWN)
#define IOCFG_AF_PP_UP       IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_PP, GPIO_PuPd_UP)
#define IOCFG_AF_OD          IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_OD, GPIO_PuPd_NOPULL)
#define IOCFG_IPD            IO_CONFIG(GPIO_Mode_IN,  0, 0,             GPIO_PuPd_DOWN)
#define IOCFG_IPU            IO_CONFIG(GPIO_Mode_IN,  0, 0,             GPIO_PuPd_UP)
#define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_Mode_IN,  0, 0,             GPIO_PuPd_NOPULL)
#define IOCFG_IPU_25         IO_CONFIG(GPIO_Mode_IN,  GPIO_Speed_25MHz, 0, GPIO_PuPd_UP)

#endif

#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)

#define IO_CONFIG_GET_MODE(cfg)  (((cfg) >> 0) & 0x03)
#define IO_CONFIG_GET_SPEED(cfg) (((cfg) >> 2) & 0x03)
#define IO_CONFIG_GET_OTYPE(cfg) (((cfg) >> 4) & 0x01)
#define IO_CONFIG_GET_PULL(cfg)  (((cfg) >> 5) & 0x03)

#endif

#if defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H723xx) || defined(STM32H725xx)
#define FLASH_CONFIG_STREAMER_BUFFER_SIZE 32  // Flash word = 256-bits (8 rows, uint32_t per row - 8 x 32)
#define FLASH_CONFIG_BUFFER_TYPE uint32_t
#elif defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
#define FLASH_CONFIG_STREAMER_BUFFER_SIZE 16  // Flash word = 128-bits (4 rows, uint32_t per row - 4 x 32)
#define FLASH_CONFIG_BUFFER_TYPE uint32_t
#elif defined(STM32G4)
#define FLASH_CONFIG_BUFFER_TYPE uint64_t
#else
#define FLASH_CONFIG_BUFFER_TYPE uint32_t
#endif

#if defined(STM32F4)
#define SPI_IO_AF_CFG           IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define SPI_IO_AF_SCK_CFG       IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_DOWN)
#define SPI_IO_AF_SDI_CFG       IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define SPI_IO_CS_HIGH_CFG      IO_CONFIG(GPIO_Mode_IN,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP)
#elif defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
#define SPI_IO_AF_CFG           IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)
#define SPI_IO_AF_SCK_CFG_HIGH  IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define SPI_IO_AF_SCK_CFG_LOW   IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN)
#define SPI_IO_AF_SDI_CFG       IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)
#define SPI_IO_CS_HIGH_CFG      IO_CONFIG(GPIO_MODE_INPUT, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#else
#error "Invalid STM MCU defined - requires SPI implementation"
#endif

#if defined(STM32F4)
#define SPIDEV_COUNT 3
#elif defined(STM32F7)
#define SPIDEV_COUNT 4
#elif defined(STM32H7)
#define SPIDEV_COUNT 6
#else
#define SPIDEV_COUNT 4
#endif

// Work around different check routines in the libraries for different MCU types
#if defined(STM32H7)
#define CHECK_SPI_RX_DATA_AVAILABLE(instance) LL_SPI_IsActiveFlag_RXWNE(instance)
#define SPI_RX_DATA_REGISTER(base) ((base)->RXDR)
#else
#define CHECK_SPI_RX_DATA_AVAILABLE(instance) LL_SPI_IsActiveFlag_RXNE(instance)
#define SPI_RX_DATA_REGISTER(base) ((base)->DR)
#endif

#if defined(STM32F4) || defined(STM32G4)
#define MAX_SPI_PIN_SEL 2
#elif defined(STM32F7)
#define MAX_SPI_PIN_SEL 4
#elif defined(STM32H7)
#define MAX_SPI_PIN_SEL 5
#else
#error Unknown MCU family
#endif

#if defined(STM32F4) || defined(STM32F7)
#define USE_TX_IRQ_HANDLER
#endif

#if defined(STM32H7)
#define UART_TX_BUFFER_ATTRIBUTE DMA_RAM /* D2 SRAM */
#define UART_RX_BUFFER_ATTRIBUTE DMA_RAM /* D2 SRAM */
#elif defined(STM32G4)
#define UART_TX_BUFFER_ATTRIBUTE DMA_RAM_W /* SRAM MPU NOT_BUFFERABLE */
#define UART_RX_BUFFER_ATTRIBUTE DMA_RAM_R /* SRAM MPU NOT CACHABLE */
#elif defined(STM32F7)
#define UART_TX_BUFFER_ATTRIBUTE FAST_DATA_ZERO_INIT /* DTCM RAM */
#define UART_RX_BUFFER_ATTRIBUTE FAST_DATA_ZERO_INIT /* DTCM RAM */
#elif defined(STM32F4)
#define UART_TX_BUFFER_ATTRIBUTE /* EMPTY */
#define UART_RX_BUFFER_ATTRIBUTE /* EMPTY */
#endif

#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
 // pin AF mode is configured for each pin individually
#define UART_TRAIT_AF_PIN 1
#elif defined(STM32F4)
// all pins on given uart use same AF
#define UART_TRAIT_AF_PORT 1
#else
#error Unknown STM MCU when defining UART_TRAIT_x
#endif

#define PLATFORM_TRAIT_RCC 1

#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
#define UART_TRAIT_PINSWAP 1
#endif

#if defined(STM32F4)
#define UARTHARDWARE_MAX_PINS 4
#elif defined(STM32F7)
#define UARTHARDWARE_MAX_PINS 4
#elif defined(STM32H7)
#define UARTHARDWARE_MAX_PINS 6
#elif defined(STM32G4)
#define UARTHARDWARE_MAX_PINS 3
#endif

#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
#define UART_REG_RXD(base) ((base)->RDR)
#define UART_REG_TXD(base) ((base)->TDR)
#elif defined(STM32F4)
#define UART_REG_RXD(base) ((base)->DR)
#define UART_REG_TXD(base) ((base)->DR)
#endif


#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
#define DMA_TRAIT_CHANNEL 1
#endif

#define SERIAL_TRAIT_PIN_CONFIG 1
#define USB_DP_PIN PA12

#if defined(USE_HAL_DRIVER)
#define I2C_TRAIT_HANDLE 1

#if defined(HAL_SPI_MODULE_ENABLED)
#define SPI_TRAIT_HANDLE 1
#endif

#endif

#if defined(STM32F4)
#define I2CDEV_COUNT 3
#else
#define I2CDEV_COUNT 4
#endif

// QUAD SPI
#if defined(STM32H7)
#define QUADSPI_TRAIT_AF_PIN 1
#define QUADSPI_TRAIT_HANDLE 1
#define MAX_QUADSPI_PIN_SEL 3
#define PLATFORM_TRAIT_SDIO_INIT 1
#endif

// F4 has non-8MHz boards
// G4 for Betaflight allow 8, 16, 24, 26 or 27MHz oscillator
#if defined(STM32F4) || defined(STM32G4)
#define PLATFORM_TRAIT_CONFIG_HSE 1
#endif

#if defined(STM32G4)
#define MCO_SOURCE_COUNT   8
#define MCO_DIVIDER_COUNT  5
#endif

#if defined(STM32H7) || defined(STM32G4)
#define DMA_CHANREQ_STRING "Request"
#endif

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7)
#define DMA_STCH_STRING    "Stream"
#endif

#ifdef USE_ITCM_RAM
#if defined(ITCM_RAM_OPTIMISATION) && !defined(DEBUG)
#define FAST_CODE                   __attribute__((section(".tcm_code"))) __attribute__((optimize(ITCM_RAM_OPTIMISATION)))
#else
#define FAST_CODE                   __attribute__((section(".tcm_code")))
#endif
// If a particular target is short of ITCM RAM, defining FAST_CODE_PREF in the target.h file will
// cause functions decorated FAST_CODE_PREF to *not* go into ITCM RAM but if FAST_CODE_PREF is not
// defined for the target, FAST_CODE_PREF will become an alias to FAST_CODE (in the common post
// header file), and functions decorated with FAST_CODE_PREF *will* go into ITCM RAM.

#define FAST_CODE_NOINLINE          NOINLINE
#endif // USE_ITCM_RAM

// noting this is not used anywhere in the codebase at the moment
#ifdef USE_CCM_CODE
#define CCM_CODE                    __attribute__((section(".ccm_code")))
#endif

#ifdef USE_FAST_DATA
#define FAST_DATA_ZERO_INIT         __attribute__ ((section(".fastram_bss"), aligned(4)))
#define FAST_DATA                   __attribute__ ((section(".fastram_data"), aligned(4)))
#endif // USE_FAST_DATA

#if defined(USE_EXST) && !defined(RAMBASED)
#define USE_FLASH_BOOT_LOADER
#endif

#if defined(USE_FLASH_MEMORY_MAPPED)
#if !defined(USE_RAM_CODE)
#define USE_RAM_CODE
#endif

#define MMFLASH_CODE RAM_CODE
#define MMFLASH_CODE_NOINLINE RAM_CODE NOINLINE

#define MMFLASH_DATA FAST_DATA
#define MMFLASH_DATA_ZERO_INIT FAST_DATA_ZERO_INIT
#endif

#ifdef USE_RAM_CODE
// RAM_CODE for methods that need to be in RAM, but don't need to be in the fastest type of memory.
// Note: if code is marked as RAM_CODE it *MUST* be in RAM, there is no alternative unlike functions marked with FAST_CODE/CCM_CODE
#define RAM_CODE                   __attribute__((section(".ram_code")))
#endif
