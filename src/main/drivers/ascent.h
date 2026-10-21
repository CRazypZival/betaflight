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

#pragma once

#ifdef USE_ASCENT

#include <stdbool.h>
#include <stdint.h>

/* 单帧推送给 SOC 的日志块上限（MSP 负载） */
#define ASCENT_CHUNK 256
/* 流缓冲大小（不超过该值的连续写入总能入队） */
#define ASCENT_RING_SIZE 4096

/* 初始化 SPI 与 CS（由 init.c 在 blackboxInit 之前调用） */
void ascentInit(void);
/* 每圈调用：把流缓冲按帧推给 SOC（单向，无应答） */
void ascentUpdate(void);
/* 追加日志字节；缓冲满时丢弃新数据 */
void ascentWrite(const uint8_t *data, int len);
/* 流缓冲剩余空间 */
int ascentWriteFree(void);
/* 入队数据是否已全部交付给 SPI 硬件 */
bool ascentFlushed(void);
bool ascentIsAvailable(void);

#include <string.h>
static inline void ascentWriteByte(uint8_t value)
{
    ascentWrite(&value, 1);
}
#endif
