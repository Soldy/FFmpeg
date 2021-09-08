/*
 * FFMPEG HEVC decoder hardware accelerator
 * Copyright (C) 2019 Raspberry Pi Ltd
 *
 * Heavily modified by Florian Wesch <fw@info-beamer.com>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef LIBAVCODEC_RPI_ARGON_H
#define LIBAVCODEC_RPI_ARGON_H

#include <stdint.h>

#define MAX_THREADS 50

#if   RPI_DEBUG == 2
#define v(...)  fprintf(stderr, "[rpi-hevc] "__VA_ARGS__)
#define vv(...) fprintf(stderr, "[rpi-hevc] "__VA_ARGS__)
#elif RPI_DEBUG == 1
#define v(...)  fprintf(stderr, "[rpi-hevc] "__VA_ARGS__)
#define vv(...)
#else
#define v(...)
#define vv(...)
#endif

typedef struct argon_s argon_t;

const char *argon_ref                 (void);
void        argon_unref               (void);
int         argon_reserve_thread_slot (void* key);
int         argon_find_thread_slot    (void* key);
void        argon_release_thread_slot (int idx);
void        argon_lock_phase          (int n);
void        argon_unlock_phase        (int n);
void        argon_decoder_lock        (void);
void        argon_decoder_unlock      (void);

void        rpi_apb_write_addr    (uint16_t addr, uint32_t data);
void        rpi_apb_write         (uint16_t addr, uint32_t data);
uint32_t    rpi_apb_read          (uint16_t addr);
void        rpi_apb_read_drop     (uint16_t addr);
void        rpi_axi_write         (uint64_t addr, uint32_t size, const void *buf);
void        rpi_axi_read          (uint64_t addr, uint32_t size, void *buf);
void        rpi_wait_interrupt    (int phase);
uint64_t    rpi_axi_get_addr      (void);
void        rpi_apb_dump_regs     (uint16_t addr, int num);
void        rpi_axi_dump          (uint64_t addr, uint32_t size);
void        rpi_axi_flush         (int mode);

#endif
