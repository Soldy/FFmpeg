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

#ifndef LIBAVCODEC_RPI_QPU_H
#define LIBAVCODEC_RPI_QPU_H

#include "libavutil/frame.h"
#include "libavutil/avassert.h"

typedef struct gpu_mem_ptr_s {
  unsigned char *arm;    // Pointer to memory mapped on ARM side
  int vc_handle;         // Videocore handle of relocatable memory
  int vcsm_handle;       // Handle for use by VCSM
  unsigned int vc;       // Address for use in GPU code
  unsigned int numbytes; // Size of memory block
} GPU_MEM_PTR_T;

int  gpu_malloc_cached(int numbytes, GPU_MEM_PTR_T *p);
int  gpu_malloc_uncached(int numbytes, GPU_MEM_PTR_T *p);
void gpu_free(GPU_MEM_PTR_T * const p);

int  gpu_get_mailbox(void);
void gpu_ref(void);
void gpu_unref(void);

static inline int gpu_is_buf1(const AVFrame * const frame) {
    return frame->buf[1] == NULL;
}

static inline GPU_MEM_PTR_T * gpu_buf1_gmem(const AVFrame * const frame) {
    return av_buffer_get_opaque(frame->buf[0]);
}

static inline uint32_t get_vc_address3(const AVFrame * const frame, const unsigned int n) {
    av_assert0(gpu_is_buf1(frame));
    const GPU_MEM_PTR_T * const gm = gpu_buf1_gmem(frame);
    return gm->vc + (frame->data[n] - gm->arm);
}

static inline uint32_t get_vc_address_y(const AVFrame * const frame) {
    return get_vc_address3(frame, 0);
}

static inline uint32_t get_vc_address_u(const AVFrame * const frame) {
    return get_vc_address3(frame, 1);
}

static inline uint32_t get_vc_address_v(const AVFrame * const frame) {
    return get_vc_address3(frame, 2);
}

#endif
