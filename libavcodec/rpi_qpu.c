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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <stdint.h>
#include "libavutil/avassert.h"

#include "config.h"

#include <pthread.h>
#include <time.h>

#include <interface/vcsm/user-vcsm.h>

#include "rpi_mailbox.h"
#include "rpi_qpu.h"
#include "rpi_argon.h"

#pragma GCC diagnostic push
// Many many redundant decls in the header files
#pragma GCC diagnostic ignored "-Wredundant-decls"
#include "interface/vmcs_host/vc_vchi_gpuserv.h"
#pragma GCC diagnostic pop

typedef struct gpu_env_s {
  int open_count;
  int mb;
} gpu_env_t;

// Stop more than one thread trying to allocate memory or use the processing resources at once
static pthread_mutex_t gpu_mutex = PTHREAD_MUTEX_INITIALIZER;
static gpu_env_t * gpu = NULL;

static int gpu_malloc_cached_internal(const int mb, const int numbytes, GPU_MEM_PTR_T * const p) { p->numbytes = (numbytes + 255) & ~255;  // Round up
    p->vcsm_handle = vcsm_malloc_cache(p->numbytes, VCSM_CACHE_TYPE_HOST | 0x80, (char *)"Video Frame" );
    //p->vcsm_handle = vcsm_malloc_cache(numbytes, VCSM_CACHE_TYPE_VC, (char *)"Video Frame" );
    //p->vcsm_handle = vcsm_malloc_cache(numbytes, VCSM_CACHE_TYPE_NONE, (char *)"Video Frame" );
    //p->vcsm_handle = vcsm_malloc_cache(numbytes, VCSM_CACHE_TYPE_HOST_AND_VC, (char *)"Video Frame" );
    av_assert0(p->vcsm_handle);
    p->vc_handle = vcsm_vc_hdl_from_hdl(p->vcsm_handle);
    av_assert0(p->vc_handle);
    p->arm = vcsm_lock(p->vcsm_handle);
    av_assert0(p->arm);
    p->vc = mbox_mem_lock(mb, p->vc_handle);
    av_assert0(p->vc);
    // printf("***** %s, %d\n", __func__, numbytes);
    return 0;
}

static int gpu_malloc_uncached_internal(const int mb, const int numbytes, GPU_MEM_PTR_T * const p) { p->numbytes = numbytes;
    p->vcsm_handle = vcsm_malloc_cache(numbytes, VCSM_CACHE_TYPE_NONE | 0x80, (char *)"Video Frame" );
    av_assert0(p->vcsm_handle);
    p->vc_handle = vcsm_vc_hdl_from_hdl(p->vcsm_handle);
    av_assert0(p->vc_handle);
    p->arm = vcsm_lock(p->vcsm_handle);
    av_assert0(p->arm);
    p->vc = mbox_mem_lock(mb, p->vc_handle);
    av_assert0(p->vc);
    // printf("***** %s, %d\n", __func__, numbytes);
    return 0;
}

static void gpu_free_internal(const int mb, GPU_MEM_PTR_T * const p) {
  mbox_mem_unlock(mb, p->vc_handle);
  vcsm_unlock_ptr(p->arm);
  vcsm_free(p->vcsm_handle);
  memset(p, 0, sizeof(*p));  // Ensure we crash hard if we try and use this again
  // printf("***** %s\n", __func__);
}

static void gpu_term(void) {
    gpu_env_t * const ge = gpu;
    v("GPU term\n");

    // We have to hope that eveything has terminated...
    gpu = NULL;

    mbox_close(ge->mb);
    free(ge);
}

static int gpu_init(gpu_env_t ** const gpu) {
    gpu_env_t * const ge = calloc(1, sizeof(gpu_env_t));
    v("GPU init\n");
    *gpu = NULL;

    if (ge == NULL)
        return -1;

    if ((ge->mb = mbox_open()) < 0)
        return -1;

    *gpu = ge;
    return 0;
}

static void gpu_unlock(void) {
    pthread_mutex_unlock(&gpu_mutex);
}

static gpu_env_t * gpu_lock(void) {
    pthread_mutex_lock(&gpu_mutex);
    av_assert0(gpu != NULL);
    return gpu;
}

static gpu_env_t * gpu_lock_ref(void) {
    pthread_mutex_lock(&gpu_mutex);

    if (gpu == NULL) {
        int rv = gpu_init(&gpu);
        if (rv != 0) {
            gpu_unlock();
            return NULL;
        }
    }

    ++gpu->open_count;
    // fprintf(stderr, "REF: use count is now %d\n", gpu->open_count);
    return gpu;
}

static void gpu_unlock_unref(gpu_env_t * const ge) {
    // fprintf(stderr, "UNREF: use count is now %d\n", ge->open_count-1);
    if (--ge->open_count == 0)
        gpu_term();
    gpu_unlock();
}

static inline gpu_env_t * gpu_ptr(void) {
    av_assert0(gpu != NULL);
    return gpu;
}

// Public gpu fns

// Allocate memory on GPU
// Fills in structure <p> containing ARM pointer, videocore handle, videocore memory address, numbytes
// Returns 0 on success.
// This allocates memory that will not be cached in ARM's data cache.
// Therefore safe to use without data cache flushing.
int gpu_malloc_uncached(int numbytes, GPU_MEM_PTR_T *p) {
    int r;
    gpu_env_t * const ge = gpu_lock_ref();
    if (ge == NULL)
        return -1;
    r = gpu_malloc_uncached_internal(ge->mb, numbytes, p);
    gpu_unlock();
    return r;
}

// This allocates data that will be
//    Cached in ARM L2
//    Uncached in VPU L2
int gpu_malloc_cached(int numbytes, GPU_MEM_PTR_T *p) {
    int r;
    gpu_env_t * const ge = gpu_lock_ref();
    if (ge == NULL)
        return -1;
    r = gpu_malloc_cached_internal(ge->mb, numbytes, p);
    gpu_unlock();
    return r;
}

void gpu_free(GPU_MEM_PTR_T * const p) {
    gpu_env_t * const ge = gpu_lock();
    gpu_free_internal(ge->mb, p);
    gpu_unlock_unref(ge);
}

int gpu_get_mailbox(void) {
    av_assert0(gpu);
    return gpu->mb;
}

void gpu_ref(void) {
    gpu_lock_ref();
    gpu_unlock();
}

void gpu_unref(void) {
    gpu_env_t * const ge = gpu_lock();
    gpu_unlock_unref(ge);
}
