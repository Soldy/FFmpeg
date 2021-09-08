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
#include <stdint.h>
#include <inttypes.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <fcntl.h>
#include <assert.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>

#include "libavutil/avassert.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wredundant-decls"
#include <interface/vcsm/user-vcsm.h>
#include <bcm_host.h>
#pragma GCC diagnostic pop

#include "rpi_qpu.h"
#include "rpi_argon.h"
#include "rpi_mailbox.h"

// argon block doesn't see VC sdram alias bits
#define MANGLE(x) ((x) &~0xc0000000)
#define AXI_MEM_SIZE (64*1024*1024)

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (0x10000)
#define VERBOSE 0

static void yield(void) { usleep(1000); }

// Set up a memory regions to access periperhals
static void *setup_io(const char *dev, int *fd) {
   void *gpio_map;
   int  mem_fd;

   if ((mem_fd = open(dev, O_RDWR|O_SYNC) ) < 0) {
      printf("can't open %s\n", dev);
      exit (-1);
   }

   gpio_map = (unsigned char *)mmap(
      NULL,
      BLOCK_SIZE,
      PROT_READ|PROT_WRITE,
      MAP_SHARED,
      mem_fd,
      0 
   );

   fprintf(stderr, "%s: %p (fd:%d)\n", __FUNCTION__, gpio_map, mem_fd);

   if (gpio_map == MAP_FAILED) {
      printf("mmap error %p\n", gpio_map);
      exit (-1);
   }

   *fd = mem_fd;
   return gpio_map;
}

static void release_io(void *gpio_map) {
   int s = munmap(gpio_map, BLOCK_SIZE);
   if (s != 0) {
      printf("unmmap error %p\n", gpio_map);
      exit(1);
   }
}

struct argon_s {
    int ref_count;
    GPU_MEM_PTR_T axi;
    int apb_fd;
    volatile unsigned int *apb;
    int interrupt_fd;
    volatile unsigned int *interrupt;
    int mbox_fd;
    void *thread_slots[MAX_THREADS];
    pthread_mutex_t mutex_phase[2];
    pthread_mutex_t decoder_lock;
};

static argon_t argon = {0};
static pthread_mutex_t argon_lock = PTHREAD_MUTEX_INITIALIZER;

int argon_reserve_thread_slot(void *key) {
    pthread_mutex_lock(&argon_lock);
    int idx;
    for (idx = 0; idx < MAX_THREADS; idx++) {
        if (argon.thread_slots[idx] == NULL)
            break;
    }
    av_assert0(idx < MAX_THREADS);
    argon.thread_slots[idx] = key;
    v("argon slot reserved %d\n", idx);
    pthread_mutex_unlock(&argon_lock);
    return idx;
}

int argon_find_thread_slot(void *key) {
    pthread_mutex_lock(&argon_lock);
    int idx;
    for (idx = 0; idx < MAX_THREADS; idx++) {
        if (argon.thread_slots[idx] == key)
            break;
    }
    av_assert0(idx < MAX_THREADS);
    pthread_mutex_unlock(&argon_lock);
    return idx;
}

void argon_release_thread_slot(int idx) {
    pthread_mutex_lock(&argon_lock);
    argon.thread_slots[idx] = NULL;
    v("argon slot released %d\n", idx);
    pthread_mutex_unlock(&argon_lock);
}

void argon_decoder_lock() {
    // pthread_mutex_lock(&argon.decoder_lock);
}

void argon_decoder_unlock() {
    // pthread_mutex_unlock(&argon.decoder_lock);
}

void argon_lock_phase(int n) {
    pthread_mutex_lock(&argon.mutex_phase[n]);
    v("locked phase %d\n", n);
}

void argon_unlock_phase(int n) {
    v("unlock phase %d\n", n);
    pthread_mutex_unlock(&argon.mutex_phase[n]);
}

//////////////////////////////////////////////////////////////////////////////

void rpi_apb_write_addr(uint16_t addr, uint32_t data) {
    vv("P %x %08x\n", addr, data);
    argon.apb[addr>>2] = data + (MANGLE(argon.axi.vc)>>6);
}

uint64_t rpi_axi_get_addr() {
    return (uint64_t)MANGLE(argon.axi.vc);
}

void rpi_apb_write(uint16_t addr, uint32_t data) {
    vv("W %x %08x\n", addr, data);
    argon.apb[addr>>2] = data;
}

uint32_t rpi_apb_read(uint16_t addr) {
    uint32_t v = argon.apb[addr>>2];
    vv("R %x (=%x)\n", addr, v);
    return v;
}

void rpi_apb_read_drop(uint16_t addr) {
    uint32_t v = argon.apb[addr>>2];
    vv("R %x (=%x)\n", addr, v);
}

void rpi_axi_write(uint64_t addr, uint32_t size, const void *buf) {
    vv("L %08" PRIx64 " %08x\n", addr, size);
    assert(addr + size <= AXI_MEM_SIZE);
    memcpy(argon.axi.arm + addr, buf, size);
}

void rpi_wait_interrupt(int phase) {
    vv("I %d\n", phase);

    #define ARG_IC_ICTRL_ACTIVE1_INT_SET                   0x00000001
    #define ARG_IC_ICTRL_ACTIVE1_EDGE_SET                  0x00000002
    #define ARG_IC_ICTRL_ACTIVE1_EN_SET                    0x00000004
    #define ARG_IC_ICTRL_ACTIVE1_STATUS_SET                0x00000008
    #define ARG_IC_ICTRL_ACTIVE2_INT_SET                   0x00000010
    #define ARG_IC_ICTRL_ACTIVE2_EDGE_SET                  0x00000020
    #define ARG_IC_ICTRL_ACTIVE2_EN_SET                    0x00000040
    #define ARG_IC_ICTRL_ACTIVE2_STATUS_SET                0x00000080

    if (phase == 1) {
      while (!(argon.interrupt[0] & ARG_IC_ICTRL_ACTIVE1_INT_SET))
        yield();
      argon.interrupt[0] = argon.interrupt[0] &~ ARG_IC_ICTRL_ACTIVE2_INT_SET; //ARG_IC_ICTRL_ACTIVE1_INT_SET|ARG_IC_ICTRL_ACTIVE2_EDGE_SET|ARG_IC_ICTRL_ACTIVE2_EDGE_SET;
    } else if (phase == 2) {
      while (!(argon.interrupt[0] & ARG_IC_ICTRL_ACTIVE2_INT_SET))
        yield();
      argon.interrupt[0] = argon.interrupt[0] &~ ARG_IC_ICTRL_ACTIVE1_INT_SET; //ARG_IC_ICTRL_ACTIVE2_INT_SET|ARG_IC_ICTRL_ACTIVE1_EDGE_SET|ARG_IC_ICTRL_ACTIVE2_EDGE_SET;
    } else assert(0);

    // fprintf(argon.fp_reg, "I %d %x out\n", phase, argon.interrupt[0]);
    if (phase == 2) {
      vv("YBASE:%08x CBASE:%08x\n",
          argon.apb[0x8018>>2]*64, argon.apb[0x8020>>2]*64);
    }
}

void rpi_apb_dump_regs(uint16_t addr, int num) {
    int i;
    if (VERBOSE) for (i=0; i<num; i++) {
        if ((i%4)==0)
            vv("%08x: ", 0x7eb00000 + addr + 4*i);
        vv("%08x", argon.apb[(addr>>2)+i]);
        vv((i%4)==3 || i+1 == num ? "\n" : " ");
    }
}

void rpi_axi_dump(uint64_t addr, uint32_t size) {
    int i;
    for (i=0; i<size>>2; i++) {
        if ((i%4)==0)
            vv("%08x: ", MANGLE(argon.axi.vc) + (uint32_t)addr + 4*i);
        vv("%08x", ((uint32_t*)argon.axi.arm)[(addr>>2)+i]);
        vv((i%4)==3 || i+1 == size>>2 ? "\n": " ");
    }
}

void rpi_axi_flush(int mode) {
}

static const char *argon_init() {
    argon.apb = setup_io("/dev/argon-hevcmem", &argon.apb_fd);
    if (!argon.apb)
        return "Failed to open apb";

    argon.interrupt = setup_io("/dev/argon-intcmem", &argon.interrupt_fd);
    if (!argon.interrupt)
        return "Failed to open interrupt";
    
    if (gpu_malloc_uncached(AXI_MEM_SIZE, &argon.axi) != 0)
        return "out of memory";

    v("argon init apb:%p axi.arm:%p axi.vc:%08x\n", argon.apb, argon.axi.arm, MANGLE(argon.axi.vc));

    argon.mbox_fd = mbox_open();
    mbox_request_clock(argon.mbox_fd);

    pthread_mutex_init(&argon.mutex_phase[0], NULL);
    pthread_mutex_init(&argon.mutex_phase[1], NULL);
    pthread_mutex_init(&argon.decoder_lock, NULL);

    return NULL;
}

static void argon_uninit() {
    pthread_mutex_destroy(&argon.mutex_phase[0]);
    pthread_mutex_destroy(&argon.mutex_phase[1]);
    pthread_mutex_destroy(&argon.decoder_lock);

    v("argon release\n");
    for (int idx = 0; idx < MAX_THREADS; idx++) {
        av_assert0(!argon.thread_slots[idx]); // slot still in use?
    }
    mbox_release_clock(argon.mbox_fd);
    mbox_close(argon.mbox_fd);

    release_io((void *)argon.apb);
    close(argon.apb_fd);

    release_io((void *)argon.interrupt);
    close(argon.interrupt_fd);

    gpu_free(&argon.axi);
}

//////////////////////////////////////////////////////////////////////////////

const char *argon_ref() {
    v("argon ref\n");
    const char *ret = NULL;
    pthread_mutex_lock(&argon_lock);
    if (argon.ref_count++ == 0)
        ret = argon_init();
    pthread_mutex_unlock(&argon_lock);
    return ret;
}

void argon_unref() {
    v("argon unref\n");
    pthread_mutex_lock(&argon_lock);
    if (--argon.ref_count == 0)
        argon_uninit();
    pthread_mutex_unlock(&argon_lock);
}
