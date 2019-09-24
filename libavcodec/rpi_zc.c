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

#include "config.h"
#include <pthread.h>

#include "libavcodec/avcodec.h"
#include "libavutil/avassert.h"
#include "libavutil/buffer_internal.h"

#include "rpi_qpu.h"
#include "rpi_mailbox.h"
#include "rpi_zc.h"
#include "rpi_argon.h"

#include <interface/vctypes/vc_image_types.h>

struct ZcPoolEnt;

typedef struct ZcPool {
    int numbytes;
    unsigned int n;
    struct ZcPoolEnt * head;
    pthread_mutex_t lock;
} ZcPool;

typedef struct ZcPoolEnt {
    // It is important that we start with gmem as other bits of code will expect to see that
    GPU_MEM_PTR_T gmem;
    unsigned int n;
    struct ZcPoolEnt * next;
    struct ZcPool * pool;
} ZcPoolEnt;

#define ALLOC_PAD       0
#define ALLOC_ROUND     0x1000
#define STRIDE_ROUND    64
#define STRIDE_OR       0

#define DEBUG_ZAP0_BUFFERS 0

static inline int av_rpi_is_sand_format(const int format) {
    return // (format >= AV_PIX_FMT_SAND128 && format <= AV_PIX_FMT_SAND64_16) ||
            format == AV_PIX_FMT_RPI4_8 || format == AV_PIX_FMT_RPI4_10;
}

static inline int av_rpi_is_sand_frame(const AVFrame * const frame) {
    return av_rpi_is_sand_format(frame->format);
}

static ZcPoolEnt * zc_pool_ent_alloc(ZcPool * const pool, const unsigned int req_size) {
    ZcPoolEnt * const zp = av_malloc(sizeof(ZcPoolEnt));

    // Round up to 4k & add 4k
    const unsigned int alloc_size = (req_size + ALLOC_PAD + ALLOC_ROUND - 1) & ~(ALLOC_ROUND - 1);

    if (zp == NULL) {
        av_log(NULL, AV_LOG_ERROR, "av_malloc(ZcPoolEnt) failed\n");
        goto fail0;
    }

    if (gpu_malloc_cached(alloc_size, &zp->gmem) != 0) {
        av_log(NULL, AV_LOG_ERROR, "av_gpu_malloc_cached(%d) failed\n", alloc_size);
        goto fail1;
    }

    v("%s: Alloc %#x bytes @ %p\n", __func__, zp->gmem.numbytes, zp->gmem.arm);

    pool->numbytes = zp->gmem.numbytes;
    zp->next = NULL;
    zp->pool = pool;
    zp->n = pool->n++;
    return zp;

fail1:
    av_free(zp);
fail0:
    return NULL;
}

static void zc_pool_ent_free(ZcPoolEnt * const zp) {
    v("%s: Free %#x bytes @ %p\n", __func__, zp->gmem.numbytes, zp->gmem.arm);

    gpu_free(&zp->gmem);
    av_free(zp);
}

static void zc_pool_flush(ZcPool * const pool) {
    ZcPoolEnt * p = pool->head;
    pool->head = NULL;
    pool->numbytes = -1;

    while (p != NULL) {
        ZcPoolEnt * const zp = p;
        p = p->next;
        zc_pool_ent_free(zp);
    }
    v("%s: Flush\n", __func__);
}

static ZcPoolEnt * zc_pool_alloc(ZcPool * const pool, const int req_bytes) {
    ZcPoolEnt * zp;
    int numbytes;

    pthread_mutex_lock(&pool->lock);

    numbytes = pool->numbytes;

    // If size isn't close then dump the pool
    // Close in this context means within 128k
    if (req_bytes > numbytes || req_bytes + 0x20000 < numbytes) {
        zc_pool_flush(pool);
        numbytes = req_bytes;
    }

    if (pool->head != NULL) {
        zp = pool->head;
        pool->head = zp->next;
    } else {
        zp = zc_pool_ent_alloc(pool, numbytes);
    }

    pthread_mutex_unlock(&pool->lock);
    return zp;
}

static void zc_pool_free(ZcPoolEnt * const zp) {
    ZcPool * const pool = zp == NULL ? NULL : zp->pool;
    if (zp != NULL) {
        pthread_mutex_lock(&pool->lock);
        v("%s: Recycle %#x, %#x\n", __func__, pool->numbytes, zp->gmem.numbytes);
        if (pool->numbytes == zp->gmem.numbytes) {
            zp->next = pool->head;
            pool->head = zp;
            pthread_mutex_unlock(&pool->lock);
        } else {
            pthread_mutex_unlock(&pool->lock);
            zc_pool_ent_free(zp);
        }
    }
}

static void zc_pool_init(ZcPool * const pool) {
    pool->numbytes = -1;
    pool->head = NULL;
    pthread_mutex_init(&pool->lock, NULL);
}

static void zc_pool_destroy(ZcPool * const pool) {
    pool->numbytes = -1;
    zc_pool_flush(pool);
    pthread_mutex_destroy(&pool->lock);
}

typedef struct AVZcEnv {
    unsigned int refcount;
    ZcPool pool;
    struct {
        int thread_safe_callbacks;
        int (*get_buffer2)(struct AVCodecContext *s, AVFrame *frame, int flags);
        void * get_buffer_context;
    } old;
} ZcEnv;

// Callback when buffer unrefed to zero
static void rpi_free_display_buffer(void *opaque, uint8_t *data) {
    ZcPoolEnt *const zp = opaque;
    zc_pool_free(zp);
}

static inline GPU_MEM_PTR_T * pic_is_gm_ptr(AVBufferRef * const buf) {
    // Kludge where we check the free fn to check this is really
    // one of our buffers - can't think of a better way
    return buf == NULL || buf->buffer->free != rpi_free_display_buffer ? NULL :
        av_buffer_get_opaque(buf);
}

AVRpiZcFrameGeometry av_rpi_zc_frame_geometry(
    const int format, const unsigned int video_width, const unsigned int video_height)
{
    AVRpiZcFrameGeometry geo;

    switch (format) {
        case AV_PIX_FMT_RPI4_8:
        {
            const unsigned int stripe_w = 128;

            static pthread_mutex_t sand_lock = PTHREAD_MUTEX_INITIALIZER;
            static VC_IMAGE_T img = {0};

            // Given the overhead of calling the mailbox keep a stashed
            // copy as we will almost certainly just want the same numbers again
            // but that means we need a lock
            pthread_mutex_lock(&sand_lock);

            if (img.width != video_width || img.height != video_height)
            {
                VC_IMAGE_T new_img = {
                    .type = VC_IMAGE_YUV_UV,
                    .width = video_width,
                    .height = video_height
                };

                gpu_ref();
                mbox_get_image_params(gpu_get_mailbox(), &new_img);
                gpu_unref();
                img = new_img;
            }

            geo.stride_y = stripe_w;
            geo.stride_c = stripe_w;
            geo.height_y = ((intptr_t)img.extra.uv.u - (intptr_t)img.image_data) / stripe_w;
            geo.height_c = img.pitch / stripe_w - geo.height_y;
            geo.stripe_is_yc = 1;
            if (geo.height_y * stripe_w > img.pitch)
            {
                // "tall" sand - all C blocks now follow Y
                geo.height_y = img.pitch / stripe_w;
                geo.height_c = geo.height_y;
                geo.stripe_is_yc = 0;
            }
            geo.planes_c = 1;
            geo.stripes = (video_width + stripe_w - 1) / stripe_w;
            geo.bytes_per_pel = 1;

            pthread_mutex_unlock(&sand_lock);
#if 0
            printf("Req: %dx%d: stride=%d/%d, height=%d/%d, stripes=%d, img.pitch=%d\n",
                   video_width, video_height,
                   geo.stride_y, geo.stride_c,
                   geo.height_y, geo.height_c,
                   geo.stripes, img.pitch);
#endif
            av_assert0((int)geo.height_y > 0 && (int)geo.height_c > 0);
            av_assert0(geo.height_y >= video_height && geo.height_c >= video_height / 2);
            break;
        }

        case AV_PIX_FMT_RPI4_10:
        {
            const unsigned int stripe_w = 128;  // bytes

            static pthread_mutex_t sand_lock = PTHREAD_MUTEX_INITIALIZER;
            static VC_IMAGE_T img = {0};

            // Given the overhead of calling the mailbox keep a stashed
            // copy as we will almost certainly just want the same numbers again
            // but that means we need a lock
            pthread_mutex_lock(&sand_lock);

            if (img.width != video_width || img.height != video_height)
            {
                VC_IMAGE_T new_img = {
                    .type = VC_IMAGE_YUV10COL,
                    .width = video_width,
                    .height = video_height
                };

                gpu_ref();
                mbox_get_image_params(gpu_get_mailbox(), &new_img);
                gpu_unref();
                img = new_img;
            }

            geo.stride_y = stripe_w;
            geo.stride_c = stripe_w;
            geo.height_y = ((intptr_t)img.extra.uv.u - (intptr_t)img.image_data) / stripe_w;
            geo.height_c = img.pitch / stripe_w - geo.height_y;
            geo.planes_c = 1;
            geo.stripes = ((video_width * 4 + 2) / 3 + stripe_w - 1) / stripe_w;
            geo.bytes_per_pel = 1;
            geo.stripe_is_yc = 1;

            pthread_mutex_unlock(&sand_lock);

            av_assert0((int)geo.height_y > 0 && (int)geo.height_c > 0);
            av_assert0(geo.height_y >= video_height && geo.height_c >= video_height / 2);
            break;
        }
        default:
            av_log(NULL, AV_LOG_ERROR, "av_rpi_zc_frame_geometry() for non hw frame\n");
            memset(&geo, 0, sizeof(geo));
            break;
    }
    return geo;
}


static AVBufferRef * rpi_buf_pool_alloc(ZcPool * const pool, int size) {
    ZcPoolEnt *const zp = zc_pool_alloc(pool, size);
    AVBufferRef * buf;
    intptr_t idata = (intptr_t)zp->gmem.arm;

    if (zp == NULL) {
        av_log(NULL, AV_LOG_ERROR, "zc_pool_alloc(%d) failed\n", size);
        goto fail0;
    }

#if DEBUG_ZAP0_BUFFERS
    memset((void*)idata, 0, size);
#endif

    if ((buf = av_buffer_create((void *)idata, size, rpi_free_display_buffer, zp, AV_BUFFER_FLAG_READONLY)) == NULL) {
        av_log(NULL, AV_LOG_ERROR, "av_buffer_create() failed\n");
        goto fail2;
    }

    return buf;

fail2:
    zc_pool_free(zp);
fail0:
    return NULL;
}

static int rpi_get_display_buffer(ZcEnv *const zc, AVFrame * const frame) {
    const AVRpiZcFrameGeometry geo = av_rpi_zc_frame_geometry(frame->format, frame->width, frame->height);
    const unsigned int size_y = geo.stride_y * geo.height_y;
    const unsigned int size_c = geo.stride_c * geo.height_c;
    const unsigned int size_pic = (size_y + size_c * geo.planes_c) * geo.stripes;
    AVBufferRef * buf;
    unsigned int i;

    // printf("Do local alloc: format=%#x, %dx%d: %u\n", frame->format, frame->width, frame->height, size_pic);

    if ((buf = rpi_buf_pool_alloc(&zc->pool, size_pic)) == NULL) {
        av_log(NULL, AV_LOG_ERROR, "rpi_get_display_buffer: Failed to get buffer from pool\n");
        return AVERROR(ENOMEM);
    }

    for (i = 0; i < AV_NUM_DATA_POINTERS; i++) {
        frame->buf[i] = NULL;
        frame->data[i] = NULL;
        frame->linesize[i] = 0;
    }

    frame->buf[0] = buf;

    frame->linesize[0] = geo.stride_y;
    frame->linesize[1] = geo.stride_c;
    frame->linesize[2] = geo.stride_c;
    // abuse: linesize[3] = "stripe stride"
    // stripe_stride is NOT the stride between slices it is (that / geo.stride_y).
    // In a general case this makes the calculation an xor and multiply rather
    // than a divide and multiply
    if (geo.stripes > 1)
        frame->linesize[3] = geo.stripe_is_yc ? geo.height_y + geo.height_c : geo.height_y;

    frame->data[0] = buf->data;
    frame->data[1] = frame->data[0] + (geo.stripe_is_yc ? size_y : size_y * geo.stripes);
    if (geo.planes_c > 1)
        frame->data[2] = frame->data[1] + size_c;

    frame->extended_data = frame->data;
    // Leave extended buf alone

    return 0;
}

static int av_rpi_zc_get_buffer2(struct AVCodecContext *s, AVFrame *frame, int flags) {
    av_assert0(av_rpi_is_sand_frame(frame));
    return rpi_get_display_buffer(s->get_buffer_context, frame);
}

static AVZcEnvPtr av_rpi_zc_env_alloc(void) {
    ZcEnv * const zc = av_mallocz(sizeof(ZcEnv));
    if (zc == NULL) {
        av_log(NULL, AV_LOG_ERROR, "av_rpi_zc_env_alloc: Context allocation failed\n");
        return NULL;
    }
    zc_pool_init(&zc->pool);
    return zc;
}

static void av_rpi_zc_env_free(AVZcEnvPtr zc) {
    av_assert0(zc);
    zc_pool_destroy(&zc->pool); ;
    av_free(zc);
}

static int av_rpi_zc_in_use(const struct AVCodecContext * const s) {
    return s->get_buffer2 == av_rpi_zc_get_buffer2;
}

AVRpiZcRefPtr av_rpi_zc_ref(struct AVCodecContext * const s, const AVFrame * const frame) {
    assert(s != NULL);
    av_assert0(pic_is_gm_ptr(frame->buf[0]));
    return av_buffer_ref(frame->buf[0]);
}

int av_rpi_zc_vc_handle(const AVRpiZcRefPtr fr_ref) {
    av_assert0(fr_ref);
    const GPU_MEM_PTR_T * const p = pic_is_gm_ptr(fr_ref);
    av_assert0(p);
    return p->vc_handle;
}

int av_rpi_zc_offset(const AVRpiZcRefPtr fr_ref) {
    av_assert0(fr_ref);
    const GPU_MEM_PTR_T * const p = pic_is_gm_ptr(fr_ref);
    av_assert0(p);
    return fr_ref->data - p->arm;
}

int av_rpi_zc_length(const AVRpiZcRefPtr fr_ref) {
    av_assert0(fr_ref);
    return fr_ref->size;
}

int av_rpi_zc_numbytes(const AVRpiZcRefPtr fr_ref) {
    av_assert0(fr_ref);
    const GPU_MEM_PTR_T * const p = pic_is_gm_ptr(fr_ref);
    av_assert0(p);
    return p->numbytes;
}

void av_rpi_zc_unref(AVRpiZcRefPtr fr_ref) {
    av_assert0(fr_ref);
    av_buffer_unref(&fr_ref);
}

int av_rpi_zc_init(struct AVCodecContext * const s) {
    if (av_rpi_zc_in_use(s)) {
        ZcEnv * const zc = s->get_buffer_context;
        ++zc->refcount;
    } else {
        ZcEnv *const zc = av_rpi_zc_env_alloc();
        if (zc == NULL) {
            return AVERROR(ENOMEM);
        }

        zc->refcount = 1;
        zc->old.get_buffer_context = s->get_buffer_context;
        zc->old.get_buffer2 = s->get_buffer2;
        zc->old.thread_safe_callbacks = s->thread_safe_callbacks;

        s->get_buffer_context = zc;
        s->get_buffer2 = av_rpi_zc_get_buffer2;
        s->thread_safe_callbacks = 1;
    }
    return 0;
}

void av_rpi_zc_uninit(struct AVCodecContext * const s) {
    if (av_rpi_zc_in_use(s)) {
        ZcEnv * const zc = s->get_buffer_context;
        if (--zc->refcount == 0) {
            s->get_buffer2 = zc->old.get_buffer2;
            s->get_buffer_context = zc->old.get_buffer_context;
            s->thread_safe_callbacks = zc->old.thread_safe_callbacks;
            av_rpi_zc_env_free(zc);
        }
    }
}
