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

#ifndef LIBAVCODEC_RPI_ZC_H
#define LIBAVCODEC_RPI_ZC_H

// Zero-Copy frame code for RPi
// RPi needs Y/U/V planes to be contiguous for display.  By default
// ffmpeg will allocate separated planes so a memcpy is needed before
// display.  This code provides a method a making ffmpeg allocate a single
// bit of memory for the frame when can then be reference counted until
// display has finished with it.

struct AVBufferRef;
struct AVFrame;
struct AVCodecContext;
enum AVPixelFormat;

// "Opaque" pointer to whatever we are using as a buffer reference
typedef struct AVBufferRef * AVRpiZcRefPtr;

struct AVZcEnv;
typedef struct AVZcEnv * AVZcEnvPtr;

typedef struct AVRpiZcFrameGeometry {
    unsigned int stride_y;  // Luma stride (bytes)
    unsigned int height_y;  // Luma height (lines)
    unsigned int stride_c;  // Chroma stride (bytes)
    unsigned int height_c;  // Chroma stride (lines)
    unsigned int planes_c;  // Chroma plane count (U, V = 2, interleaved = 1)
    unsigned int stripes;   // Number of stripes (sand)
    unsigned int bytes_per_pel;
    int stripe_is_yc;       // A single stripe is Y then C (false for tall sand)
} AVRpiZcFrameGeometry;

AVRpiZcFrameGeometry av_rpi_zc_frame_geometry(
    const int format,
    const unsigned int video_width, const unsigned int video_height
);

// Generate a ZC reference to the buffer(s) in this frame
// If the buffer doesn't appear to be one allocated by _get_buffer_2
// then the behaviour depends on maycopy:
//   If maycopy=0 then return NULL
//   If maycopy=1 && the src frame is in a form where we can easily copy
//     the data, then allocate a new buffer and copy the data into it
//   Otherwise return NULL
AVRpiZcRefPtr av_rpi_zc_ref(struct AVCodecContext * const s, const struct AVFrame * const frame);

// Get the vc_handle from the frame ref
// Returns -1 if ref doesn't look valid
int av_rpi_zc_vc_handle(const AVRpiZcRefPtr fr_ref);

// Get offset from the start of the memory referenced
// by the vc_handle to valid data
int av_rpi_zc_offset(const AVRpiZcRefPtr fr_ref);

// Length of buffer data
int av_rpi_zc_length(const AVRpiZcRefPtr fr_ref);

// Get the number of bytes allocated from the frame ref
// Returns 0 if ref doesn't look valid
int av_rpi_zc_numbytes(const AVRpiZcRefPtr fr_ref);

// Unreference the buffer refed/allocated by _zc_ref
// If fr_ref is NULL then this will NOP
void av_rpi_zc_unref(AVRpiZcRefPtr fr_ref);

// Init ZC into a context
// There is nothing magic in this fn - it just packages setting
// get_buffer2 & get_buffer_context
int av_rpi_zc_init(struct AVCodecContext * const s);

// Free ZC from a context
// There is nothing magic in this fn - it just packages unsetting
// get_buffer2 & get_buffer_context
void av_rpi_zc_uninit(struct AVCodecContext * const s);

#endif

