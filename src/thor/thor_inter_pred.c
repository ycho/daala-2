/*
Copyright (c) 2015, Cisco Systems
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* -*- mode: c; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2; -*- */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <memory.h>
#include <assert.h>

#include "thor_global.h"
#include "thor_common_block.h"
#include "thor_simd.h"
#include "thor_common_kernels.h"

#if HEVC_INTERPOLATION
#define NTAPY 8
#define OFFY (NTAPY/2)
#define OFFYM1 (OFFY-1)
static const int16_t filter_coeffsY[4][8] = {
    { 0,  0,  0, 64,  0,  0, 0,  0},
    {-1,  4,-10, 58, 17, -5, 1,  0},
    {-1,  4,-11, 40, 40,-11, 4, -1},
    { 0,  1, -5, 17, 58,-10, 4, -1}
};
#else
#define NTAPY 6
#define OFFY (NTAPY/2)
#define OFFYM1 (OFFY-1)
static const int16_t filter_coeffsY[4][8] = {
    { 0,  0,128,  0,  0,  0},
    { 3,-15,111, 37,-10,  2},
    { 3,-17, 78, 78,-17,  3},
    { 2,-10, 37,111,-15,  3}
};
#endif

static const int8_t filter_coeffsC[8][4] = {
    { 0, 64,  0,  0},
    {-2, 58,  10,-2},
    {-4, 54, 16, -2},
    {-4, 44, 28, -4},
    {-4, 36, 36, -4},
    {-4, 28, 44, -4},
    {-2, 16, 54, -4},
    {-2, 10, 58, -2}
};

void thor_get_inter_prediction_chroma(uint8_t *pblock, uint8_t *ref, int width, int height, int stride, int pstride,
 int32_t mvx, int32_t mvy)
{
  int i,j;

  int m,i_off,j_off;
  mv_t mvtemp;
/* mvtemp.x = sign ? -mv->x : mv->x;
  mvtemp.y = sign ? -mv->y : mv->y; */
  mvtemp.x = (int16_t)mvx;
  mvtemp.y = (int16_t)mvy;
  int ver_frac = (mvtemp.y)&7;
  int hor_frac = (mvtemp.x)&7;
  int ver_int = (mvtemp.y)>>3;
  int hor_int = (mvtemp.x)>>3;
  int16_t tmp[80][80];

  if (ver_frac==0 && hor_frac==0){
    j_off = 0 + hor_int;
    for(i=0;i<height;i++){
      i_off = i + ver_int;
      memcpy(pblock + i*pstride,ref + i_off*stride + j_off, width*sizeof(uint8_t));
    }
    return;
  }

  if (use_simd && width > 2)
    get_inter_prediction_chroma_simd(width, height, hor_frac, ver_frac, pblock, pstride, ref + ver_int*stride + hor_int, stride);
  else {
    /* Horizontal filtering */
    for(i=-1;i<height+2;i++){
      for (j=0;j<width;j++){
        int sum = 0;
        i_off = i + ver_int;
        j_off = j + hor_int;
        for (m=0;m<4;m++) sum += filter_coeffsC[hor_frac][m] * ref[i_off * stride + j_off + m - 1];
        tmp[i+1][j] = sum;
      }
    }

    /* Vertical filtering */
    for(i=0;i<height;i++){
      for (j=0;j<width;j++){
        int sum = 0;
        for (m=0;m<4;m++) sum += filter_coeffsC[ver_frac][m] * tmp[i+m][j];
        pblock[i*pstride+j] = clip255((sum + 2048)>>12);
      }
    }
  }
}

void thor_get_inter_prediction_luma(uint8_t *pblock, uint8_t *ref, int width, int height, int stride, int pstride,
 int32_t mvx, int32_t mvy)
{
  int i,j;

  int m,i_off,j_off;
  mv_t mvtemp;
/* mvtemp.x = sign ? -mv->x : mv->x;
  mvtemp.y = sign ? -mv->y : mv->y; */
  mvtemp.x = (int16_t)mvx;
  mvtemp.y = (int16_t)mvy;
  int ver_frac = (mvtemp.y)&3;
  int hor_frac = (mvtemp.x)&3;
  int ver_int = (mvtemp.y)>>2;
  int hor_int = (mvtemp.x)>>2;
  int32_t tmp[80][80]; //7-bit filter exceeds 16 bit temporary storage

  /* Integer position */
  if (ver_frac==0 && hor_frac==0){
    j_off = 0 + hor_int;
    for(i=0;i<height;i++){
      i_off = i + ver_int;
      memcpy(pblock + i*pstride,ref + i_off*stride+j_off, width*sizeof(uint8_t));
    }
    return;
  }

#if HEVC_INTERPOLATION
  /* Vertical filtering */
  for(i=-OFFYM1;i<width+OFFY;i++){
    for (j=0;j<height;j++){
      int sum = 0;
      i_off = i + hor_int;
      j_off = j + ver_int;
      for (m=0;m<NTAPY;m++) sum += filter_coeffsY[ver_frac][m] * ref[(j_off + m - OFFYM1) * stride + i_off];
      tmp[j][i+OFFYM1] = sum;
    }
  }
  /* Horizontal filtering */
  for(i=0;i<width;i++){
    for (j=0;j<height;j++){
      int sum = 0;
      for (m=0;m<NTAPY;m++) sum += filter_coeffsY[hor_frac][m] * tmp[j][i+m];
      pblock[j*pstride+i] = clip255((sum + 2048)>>12);
    }
  }
  return;
#endif

  if (use_simd) {
    get_inter_prediction_luma_simd(width, height, hor_frac, ver_frac, pblock, pstride, ref + ver_int*stride + hor_int, stride);
  }
  else {
    /* Special lowpass filter at center position */
    if (ver_frac == 2 && hor_frac == 2) {
      for(i=0;i<height;i++){
        for (j=0;j<width;j++){
          int sum = 0;
          i_off = i + ver_int;
          j_off = j + hor_int;

          sum += 0*ref[(i_off-1)*stride+j_off-1]+1*ref[(i_off-1)*stride+j_off+0]+1*ref[(i_off-1)*stride+j_off+1]+0*ref[(i_off-1)*stride+j_off+2];
          sum += 1*ref[(i_off+0)*stride+j_off-1]+2*ref[(i_off+0)*stride+j_off+0]+2*ref[(i_off+0)*stride+j_off+1]+1*ref[(i_off+0)*stride+j_off+2];
          sum += 1*ref[(i_off+1)*stride+j_off-1]+2*ref[(i_off+1)*stride+j_off+0]+2*ref[(i_off+1)*stride+j_off+1]+1*ref[(i_off+1)*stride+j_off+2];
          sum += 0*ref[(i_off+2)*stride+j_off-1]+1*ref[(i_off+2)*stride+j_off+0]+1*ref[(i_off+2)*stride+j_off+1]+0*ref[(i_off+2)*stride+j_off+2];
          pblock[i*pstride+j] = clip255((sum + 8)>>4);
        }
      }
    } else {

      /* Vertical filtering */
      for(i=-OFFYM1;i<width+OFFY;i++){
        for (j=0;j<height;j++){
          int sum = 0;
          i_off = i + hor_int;
          j_off = j + ver_int;
          for (m=0;m<NTAPY;m++) sum += filter_coeffsY[ver_frac][m] * ref[(j_off + m - OFFYM1) * stride + i_off]; //7-bit version
          tmp[j][i+OFFYM1] = sum;
        }
      }
      /* Horizontal filtering */
      for(i=0;i<width;i++){
        for (j=0;j<height;j++){
          int sum = 0;
          for (m=0;m<NTAPY;m++) sum += filter_coeffsY[hor_frac][m] * tmp[j][i+m]; //7-bit version
          pblock[j*pstride+i] = clip255((sum + 8192)>>14); //7-bit version
        }
      }
    }
  }
}
