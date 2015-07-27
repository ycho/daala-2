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

#include "thor_simd.h"
#include "thor_global.h"

static void get_inter_prediction_luma_edge(int width, int height, int xoff, int yoff,
                                             uint8_t *restrict qp, int qstride,
                                             const uint8_t *restrict ip, int istride)
{
  static const ALIGN(16) int16_t coeffs[4][6][4] = {
    { {   3,   3,   3,   3 },
      { -15, -15, -15, -15 },
      { -17, -17, -17, -17 },  // 111 - 128
      {  37,  37,  37,  37 },
      { -10, -10, -10, -10 },
      {   2,   2,   2,   2 }
    },
    { {   3,   3,   3,   3 },
      { -17, -17, -17, -17 },
      { -50, -50, -50, -50 },  // 78 - 128
      {  78,  78,  78,  78 },
      { -17, -17, -17, -17 },
      {   3,   3,   3,   3 }
    },
    { {   2,   2,   2,   2 },
      { -10, -10, -10, -10 },
      { -91, -91, -91, -91 },  // 37 - 128
      { 111, 111, 111, 111 },
      { -15, -15, -15, -15 },
      {   3,   3,   3,   3 } }
  };

  const unsigned char *restrict ip2 = ip;
  int cf = xoff + yoff - 1;
  int sx = !yoff;
  int s1 = !xoff * istride;
  ip += width - 2 * istride / 2;
  ip2 += height - istride;
  qp -= qstride;

  v64 c0 = v64_load_aligned(coeffs[cf][0]);
  v64 c1 = v64_load_aligned(coeffs[cf][1]);
  v64 c2 = v64_load_aligned(coeffs[cf][2]);
  v64 c3 = v64_load_aligned(coeffs[cf][3]);
  v64 c4 = v64_load_aligned(coeffs[cf][4]);
  v64 c5 = v64_load_aligned(coeffs[cf][5]);
  v64 cr = v64_dup_16(64);
  int st1 = s1 + sx;

  for (int y = 0; y < height; y++) {

    qp += qstride;
    ip += istride - width;
    ip2 += istride - width;

    if (width == 4) {
      v64 l0, l1, l2, l3, l4, l5;
      v64 r0, r1, r2, r3, r4, r5;
      v64 fx1b, rs;
      const unsigned char *r = ip - 2 * s1 - 2 * sx;
      l0 = v64_load_unaligned(r);
      r += st1;
      l1 = v64_load_unaligned(r);
      r += st1;
      l2 = v64_load_unaligned(ip);
      r += st1;
      l3 = v64_load_unaligned(r);
      r += st1;
      l4 = v64_load_unaligned(r);
      r += st1;
      l5 = v64_load_unaligned(r);
      fx1b = v64_unpacklo_u8_s16(l2);
      r0 = v64_mullo_s16(c0, v64_unpacklo_u8_s16(l0));
      r1 = v64_mullo_s16(c1, v64_unpacklo_u8_s16(l1));
      r2 = v64_mullo_s16(c2, fx1b);
      r3 = v64_mullo_s16(c3, v64_unpacklo_u8_s16(l3));
      r4 = v64_mullo_s16(c4, v64_unpacklo_u8_s16(l4));
      r5 = v64_mullo_s16(c5, v64_unpacklo_u8_s16(l5));
      rs = v64_add_16(v64_add_16(v64_add_16(v64_add_16(v64_add_16(v64_add_16(cr, r0), r1), r2), r3), r4), r5);
      ip += 4;
      rs = v64_add_16(v64_shr_n_s16(rs, 7), fx1b);
      u32_store_aligned(qp, v64_low_u32(v64_pack_s16_u8(rs, rs)));
    } else {
      for (int x = 0; x < width; x += 8) {
        v64 l0, l1, l2, l3, l4, l5;
        v64 r0, r1, r2, r3, r4, r5;
        v64 fx1a, fx1b, rs1, rs2;
        const unsigned char *r = ip - 2 * s1 - 2 * sx;
        l0 = v64_load_unaligned(r);
        r += st1;
        l1 = v64_load_unaligned(r);
        r += st1;
        l2 = v64_load_unaligned(ip);
        r += st1;
        l3 = v64_load_unaligned(r);
        r += st1;
        l4 = v64_load_unaligned(r);
        r += st1;
        l5 = v64_load_unaligned(r);
        fx1a = v64_unpackhi_u8_s16(l2);
        r0 = v64_mullo_s16(c0, v64_unpackhi_u8_s16(l0));
        r1 = v64_mullo_s16(c1, v64_unpackhi_u8_s16(l1));
        r2 = v64_mullo_s16(c2, fx1a);
        r3 = v64_mullo_s16(c3, v64_unpackhi_u8_s16(l3));
        r4 = v64_mullo_s16(c4, v64_unpackhi_u8_s16(l4));
        r5 = v64_mullo_s16(c5, v64_unpackhi_u8_s16(l5));
        rs1 = v64_add_16(v64_add_16(v64_add_16(v64_add_16(v64_add_16(v64_add_16(cr, r0), r1), r2), r3), r4), r5);
        fx1b = v64_unpacklo_u8_s16(l2);
        r0 = v64_mullo_s16(c0, v64_unpacklo_u8_s16(l0));
        r1 = v64_mullo_s16(c1, v64_unpacklo_u8_s16(l1));
        r2 = v64_mullo_s16(c2, fx1b);
        r3 = v64_mullo_s16(c3, v64_unpacklo_u8_s16(l3));
        r4 = v64_mullo_s16(c4, v64_unpacklo_u8_s16(l4));
        r5 = v64_mullo_s16(c5, v64_unpacklo_u8_s16(l5));
        rs2 = v64_add_16(v64_add_16(v64_add_16(v64_add_16(v64_add_16(v64_add_16(cr, r0), r1), r2), r3), r4), r5);

        ip += 8;
        rs1 = v64_add_16(v64_shr_n_s16(rs1, 7), fx1a);
        rs2 = v64_add_16(v64_shr_n_s16(rs2, 7), fx1b);
        v64_store_aligned(qp + x, v64_pack_s16_u8(rs1, rs2));
      }
    }
  }
}

static void get_inter_prediction_luma_inner(int width, int height, int xoff, int yoff,
                                            uint8_t *restrict qp, int qstride,
                                            const uint8_t *restrict ip, int istride)
{
#define F0 { 0,   0,   1,   0,   0, 0,   0, 0 }
#define F1 { 3, -15, 111,  37, -10, 2,   0, 0 }
#define F2 { 2, -10,  37, 111, -15, 3,   0, 0 }
#define F3 { 3, -17,  78,  78, -17, 3,   0, 0 }

  static ALIGN(16) int16_t coeffs2[24][2][8] = {
    { F0, F0 }, { F0, F0 }, { F0, F0 }, { F0, F0 },
    { F0, F0 }, { F1, F1 }, { F3, F1 }, { F2, F1 },
    { F0, F0 }, { F1, F3 }, { F3, F3 }, { F2, F3 },
  };

  if (width == 4) {
    static ALIGN(16) int16_t coeffs[3][6][8] = {
      { { 0 } },
      {
        {   3,   3,   3,   3,   3,   3,   3,   3 },
        { -15, -15, -15, -15, -15, -15, -15, -15 },
        { -17, -17, -17, -17, -17, -17, -17, -17 },  // 111 - 128
        {  37,  37,  37,  37,  37,  37,  37,  37 },
        { -10, -10, -10, -10, -10, -10, -10, -10 },
        {   2,   2,   2,   2,   2,   2,   2,   2 }
      },
      { {   3,   3,   3,   3,   3,   3,   3,   3 },
        { -17, -17, -17, -17, -17, -17, -17, -17 },
        { -50, -50, -50, -50, -50, -50, -50, -50 },  // 78 - 128
        {  78,  78,  78,  78,  78,  78,  78,  78 },
        { -17, -17, -17, -17, -17, -17, -17, -17 },
        {   3,   3,   3,   3,   3,   3,   3,   3 }
      }
    };
    v128 c = v128_load_aligned(coeffs2[xoff + yoff*4][0]);

    v128 c0 = v128_load_aligned(coeffs[yoff][0]);
    v128 c1 = v128_load_aligned(coeffs[yoff][1]);
    v128 c2 = v128_load_aligned(coeffs[yoff][2]);
    v128 c3 = v128_load_aligned(coeffs[yoff][3]);
    v128 c4 = v128_load_aligned(coeffs[yoff][4]);
    v128 c5 = v128_load_aligned(coeffs[yoff][5]);

    for (int y = 0; y < height; y++) {
      int res;
      v128 ax = v128_unpacklo_u8_s16(v64_load_unaligned(ip - 2));
      v128 a0 = v128_mullo_s16(c0, v128_unpacklo_u8_s16(v64_load_unaligned(ip - 2 * istride - 2)));
      v128 a1 = v128_mullo_s16(c1, v128_unpacklo_u8_s16(v64_load_unaligned(ip - 1 * istride - 2)));
      v128 a2 = v128_mullo_s16(c2, ax);
      v128 a3 = v128_mullo_s16(c3, v128_unpacklo_u8_s16(v64_load_unaligned(ip + 1 * istride - 2)));
      v128 a4 = v128_mullo_s16(c4, v128_unpacklo_u8_s16(v64_load_unaligned(ip + 2 * istride - 2)));
      v128 a5 = v128_mullo_s16(c5, v128_unpacklo_u8_s16(v64_load_unaligned(ip + 3 * istride - 2)));

      for (int x = 0; x < 3; x++) {
        res = (v128_dotp_s16(c, v128_add_16(v128_add_16(v128_add_16(v128_add_16(v128_add_16(a0, a1), a2), a3), a4), a5)) + v128_dotp_s16(c, ax) * 128 + 8192) >> 14;
        *qp++ = clip255(res);
        ax = v128_shr_n_byte(ax, 2);
        a0 = v128_shr_n_byte(a0, 2);
        a1 = v128_shr_n_byte(a1, 2);
        a2 = v128_shr_n_byte(a2, 2);
        a3 = v128_shr_n_byte(a3, 2);
        a4 = v128_shr_n_byte(a4, 2);
        a5 = v128_shr_n_byte(a5, 2);
      }

      int a08, a18, a28, a38, a48, a58;
      switch ((yoff == 1)*2+(xoff == 1)) {
      case 0:
        a08 = ip[6-2*istride]*3*3;
        a18 = ip[6-1*istride]*-17*3;
        a28 = ip[6-0*istride]*78*3;
        a38 = ip[6+1*istride]*78*3;
        a48 = ip[6+2*istride]*-17*3;
        a58 = ip[6+3*istride]*3*3;
        break;
      case 1:
        a08 = ip[6-2*istride]*3*2;
        a18 = ip[6-1*istride]*-17*2;
        a28 = ip[6-0*istride]*78*2;
        a38 = ip[6+1*istride]*78*2;
        a48 = ip[6+2*istride]*-17*2;
        a58 = ip[6+3*istride]*3*2;
        break;
      case 2:
        a08 = ip[6-2*istride]*3*3;
        a18 = ip[6-1*istride]*-15*3;
        a28 = ip[6-0*istride]*111*3;
        a38 = ip[6+1*istride]*37*3;
        a48 = ip[6+2*istride]*-10*3;
        a58 = ip[6+3*istride]*2*3;
        break;
      default:
        a08 = ip[6-2*istride]*3*2;
        a18 = ip[6-1*istride]*-15*2;
        a28 = ip[6-0*istride]*111*2;
        a38 = ip[6+1*istride]*37*2;
        a48 = ip[6+2*istride]*-10*2;
        a58 = ip[6+3*istride]*2*2;
        break;
      }

      res = v128_dotp_s16(c, v128_add_16(v128_add_16(v128_add_16(v128_add_16(v128_add_16(a0, a1), a2), a3), a4), a5)) +
        v128_dotp_s16(c, ax) * 128 + a08 + a18 + a28 + a38 + a48 + a58;
      *qp++ = clip255((res + 8192) >> 14);
      ip += istride;
      qp += qstride - 4;
    }

  } else {
    v128 c = v128_load_aligned(coeffs2[xoff + yoff*4][0]);
    const uint8_t *restrict ip2 = ip;
    v128 c1, c2, c3;
    int16_t *ax = thor_alloc((width+8)*height*2, 16);

    if (yoff == 1) {
      c1 = v128_dup_16((  3 << 8) | (uint8_t)-15);
      c2 = v128_dup_16((-17 << 8) | (uint8_t) 37);
      c3 = v128_dup_16((-10 << 8) | (uint8_t)  2);
    } else {
      c1 = v128_dup_16((  3 << 8) | (uint8_t)-17);
      c2 = v128_dup_16((-50 << 8) | (uint8_t) 78);
      c3 = v128_dup_16((-17 << 8) | (uint8_t)  3);
    }

    for (int y = 0; y < height; y++) {
      int16_t *a = ax + y*(width+8);
      for (int i = 0; i <= width; i += 8) {
        v128 t1 = v128_madd_us8(v128_zip_8(v64_load_unaligned(ip - 2 * istride - 2),
                                           v64_load_unaligned(ip - 1 * istride - 2)), c1);
        v128 t2 = v128_madd_us8(v128_zip_8(v64_load_unaligned(ip - 0 * istride - 2),
                                           v64_load_unaligned(ip + 1 * istride - 2)), c2);
        v128 t3 = v128_madd_us8(v128_zip_8(v64_load_unaligned(ip + 2 * istride - 2),
                                           v64_load_unaligned(ip + 3 * istride - 2)), c3);
        v128_store_aligned(a + i, v128_add_16(v128_add_16(t1, t2), t3));
        ip += 8;
      }
      ip += istride - width - 8;
    }
    ip = ip2 - 2;

    for (int y = 0; y < height; y++) {
      int16_t *a = ax + y*(width+8);
      for (int i = 0; i < width; i += 8) {
        v128 r0 = v128_from_64((v128_dotp_s16(c, v128_load_unaligned(a + i + 7)) + v128_dotp_s16(c, v128_unpack_u8_s16(v64_load_unaligned(ip + y*istride + i + 7))) * 128) << 32 |
                               (uint32_t)(v128_dotp_s16(c, v128_load_unaligned(a + i + 6)) + v128_dotp_s16(c, v128_unpack_u8_s16(v64_load_unaligned(ip + y*istride + i + 6))) * 128),
                               (v128_dotp_s16(c, v128_load_unaligned(a + i + 5)) + v128_dotp_s16(c, v128_unpack_u8_s16(v64_load_unaligned(ip + y*istride + i + 5))) * 128) << 32 |
                               (uint32_t)(v128_dotp_s16(c, v128_load_unaligned(a + i + 4)) + v128_dotp_s16(c, v128_unpack_u8_s16(v64_load_unaligned(ip + y*istride + i + 4))) * 128));
        v128 r1 = v128_from_64((v128_dotp_s16(c, v128_load_unaligned(a + i + 3)) + v128_dotp_s16(c, v128_unpack_u8_s16(v64_load_unaligned(ip + y*istride + i + 3))) * 128) << 32 |
                               (uint32_t)(v128_dotp_s16(c, v128_load_unaligned(a + i + 2)) + v128_dotp_s16(c, v128_unpack_u8_s16(v64_load_unaligned(ip + y*istride + i + 2))) * 128),
                               (v128_dotp_s16(c, v128_load_unaligned(a + i + 1)) + v128_dotp_s16(c, v128_unpack_u8_s16(v64_load_unaligned(ip + y*istride + i + 1))) * 128) << 32 |
                               (uint32_t)(v128_dotp_s16(c, v128_load_unaligned(a + i + 0)) + v128_dotp_s16(c, v128_unpack_u8_s16(v64_load_unaligned(ip + y*istride + i + 0))) * 128));
        r0 = v128_shr_n_s32(v128_add_32(r0, v128_dup_32(8192)), 14);
        r1 = v128_shr_n_s32(v128_add_32(r1, v128_dup_32(8192)), 14);
        r0 = v128_pack_s32_s16(r0, r1);
        v64_store_aligned(qp + y*qstride + i, v128_low_v64(v128_pack_s16_u8(r0, r0)));
      }
    }
    thor_free(ax);
  }
}

static void get_inter_prediction_luma_centre(int width, int height,
                                             uint8_t *restrict qp, int qstride,
                                             const uint8_t *restrict ip, int istride)
{
  if (width == 4) {
    v128 round = v128_dup_16(8);
    for (int i = 0; i < height; i++) {
      v64 r, s;
      r = v64_add_16(v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip - 1 * istride + 0))),
                     v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip - 1 * istride + 1))));
      r = v64_add_16(r, v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip - 0 * istride - 1))));
      r = v64_add_16(r, v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip + 1 * istride - 1))));
      r = v64_add_16(r, v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip + 1 * istride + 2))));
      r = v64_add_16(r, v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip + 2 * istride + 0))));
      r = v64_add_16(r, v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip + 2 * istride + 1))));
      r = v64_add_16(r, v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip - 0 * istride + 2))));
      s = v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip - 0 * istride + 0)));
      r = v64_add_16(r, v64_add_16(s, s));
      s = v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip - 0 * istride + 1)));
      r = v64_add_16(r, v64_add_16(s, s));
      s = v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip + 1 * istride + 0)));
      r = v64_add_16(r, v64_add_16(s, s));
      s = v64_unpacklo_u8_s16(v64_from_32(0, u32_load_unaligned(ip + 1 * istride + 1)));
      r = v64_add_16(r, v64_add_16(s, s));
      r = v64_shr_s16(v64_add_16(r, round), 4);
      u32_store_aligned(qp + i * qstride, v64_low_u32(v64_pack_s16_u8(r, r)));
      ip += istride;
    }
  } else {
    v128 round = v128_dup_16(8);
    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j += 8) {
        v128 r, s;
        r = v128_add_16(v128_unpack_u8_s16(v64_load_unaligned(ip - 1 * istride + 0)),
                        v128_unpack_u8_s16(v64_load_unaligned(ip - 1 * istride + 1)));
        r = v128_add_16(r, v128_unpack_u8_s16(v64_load_unaligned(ip - 0 * istride - 1)));
        r = v128_add_16(r, v128_unpack_u8_s16(v64_load_unaligned(ip + 1 * istride - 1)));
        r = v128_add_16(r, v128_unpack_u8_s16(v64_load_unaligned(ip + 1 * istride + 2)));
        r = v128_add_16(r, v128_unpack_u8_s16(v64_load_unaligned(ip + 2 * istride + 0)));
        r = v128_add_16(r, v128_unpack_u8_s16(v64_load_unaligned(ip + 2 * istride + 1)));
        r = v128_add_16(r, v128_unpack_u8_s16(v64_load_unaligned(ip - 0 * istride + 2)));
        s = v128_unpack_u8_s16(v64_load_unaligned(ip - 0 * istride + 0));
        r = v128_add_16(r, v128_add_16(s, s));
        s = v128_unpack_u8_s16(v64_load_unaligned(ip - 0 * istride + 1));
        r = v128_add_16(r, v128_add_16(s, s));
        s = v128_unpack_u8_s16(v64_load_unaligned(ip + 1 * istride + 0));
        r = v128_add_16(r, v128_add_16(s, s));
        s = v128_unpack_u8_s16(v64_load_unaligned(ip + 1 * istride + 1));
        r = v128_add_16(r, v128_add_16(s, s));
        r = v128_shr_s16(v128_add_16(r, round), 4);
        v64_store_aligned(qp + i * qstride + j, v128_low_v64(v128_pack_s16_u8(r, r)));
        ip += 8;
      }
      ip += istride - width;
    }
  }
}

void get_inter_prediction_luma_simd(int width, int height, int xoff, int yoff,
                                    uint8_t *restrict qp, int qstride,
                                    const uint8_t *restrict ip, int istride)
{
  if (xoff == 2 && yoff == 2)
    get_inter_prediction_luma_centre(width, height, qp, qstride, ip, istride);
  else {
    /* Use symmetric property of the filter */
    if (yoff == 3) {
      ip += height*istride;
      qp += (height-1)*qstride;
      istride = -istride;
      qstride = -qstride;
      yoff = 1;
    }
    (!xoff || !yoff ? get_inter_prediction_luma_edge : get_inter_prediction_luma_inner)
      (width, height, xoff, yoff, qp, qstride, ip, istride);
  }
}

void get_inter_prediction_chroma_simd(int width, int height, int xoff, int yoff,
                                      unsigned char *restrict qp, int qstride,
                                      const unsigned char *restrict ip, int istride) {
  static const ALIGN(16) int16_t coeffs[8][4] = {
    { 0, 64,  0,  0},
    {-2, 58, 10, -2},
    {-4, 54, 16, -2},
    {-4, 44, 28, -4},
    {-4, 36, 36, -4},
    {-4, 28, 44, -4},
    {-2, 16, 54, -4},
    {-2, 10, 58, -2}
  };

  const v128 c0 = v128_dup_16(coeffs[yoff][0]);
  const v128 c1 = v128_dup_16(coeffs[yoff][1]);
  const v128 c2 = v128_dup_16(coeffs[yoff][2]);
  const v128 c3 = v128_dup_16(coeffs[yoff][3]);
  const v128 round = v128_dup_32(2048);
  const v64 filter = v64_load_aligned(coeffs[xoff]);
  int i;

  if (width == 4) {
    v128 in0 = v128_unpack_u8_s16(v64_load_unaligned(ip - 1*istride - 1));
    v128 in1 = v128_unpack_u8_s16(v64_load_unaligned(ip + 0*istride - 1));
    v128 in2 = v128_unpack_u8_s16(v64_load_unaligned(ip + 1*istride - 1));
    int i;

    for (i = 0; i < height; i++) {
      v128 in3 = v128_unpack_u8_s16(v64_load_unaligned(ip + (i+2)*istride - 1));
      v128 out1 = v128_add_16(v128_add_16(v128_add_16(v128_mullo_s16(c0, in0), v128_mullo_s16(c1, in1)), v128_mullo_s16(c2, in2)), v128_mullo_s16(c3, in3));

      v128 hor_out = v128_shr_n_s32(v128_add_32(v128_from_32(v64_dotp_s16(v128_low_v64(v128_shr_n_byte(out1, 6)), filter),
                                                             v64_dotp_s16(v128_low_v64(v128_shr_n_byte(out1, 4)), filter),
                                                             v64_dotp_s16(v128_low_v64(v128_shr_n_byte(out1, 2)), filter),
                                                             v64_dotp_s16(v128_low_v64(out1), filter)), round), 12);
      v64 out = v64_pack_s32_s16(v128_high_v64(hor_out), v128_low_v64(hor_out));
      u32_store_aligned(qp + qstride * i, v64_low_u32(v64_pack_s16_u8(out, out)));

      in0 = in1;
      in1 = in2;
      in2 = in3;
    }
  } else {
    int j;

    for (j = 0; j < width; j += 8) {
      v128 load0 = v128_load_unaligned(ip - 1*istride + j - 1);
      v128 load1 = v128_load_unaligned(ip + 0*istride + j - 1);
      v128 load2 = v128_load_unaligned(ip + 1*istride + j - 1);
      v128 in00 = v128_unpacklo_u8_s16(load0);
      v128 in01 = v128_unpacklo_u8_s16(load1);
      v128 in02 = v128_unpacklo_u8_s16(load2);
      v128 in10 = v128_unpackhi_u8_s16(load0);
      v128 in11 = v128_unpackhi_u8_s16(load1);
      v128 in12 = v128_unpackhi_u8_s16(load2);

      for (i = 0; i < height; i++) {
        v128 load3 = v128_load_unaligned(ip + (i+2)*istride + j - 1);
        v128 in03 = v128_unpacklo_u8_s16(load3);
        v128 in13 = v128_unpackhi_u8_s16(load3);

        /* Vertical */
        v128 out0 = v128_add_16(v128_add_16(v128_add_16(v128_mullo_s16(c0, in00), v128_mullo_s16(c1, in01)), v128_mullo_s16(c2, in02)), v128_mullo_s16(c3, in03));

        v128 out1 = v128_add_16(v128_add_16(v128_add_16(v128_mullo_s16(c0, in10), v128_mullo_s16(c1, in11)), v128_mullo_s16(c2, in12)), v128_mullo_s16(c3, in13));

        /* Horizontal */
        uint64_t in0 = v64_dotp_s16(v128_low_v64(out0), filter);
        uint64_t in1 = v64_dotp_s16(v128_low_v64(v128_shr_n_byte(out0, 2)), filter);
        uint64_t in2 = v64_dotp_s16(v128_low_v64(v128_shr_n_byte(out0, 4)), filter);
        uint64_t in3 = v64_dotp_s16(v128_low_v64(v128_shr_n_byte(out0, 6)), filter);
        uint64_t in4 = v64_dotp_s16(v128_high_v64(out0), filter);
        uint64_t in5 = v64_dotp_s16(v128_low_v64(v128_align(out1, out0, 10)), filter);
        uint64_t in6 = v64_dotp_s16(v128_low_v64(v128_align(out1, out0, 12)), filter);
        uint64_t in7 = v64_dotp_s16(v128_low_v64(v128_align(out1, out0, 14)), filter);

        v128 out = v128_pack_s32_s16(v128_shr_n_s32(v128_add_32(v128_from_32(in7, in6, in5, in4), round), 12),
                                     v128_shr_n_s32(v128_add_32(v128_from_32(in3, in2, in1, in0), round), 12));
        v64_store_aligned(qp + qstride * i + j, v128_low_v64(v128_pack_s16_u8(out, out)));

        /* Shift input one line up */
        in00 = in01;
        in01 = in02;
        in02 = in03;

        in10 = in11;
        in11 = in12;
        in12 = in13;
      }
    }
  }
}
