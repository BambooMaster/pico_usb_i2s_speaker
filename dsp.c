/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 BambooMaster
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdatomic.h>
#include <string.h>
#include "pico/stdlib.h"

#include "i2s.h"
#include "dsp.h"

#define DEQUEUE_LEN_X8  24
#define DEQUEUE_LEN_X4  48
#define HB_DEQUEUE_MAX_LEN MAX(DEQUEUE_LEN_X8 , DEQUEUE_LEN_X4)

#define HB_TAP_NUM(fir_len)           (((fir_len) + 1) / 4)
#define HB_DELAY_NUM(fir_len)         (((fir_len) + 1) / 2)
#define HB_STATE_LEN(fir_len, in_len) ((HB_DELAY_NUM(fir_len) - 1) + (in_len))

// 44.1kHz Fpass:20kHz Astop:100dB
#define HB_X8_1ST_FIR_LEN 375
static const float __not_in_flash("hb_coeffs") hb_x8_1st_coeff[] = {
  -1.56245378e-07f,  1.88480221e-07f, -3.00697660e-07f,  4.55878596e-07f, -6.65623327e-07f,  9.43783374e-07f, -1.30677725e-06f,  1.77393365e-06f, -2.36784386e-06f,  3.11475097e-06f,
  -4.04497223e-06f,  5.19333010e-06f, -6.59963144e-06f,  8.30914732e-06f, -1.03731393e-05f,  1.28494014e-05f, -1.58028100e-05f,  1.93059221e-05f, -2.34395818e-05f,  2.82935304e-05f,
  -3.39670547e-05f,  4.05696289e-05f, -4.82215698e-05f,  5.70547277e-05f, -6.72131415e-05f,  7.88537324e-05f, -9.21469924e-05f,  1.07277657e-04f, -1.24445389e-04f,  1.43865507e-04f,
  -1.65769502e-04f,  1.90406122e-04f, -2.18041247e-04f,  2.48959782e-04f, -2.83464892e-04f,  3.21879871e-04f, -3.64548328e-04f,  4.11834659e-04f, -4.64125212e-04f,  5.21828946e-04f,
  -5.85377994e-04f,  6.55228672e-04f, -7.31862533e-04f,  8.15787325e-04f, -9.07538061e-04f,  1.00767842e-03f, -1.11680236e-03f,  1.23553583e-03f, -1.36453881e-03f,  1.50450767e-03f,
  -1.65617812e-03f,  1.82032855e-03f, -1.99778399e-03f,  2.18942075e-03f, -2.39617208e-03f,  2.61903478e-03f, -2.85907720e-03f,  3.11744861e-03f, -3.39539053e-03f,  3.69425020e-03f,
  -4.01549685e-03f,  4.36074117e-03f, -4.73175893e-03f,  5.13051929e-03f, -5.55921964e-03f,  6.02032745e-03f, -6.51663246e-03f,  7.05131021e-03f, -7.62800141e-03f,  8.25091085e-03f,
  -8.92493191e-03f,  9.65580457e-03f, -1.04503180e-02f,  1.13165725e-02f, -1.22643222e-02f,  1.33054298e-02f, -1.44544761e-02f,  1.57295929e-02f, -1.71536169e-02f,  1.87557219e-02f,
  -2.05737763e-02f,  2.26578318e-02f, -2.50754320e-02f,  2.79199544e-02f, -3.13242197e-02f,  3.54837079e-02f, -4.06983456e-02f,  4.74528654e-02f, -5.65848279e-02f,  6.96766714e-02f,
  -9.01198435e-02f,  1.26732844e-01f, -2.11851433e-01f,  6.36501305e-01f
};

// 44.1kHz Fpass:20kHz Astop:100dB
#define HB_X8_2ND_FIR_LEN 31
static const float __not_in_flash("hb_coeffs") hb_x8_2nd_coeff[] = {
  -5.53624077e-05f,  5.40443996e-04f, -2.79532850e-03f,  1.01362837e-02f, -2.90851350e-02f,  7.20354395e-02f, -1.73331799e-01f,  6.22555395e-01f
};

// 44.1kHz Fpass:20kHz Astop:100dB
#define HB_X8_3RD_FIR_LEN 19
static const float __not_in_flash("hb_coeffs") hb_x8_3rd_coeff[] = {
  7.33308556e-04f, -7.47490191e-03f,  3.81132885e-02f, -1.39322202e-01f,  6.07950521e-01f
};

// 96kHz Fpass:43.5kHz Astop:100dB
#define HB_X4_1ST_FIR_LEN 375
static const float __not_in_flash("hb_coeffs") hb_x4_1st_coeff[] = {
  -1.56245378e-07f,  1.88480221e-07f, -3.00697660e-07f,  4.55878596e-07f, -6.65623327e-07f,  9.43783374e-07f, -1.30677725e-06f,  1.77393365e-06f, -2.36784386e-06f,  3.11475097e-06f,
  -4.04497223e-06f,  5.19333010e-06f, -6.59963144e-06f,  8.30914732e-06f, -1.03731393e-05f,  1.28494014e-05f, -1.58028100e-05f,  1.93059221e-05f, -2.34395818e-05f,  2.82935304e-05f,
  -3.39670547e-05f,  4.05696289e-05f, -4.82215698e-05f,  5.70547277e-05f, -6.72131415e-05f,  7.88537324e-05f, -9.21469924e-05f,  1.07277657e-04f, -1.24445389e-04f,  1.43865507e-04f,
  -1.65769502e-04f,  1.90406122e-04f, -2.18041247e-04f,  2.48959782e-04f, -2.83464892e-04f,  3.21879871e-04f, -3.64548328e-04f,  4.11834659e-04f, -4.64125212e-04f,  5.21828946e-04f,
  -5.85377994e-04f,  6.55228672e-04f, -7.31862533e-04f,  8.15787325e-04f, -9.07538061e-04f,  1.00767842e-03f, -1.11680236e-03f,  1.23553583e-03f, -1.36453881e-03f,  1.50450767e-03f,
  -1.65617812e-03f,  1.82032855e-03f, -1.99778399e-03f,  2.18942075e-03f, -2.39617208e-03f,  2.61903478e-03f, -2.85907720e-03f,  3.11744861e-03f, -3.39539053e-03f,  3.69425020e-03f,
  -4.01549685e-03f,  4.36074117e-03f, -4.73175893e-03f,  5.13051929e-03f, -5.55921964e-03f,  6.02032745e-03f, -6.51663246e-03f,  7.05131021e-03f, -7.62800141e-03f,  8.25091085e-03f,
  -8.92493191e-03f,  9.65580457e-03f, -1.04503180e-02f,  1.13165725e-02f, -1.22643222e-02f,  1.33054298e-02f, -1.44544761e-02f,  1.57295929e-02f, -1.71536169e-02f,  1.87557219e-02f,
  -2.05737763e-02f,  2.26578318e-02f, -2.50754320e-02f,  2.79199544e-02f, -3.13242197e-02f,  3.54837079e-02f, -4.06983456e-02f,  4.74528654e-02f, -5.65848279e-02f,  6.96766714e-02f,
  -9.01198435e-02f,  1.26732844e-01f, -2.11851433e-01f,  6.36501305e-01f
};

// 96kHz Fpass:43.5kHz Astop:100dB
#define HB_X4_2ND_FIR_LEN 31
static const float __not_in_flash("hb_coeffs") hb_x4_2nd_coeff[] = {
  -5.53624077e-05f,  5.40443996e-04f, -2.79532850e-03f,  1.01362837e-02f, -2.90851350e-02f,  7.20354395e-02f, -1.73331799e-01f,  6.22555395e-01f
};

// x8 L
static float __attribute__((aligned(8))) hb_x8_1st_state_l[HB_STATE_LEN(HB_X8_1ST_FIR_LEN, DEQUEUE_LEN_X8)] = {0.0f};
static float __attribute__((aligned(8))) hb_x8_2nd_state_l[HB_STATE_LEN(HB_X8_2ND_FIR_LEN, DEQUEUE_LEN_X8 * 2)] = {0.0f};
static float __attribute__((aligned(8))) hb_x8_3rd_state_l[HB_STATE_LEN(HB_X8_3RD_FIR_LEN, DEQUEUE_LEN_X8 * 4)] = {0.0f};

// x8 R
static float __attribute__((aligned(8))) hb_x8_1st_state_r[HB_STATE_LEN(HB_X8_1ST_FIR_LEN, DEQUEUE_LEN_X8)] = {0.0f};
static float __attribute__((aligned(8))) hb_x8_2nd_state_r[HB_STATE_LEN(HB_X8_2ND_FIR_LEN, DEQUEUE_LEN_X8 * 2)] = {0.0f};
static float __attribute__((aligned(8))) hb_x8_3rd_state_r[HB_STATE_LEN(HB_X8_3RD_FIR_LEN, DEQUEUE_LEN_X8 * 4)] = {0.0f};

// x4 L
static float __attribute__((aligned(8))) hb_x4_1st_state_l[HB_STATE_LEN(HB_X4_1ST_FIR_LEN, DEQUEUE_LEN_X4)] = {0.0f};
static float __attribute__((aligned(8))) hb_x4_2nd_state_l[HB_STATE_LEN(HB_X4_2ND_FIR_LEN, DEQUEUE_LEN_X4 * 2)] = {0.0f};

// x4 R
static float __attribute__((aligned(8))) hb_x4_1st_state_r[HB_STATE_LEN(HB_X4_1ST_FIR_LEN, DEQUEUE_LEN_X4)] = {0.0f};
static float __attribute__((aligned(8))) hb_x4_2nd_state_r[HB_STATE_LEN(HB_X4_2ND_FIR_LEN, DEQUEUE_LEN_X4 * 2)] = {0.0f};

// 演算用
static float hb_temp_a_l[HB_DEQUEUE_MAX_LEN * 8];
static float hb_temp_b_l[HB_DEQUEUE_MAX_LEN * 8];
static float hb_temp_a_r[HB_DEQUEUE_MAX_LEN * 8];
static float hb_temp_b_r[HB_DEQUEUE_MAX_LEN * 8];

static atomic_uint dsp_sample_rate_hz = 44100;

// 音量用テーブル
static const float gain_table[101] = {
  1.000000000e+00f, 8.912509084e-01f, 7.943282127e-01f, 7.079457641e-01f, 6.309573650e-01f, 5.623413324e-01f, 5.011872053e-01f, 4.466835856e-01f, 3.981071711e-01f, 3.548133969e-01f,
  3.162277639e-01f, 2.818382978e-01f, 2.511886358e-01f, 2.238721102e-01f, 1.995262355e-01f, 1.778279394e-01f, 1.584893167e-01f, 1.412537545e-01f, 1.258925349e-01f, 1.122018471e-01f,
  1.000000015e-01f, 8.912509680e-02f, 7.943282276e-02f, 7.079457492e-02f, 6.309573352e-02f, 5.623413250e-02f, 5.011872202e-02f, 4.466835782e-02f, 3.981071711e-02f, 3.548133746e-02f,
  3.162277490e-02f, 2.818382904e-02f, 2.511886507e-02f, 2.238721214e-02f, 1.995262317e-02f, 1.778279431e-02f, 1.584893279e-02f, 1.412537508e-02f, 1.258925442e-02f, 1.122018415e-02f,
  9.999999776e-03f, 8.912509307e-03f, 7.943281904e-03f, 7.079457864e-03f, 6.309573539e-03f, 5.623413250e-03f, 5.011872388e-03f, 4.466835875e-03f, 3.981071524e-03f, 3.548133885e-03f,
  3.162277630e-03f, 2.818383044e-03f, 2.511886414e-03f, 2.238721121e-03f, 1.995262224e-03f, 1.778279431e-03f, 1.584893209e-03f, 1.412537531e-03f, 1.258925418e-03f, 1.122018439e-03f,
  1.000000047e-03f, 8.912509657e-04f, 7.943282253e-04f, 7.079457864e-04f, 6.309573655e-04f, 5.623413017e-04f, 5.011872272e-04f, 4.466835817e-04f, 3.981071641e-04f, 3.548133827e-04f,
  3.162277571e-04f, 2.818382927e-04f, 2.511886414e-04f, 2.238721208e-04f, 1.995262282e-04f, 1.778279402e-04f, 1.584893180e-04f, 1.412537531e-04f, 1.258925477e-04f, 1.122018439e-04f,
  9.999999747e-05f, 8.912509657e-05f, 7.943282253e-05f, 7.079458010e-05f, 6.309573655e-05f, 5.623413381e-05f, 5.011872418e-05f, 4.466835890e-05f, 3.981071859e-05f, 3.548133827e-05f,
  3.162277790e-05f, 2.818382927e-05f, 2.511886487e-05f, 2.238721208e-05f, 1.995262392e-05f, 1.778279329e-05f, 1.584893107e-05f, 1.412537586e-05f, 1.258925386e-05f, 1.122018421e-05f,
  9.999999747e-06f, 
};

// ハーフバンドフィルタ 2倍インターポレーション
// 入力サンプル数は4の倍数
static inline __attribute__((always_inline)) void hb_interp2x_f32(
  float * restrict state,
  const float * restrict coeff,
  const int fir_len,
  const float * restrict in,
  float * restrict out,
  const int input_len
){
  const int tap_num = HB_TAP_NUM(fir_len);
  const int delay_num = HB_DELAY_NUM(fir_len);
  const int hist_len = delay_num - 1;

  memmove(state, &state[input_len], hist_len * sizeof(float));
  memcpy(&state[hist_len], in, input_len * sizeof(float));

  for (int i = 0; i < input_len; i += 4){
    float acc_0 = 0.0f, acc_1 = 0.0f, acc_2 = 0.0f, acc_3 = 0.0f;

    if (tap_num >= 8){
      int j = 0;
      for (; j <= tap_num - 4; j += 4){
        const float c0 = coeff[j];
        acc_0 += (state[i + j + 0] + state[i + hist_len - j + 0]) * c0;
        acc_1 += (state[i + 1 + j] + state[i + 1 + hist_len - j]) * c0;
        acc_2 += (state[i + 2 + j] + state[i + 2 + hist_len - j]) * c0;
        acc_3 += (state[i + 3 + j] + state[i + 3 + hist_len - j]) * c0;

        const float c1 = coeff[j + 1];
        acc_0 += (state[i + 0 + j + 1] + state[i + 0 + hist_len - j - 1]) * c1;
        acc_1 += (state[i + 1 + j + 1] + state[i + 1 + hist_len - j - 1]) * c1;
        acc_2 += (state[i + 2 + j + 1] + state[i + 2 + hist_len - j - 1]) * c1;
        acc_3 += (state[i + 3 + j + 1] + state[i + 3 + hist_len - j - 1]) * c1;

        const float c2 = coeff[j + 2];
        acc_0 += (state[i + 0 + j + 2] + state[i + 0 + hist_len - j - 2]) * c2;
        acc_1 += (state[i + 1 + j + 2] + state[i + 1 + hist_len - j - 2]) * c2;
        acc_2 += (state[i + 2 + j + 2] + state[i + 2 + hist_len - j - 2]) * c2;
        acc_3 += (state[i + 3 + j + 2] + state[i + 3 + hist_len - j - 2]) * c2;

        const float c3 = coeff[j + 3];
        acc_0 += (state[i + 0 + j + 3] + state[i + 0 + hist_len - j - 3]) * c3;
        acc_1 += (state[i + 1 + j + 3] + state[i + 1 + hist_len - j - 3]) * c3;
        acc_2 += (state[i + 2 + j + 3] + state[i + 2 + hist_len - j - 3]) * c3;
        acc_3 += (state[i + 3 + j + 3] + state[i + 3 + hist_len - j - 3]) * c3;
      }
      for (; j < tap_num; j++){
        const float c = coeff[j];
        acc_0 += (state[i + 0 + j] + state[i + 0 + hist_len - j]) * c;
        acc_1 += (state[i + 1 + j] + state[i + 1 + hist_len - j]) * c;
        acc_2 += (state[i + 2 + j] + state[i + 2 + hist_len - j]) * c;
        acc_3 += (state[i + 3 + j] + state[i + 3 + hist_len - j]) * c;
      }
    }
    else {
      const float *pf = &state[i];
      asm volatile("" : "+r"(pf));
      const float *pb = &pf[hist_len];

      for (int j = 0; j < tap_num; j++){
        const float c = coeff[j];
        acc_0 += (pf[j] + pb[- j]) * c;
        acc_1 += (pf[j + 1] + pb[- j + 1]) * c;
        acc_2 += (pf[j + 2] + pb[- j + 2]) * c;
        acc_3 += (pf[j + 3] + pb[- j + 3]) * c;
      }
    }

    out[i * 2 + 0] = acc_0;
    out[i * 2 + 2] = acc_1;
    out[i * 2 + 4] = acc_2;
    out[i * 2 + 6] = acc_3;

    out[i * 2 + 1] = state[i + 0 + tap_num];
    out[i * 2 + 3] = state[i + 1 + tap_num];
    out[i * 2 + 5] = state[i + 2 + tap_num];
    out[i * 2 + 7] = state[i + 3 + tap_num];
  }
}

// 8倍インターポレーション　1回目
static void __attribute__((noinline)) __not_in_flash_func(hb_interp2x_f32_x8_1st)(
  const float * restrict in_l,
  const float * restrict in_r,
  float * restrict out_l,
  float * restrict out_r
){
  hb_interp2x_f32(hb_x8_1st_state_l, hb_x8_1st_coeff, HB_X8_1ST_FIR_LEN,
    in_l, out_l, DEQUEUE_LEN_X8);
  hb_interp2x_f32(hb_x8_1st_state_r, hb_x8_1st_coeff, HB_X8_1ST_FIR_LEN,
    in_r, out_r, DEQUEUE_LEN_X8);
}

// 8倍インターポレーション　2回目
static void __attribute__((noinline)) __not_in_flash_func(hb_interp2x_f32_x8_2nd)(
  const float * restrict in_l,
  const float * restrict in_r,
  float * restrict out_l,
  float * restrict out_r
){
  hb_interp2x_f32(hb_x8_2nd_state_l, hb_x8_2nd_coeff, HB_X8_2ND_FIR_LEN,
    in_l, out_l, DEQUEUE_LEN_X8 * 2);
  hb_interp2x_f32(hb_x8_2nd_state_r, hb_x8_2nd_coeff, HB_X8_2ND_FIR_LEN,
    in_r, out_r, DEQUEUE_LEN_X8 * 2);
}

// 8倍インターポレーション　3回目
static void __attribute__((noinline)) __not_in_flash_func(hb_interp2x_f32_x8_3rd)(
  const float * restrict in_l,
  const float * restrict in_r,
  float * restrict out_l,
  float * restrict out_r
){
  hb_interp2x_f32(hb_x8_3rd_state_l, hb_x8_3rd_coeff, HB_X8_3RD_FIR_LEN,
    in_l, out_l, DEQUEUE_LEN_X8 * 4);
  hb_interp2x_f32(hb_x8_3rd_state_r, hb_x8_3rd_coeff, HB_X8_3RD_FIR_LEN,
    in_r, out_r, DEQUEUE_LEN_X8 * 4);
}

// 4倍インターポレーション　1回目
static void __attribute__((noinline)) __not_in_flash_func(hb_interp2x_f32_x4_1st)(
  const float * restrict in_l,
  const float * restrict in_r,
  float * restrict out_l,
  float * restrict out_r
){
  hb_interp2x_f32(hb_x4_1st_state_l, hb_x4_1st_coeff, HB_X4_1ST_FIR_LEN,
    in_l, out_l, DEQUEUE_LEN_X4);
  hb_interp2x_f32(hb_x4_1st_state_r, hb_x4_1st_coeff, HB_X4_1ST_FIR_LEN,
    in_r, out_r, DEQUEUE_LEN_X4);
}

// 4倍インターポレーション　2回目
static void __attribute__((noinline)) __not_in_flash_func(hb_interp2x_f32_x4_2nd)(
  const float * restrict in_l,
  const float * restrict in_r,
  float * restrict out_l,
  float * restrict out_r
){
  hb_interp2x_f32(hb_x4_2nd_state_l, hb_x4_2nd_coeff, HB_X4_2ND_FIR_LEN,
    in_l, out_l, DEQUEUE_LEN_X4 * 2);
  hb_interp2x_f32(hb_x4_2nd_state_r, hb_x4_2nd_coeff, HB_X4_2ND_FIR_LEN,
    in_r, out_r, DEQUEUE_LEN_X4 * 2);
}

void dsp_set_sample_rate_hz(uint32_t sample_rate_hz){
  switch(sample_rate_hz){
    case 44100:
    case 88200:
      i2s_change_clock(352800);
      break;

    case 48000:
    case 96000:
      i2s_change_clock(384000);
      break;

    default:
      return;
  }
  atomic_store(&dsp_sample_rate_hz, sample_rate_hz);
}

uint32_t dsp_get_sample_rate_hz(void){
  return atomic_load(&dsp_sample_rate_hz);
}

static float volume_to_gain(int16_t v){
  int16_t vol_index;
  vol_index = -v >> 8;
  if (vol_index > 100) vol_index = 100;
  else if (vol_index < 0) vol_index = 0;
  return gain_table[vol_index];
}

void core1_main_dsp(void){
  int dma_sample;
  bool mute = false;
  int buf_length;
  static int32_t dma_buf_a[2][HB_DEQUEUE_MAX_LEN * 2 * 8], dma_buf_b[2][HB_DEQUEUE_MAX_LEN * 2 * 8];
  uint8_t dma_use = 0;
  int dequeue_len;

  int sample;
  int32_t i2s_buf_l[HB_DEQUEUE_MAX_LEN * 8], i2s_buf_r[HB_DEQUEUE_MAX_LEN * 8];

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

  while (1){
    buf_length = i2s_get_queue_length();
    int input_sample_rate_hz = dsp_get_sample_rate_hz();
    // 0.5ms分ずつi2sに送る
    switch(input_sample_rate_hz){
      case 44100:
      case 48000:
        dequeue_len = DEQUEUE_LEN_X8;
        break;
      case 88200:
      case 96000:
        dequeue_len = DEQUEUE_LEN_X4;
        break;
    }

    // if (dequeue_len > HB_DEQUEUE_MAX_LEN) {
    //   dequeue_len = HB_DEQUEUE_MAX_LEN;
    // }

    // printf("%3d\n", buf_length);

    // i2sキューが一定以上溜まったらミュート解除
    if (buf_length == 0 && mute == false){
      mute = true;
      gpio_put(PICO_DEFAULT_LED_PIN, 0);
    }
    else if (buf_length >= (dequeue_len * 3) && mute == true){
      mute = false;
      gpio_put(PICO_DEFAULT_LED_PIN, 1);
    }

    if (mute == false){
      // i2sキューから取り出す
      sample = i2s_dequeue(i2s_buf_l, i2s_buf_r, dequeue_len);

      // キューから取り出したデータ量が要求より少ない場合は、0埋めしてミュート状態へ
      if (sample < dequeue_len){
        for (int i = sample; i < dequeue_len; i++){
          i2s_buf_l[i] = 0;
          i2s_buf_r[i] = 0;
        }
        sample = dequeue_len;
        mute = true;
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
      }
    }
    else {
      // ミュート状態の時は0を送信
      for (int i = 0; i < dequeue_len; i++){
        i2s_buf_l[i] = 0;
        i2s_buf_r[i] = 0;
      }
      sample = dequeue_len;
    }

    // 音量設定
    int16_t v;
    float gain_l, gain_r;
    v = i2s_get_volume_l();
    gain_l = volume_to_gain(v);
    v = i2s_get_volume_r();
    gain_r = volume_to_gain(v);

    // int32_tをfloatに変換
    for (int i = 0; i < sample; i++){
      hb_temp_b_l[i] = (float)i2s_buf_l[i] * gain_l;
      hb_temp_b_r[i] = (float)i2s_buf_r[i] * gain_r;
    }

    if (input_sample_rate_hz <= 48000){
      // インターポレーション
      hb_interp2x_f32_x8_1st(hb_temp_b_l, hb_temp_b_r, hb_temp_a_l, hb_temp_a_r);
      hb_interp2x_f32_x8_2nd(hb_temp_a_l, hb_temp_a_r, hb_temp_b_l, hb_temp_b_r);
      hb_interp2x_f32_x8_3rd(hb_temp_b_l, hb_temp_b_r, hb_temp_a_l, hb_temp_a_r);
      sample *= 8;

      // floatをint32_tに変換
      for (int i = 0; i < sample; i++){
        i2s_buf_l[i] = (int32_t)hb_temp_a_l[i];
        i2s_buf_r[i] = (int32_t)hb_temp_a_r[i];
      }
    }
    else{
      // インターポレーション
      hb_interp2x_f32_x4_1st(hb_temp_b_l, hb_temp_b_r, hb_temp_a_l, hb_temp_a_r);
      hb_interp2x_f32_x4_2nd(hb_temp_a_l, hb_temp_a_r, hb_temp_b_l, hb_temp_b_r);
      sample *= 4;

      // floatをint32_tに変換
      for (int i = 0; i < sample; i++){
        i2s_buf_l[i] = (int32_t)hb_temp_b_l[i];
        i2s_buf_r[i] = (int32_t)hb_temp_b_r[i];
      }
    }

    // pio送信形式に変換
    dma_sample = i2s_format_piodata(i2s_buf_l, i2s_buf_r, sample, dma_buf_a[dma_use], dma_buf_b[dma_use]);

    // dmaが終わるまで待機
    i2s_dma_transfer_blocking(dma_buf_a[dma_use], dma_buf_b[dma_use], dma_sample);
    dma_use ^= 1;
  }
}
