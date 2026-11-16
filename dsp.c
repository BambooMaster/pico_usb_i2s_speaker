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

#define HB_TAP_NUM(fir_len)           (((fir_len) + 1) / 4)
#define HB_DELAY_NUM(fir_len)         (((fir_len) + 1) / 2)
#define HB_STATE_LEN(fir_len, in_len) ((HB_DELAY_NUM(fir_len) - 1) + (in_len))

// 44.1kHz Fpass:20kHz Astop:100dB
static const float __not_in_flash("fir_coeffs") fir_x8_1st_tap[] = {
  2.11153318e-05f, -3.59259616e-05f,  6.54839896e-05f, -1.10059111e-04f,  1.74638867e-04f, -2.65238206e-04f,  3.88988730e-04f, -5.54229607e-04f,  7.70599426e-04f, -1.04912203e-03f,
  1.40230009e-03f, -1.84422101e-03f,  2.39068808e-03f, -3.05942240e-03f,  3.87035108e-03f, -4.84601748e-03f,  6.01221082e-03f, -7.39890451e-03f,  9.04165948e-03f, -1.09837862e-02f,
  1.32796197e-02f, -1.59996657e-02f,  1.92387487e-02f, -2.31295663e-02f,  2.78657410e-02f, -3.37436626e-02f,  4.12427866e-02f, -5.11937210e-02f,  6.51704275e-02f, -8.65544596e-02f,
  1.24153200e-01f, -2.10290320e-01f,  6.35978697e-01f
};

// 44.1kHz Fpass:20kHz Astop:100dB
static const float __not_in_flash("fir_coeffs") fir_x8_2nd_tap[] = {
  -5.03897781e-04f,  4.11250476e-03f, -1.80854062e-02f,  5.76758278e-02f, -1.60646345e-01f,  6.17445722e-01f
};

// 44.1kHz Fpass:20kHz Astop:100dB
static const float __not_in_flash("fir_coeffs") fir_x8_3rd_tap[] = {
  -3.05308615e-03f,  2.67882995e-02f, -1.24565929e-01f,  6.00830415e-01f
};

// 96kHz Fpass:43.5kHz Astop:100dB
static const float __not_in_flash("fir_coeffs") fir_x4_1st_tap[] = {
  1.96687119e-05f, -3.39592525e-05f,  6.23100367e-05f, -1.05287684e-04f,  1.67824982e-04f, -2.55886866e-04f,  3.76573379e-04f, -5.38213046e-04f,  7.50446265e-04f, -1.02432367e-03f,
  1.37239667e-03f, -1.80882174e-03f,  2.34950574e-03f, -3.01229992e-03f,  3.81727526e-03f, -4.78714063e-03f,  5.94786389e-03f, -7.32959621e-03f,  8.96808853e-03f, -1.09068442e-02f,
  1.32003614e-02f, -1.59192981e-02f,  1.91585911e-02f, -2.30510313e-02f,  2.77903035e-02f, -3.36727896e-02f,  4.11779195e-02f, -5.11362193e-02f,  6.51215262e-02f, -8.65152404e-02f,
  1.24124540e-01f, -2.10272857e-01f,  6.35972830e-01f
};

// 96kHz Fpass:43.5kHz Astop:100dB
static const float __not_in_flash("fir_coeffs") fir_x4_2nd_tap[] = {
  -5.02622878e-04f,  4.10592295e-03f, -1.80685431e-02f,  5.76490732e-02f, -1.60619935e-01f,  6.17434528e-01f
};

#define FIR_X8_1ST_TAP_NUM 131
#define FIR_X8_2ND_TAP_NUM 23
#define FIR_X8_3RD_TAP_NUM 15
#define FIR_X4_1ST_TAP_NUM 131
#define FIR_X4_2ND_TAP_NUM 23

// x8 L
static float fir_state_x8_1st_l[HB_STATE_LEN(FIR_X8_1ST_TAP_NUM, DEQUEUE_LEN_X8)] = {0.0f};
static float fir_state_x8_2nd_l[HB_STATE_LEN(FIR_X8_2ND_TAP_NUM, DEQUEUE_LEN_X8 * 2)] = {0.0f};
static float fir_state_x8_3rd_l[HB_STATE_LEN(FIR_X8_3RD_TAP_NUM, DEQUEUE_LEN_X8 * 4)] = {0.0f};

// x8 R
static float fir_state_x8_1st_r[HB_STATE_LEN(FIR_X8_1ST_TAP_NUM, DEQUEUE_LEN_X8)] = {0.0f};
static float fir_state_x8_2nd_r[HB_STATE_LEN(FIR_X8_2ND_TAP_NUM, DEQUEUE_LEN_X8 * 2)] = {0.0f};
static float fir_state_x8_3rd_r[HB_STATE_LEN(FIR_X8_3RD_TAP_NUM, DEQUEUE_LEN_X8 * 4)] = {0.0f};

// x4 L
static float fir_state_x4_1st_l[HB_STATE_LEN(FIR_X4_1ST_TAP_NUM, DEQUEUE_LEN_X4)] = {0.0f};
static float fir_state_x4_2nd_l[HB_STATE_LEN(FIR_X4_2ND_TAP_NUM, DEQUEUE_LEN_X4 * 2)] = {0.0f};

// x4 R
static float fir_state_x4_1st_r[HB_STATE_LEN(FIR_X4_1ST_TAP_NUM, DEQUEUE_LEN_X4)] = {0.0f};
static float fir_state_x4_2nd_r[HB_STATE_LEN(FIR_X4_2ND_TAP_NUM, DEQUEUE_LEN_X4 * 2)] = {0.0f};

// 演算用
static float fir_temp_a_l[FIR_DEQUEUE_MAX_LEN * 8];
static float fir_temp_b_l[FIR_DEQUEUE_MAX_LEN * 8];
static float fir_temp_a_r[FIR_DEQUEUE_MAX_LEN * 8];
static float fir_temp_b_r[FIR_DEQUEUE_MAX_LEN * 8];

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
// 入力サンプル数は2の倍数
static inline __attribute__((always_inline)) void hb_interp2x_f32(
  float * restrict state_l,
  float * restrict state_r,
  const float * restrict tap,
  const int tap_num,
  const int delay_num,
  const float * restrict in_l,
  const float * restrict in_r,
  float * restrict out_l,
  float * restrict out_r,
  const int input_len
){
  memmove(state_l, &state_l[input_len], (delay_num - 1) * sizeof(float));
  memmove(state_r, &state_r[input_len], (delay_num - 1) * sizeof(float));
  memcpy(&state_l[delay_num - 1], in_l, input_len * sizeof(float));
  memcpy(&state_r[delay_num - 1], in_r, input_len * sizeof(float));

  for (int i = 0; i < input_len; i += 2){
    float acc_l0 = 0.0f, acc_l1 = 0.0f;
    float acc_r0 = 0.0f, acc_r1 = 0.0f;

    for (int j = 0; j < tap_num; j++){
      const float c = tap[j];
      const float sl_f0 = state_l[i + j];
      const float sl_b0 = state_l[i + (delay_num - 1) - j];
      const float sr_f0 = state_r[i + j];
      const float sr_b0 = state_r[i + (delay_num - 1) - j];
      const float sl_f1 = state_l[i + 1 + j];
      const float sl_b1 = state_l[i + 1 + (delay_num - 1) - j];
      const float sr_f1 = state_r[i + 1 + j];
      const float sr_b1 = state_r[i + 1 + (delay_num - 1) - j];

      acc_l0 +=(sl_f0 + sl_b0) * c;
      acc_r0 +=(sr_f0 + sr_b0) * c;
      acc_l1 +=(sl_f1 + sl_b1) * c;
      acc_r1 +=(sr_f1 + sr_b1) * c;
    }

    out_l[i * 2] = acc_l0;
    out_r[i * 2] = acc_r0;
    out_l[i * 2 + 2] = acc_l1;
    out_r[i * 2 + 2] = acc_r1;

    out_l[i * 2 + 1] = state_l[i + tap_num];
    out_r[i * 2 + 1] = state_r[i + tap_num];
    out_l[i * 2 + 3] = state_l[i + 1 + tap_num];
    out_r[i * 2 + 3] = state_r[i + 1 + tap_num];
  }
}

// 8倍インターポレーション　1回目
static void __attribute__((noinline)) __not_in_flash_func(hb_interp2x_f32_8x_1st)(
  const float * restrict in_l,
  const float * restrict in_r,
  float * restrict out_l,
  float * restrict out_r
){
  hb_interp2x_f32(fir_state_x8_1st_l, fir_state_x8_1st_r, fir_x8_1st_tap,
    HB_TAP_NUM(FIR_X8_1ST_TAP_NUM), HB_DELAY_NUM(FIR_X8_1ST_TAP_NUM),
    in_l, in_r, out_l, out_r, DEQUEUE_LEN_X8);
}

// 8倍インターポレーション　2回目
static void __attribute__((noinline)) __not_in_flash_func(hb_interp2x_f32_8x_2nd)(
  const float * restrict in_l,
  const float * restrict in_r,
  float * restrict out_l,
  float * restrict out_r
){
  hb_interp2x_f32(fir_state_x8_2nd_l, fir_state_x8_2nd_r, fir_x8_2nd_tap,
    HB_TAP_NUM(FIR_X8_2ND_TAP_NUM), HB_DELAY_NUM(FIR_X8_2ND_TAP_NUM),
    in_l, in_r, out_l, out_r, DEQUEUE_LEN_X8 * 2);
}

// 8倍インターポレーション　3回目
static void __attribute__((noinline)) __not_in_flash_func(hb_interp2x_f32_8x_3rd)(
  const float * restrict in_l,
  const float * restrict in_r,
  float * restrict out_l,
  float * restrict out_r
){
  hb_interp2x_f32(fir_state_x8_3rd_l, fir_state_x8_3rd_r, fir_x8_3rd_tap,
    HB_TAP_NUM(FIR_X8_3RD_TAP_NUM), HB_DELAY_NUM(FIR_X8_3RD_TAP_NUM),
    in_l, in_r, out_l, out_r, DEQUEUE_LEN_X8 * 4);
}

// 4倍インターポレーション　1回目
static void __attribute__((noinline)) __not_in_flash_func(hb_interp2x_f32_4x_1st)(
  const float * restrict in_l,
  const float * restrict in_r,
  float * restrict out_l,
  float * restrict out_r
){
  hb_interp2x_f32(fir_state_x4_1st_l, fir_state_x4_1st_r, fir_x4_1st_tap,
    HB_TAP_NUM(FIR_X4_1ST_TAP_NUM), HB_DELAY_NUM(FIR_X4_1ST_TAP_NUM),
    in_l, in_r, out_l, out_r, DEQUEUE_LEN_X4);
}

// 4倍インターポレーション　2回目
static void __attribute__((noinline)) __not_in_flash_func(hb_interp2x_f32_4x_2nd)(
  const float * restrict in_l,
  const float * restrict in_r,
  float * restrict out_l,
  float * restrict out_r
){
  hb_interp2x_f32(fir_state_x4_2nd_l, fir_state_x4_2nd_r, fir_x4_2nd_tap,
    HB_TAP_NUM(FIR_X4_2ND_TAP_NUM), HB_DELAY_NUM(FIR_X4_2ND_TAP_NUM),
    in_l, in_r, out_l, out_r, DEQUEUE_LEN_X4 * 2);
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
  static int32_t dma_buf_a[2][FIR_DEQUEUE_MAX_LEN * 2 * 8], dma_buf_b[2][FIR_DEQUEUE_MAX_LEN * 2 * 8];
  uint8_t dma_use = 0;
  int dequeue_len;

  int sample;
  int32_t i2s_buf_l[FIR_DEQUEUE_MAX_LEN * 8], i2s_buf_r[FIR_DEQUEUE_MAX_LEN * 8];

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

    // if (dequeue_len > FIR_DEQUEUE_MAX_LEN) {
    //   dequeue_len = FIR_DEQUEUE_MAX_LEN;
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
      fir_temp_b_l[i] = (float)i2s_buf_l[i] * gain_l;
      fir_temp_b_r[i] = (float)i2s_buf_r[i] * gain_r;
    }

    if (input_sample_rate_hz <= 48000){
      // インターポレーション
      hb_interp2x_f32_8x_1st(fir_temp_b_l, fir_temp_b_r, fir_temp_a_l, fir_temp_a_r);
      hb_interp2x_f32_8x_2nd(fir_temp_a_l, fir_temp_a_r, fir_temp_b_l, fir_temp_b_r);
      hb_interp2x_f32_8x_3rd(fir_temp_b_l, fir_temp_b_r, fir_temp_a_l, fir_temp_a_r);
      sample *= 8;

      // floatをint32_tに変換
      for (int i = 0; i < sample; i++){
        i2s_buf_l[i] = (int32_t)fir_temp_a_l[i];
        i2s_buf_r[i] = (int32_t)fir_temp_a_r[i];
      }
    }
    else{
      // インターポレーション
      hb_interp2x_f32_4x_1st(fir_temp_b_l, fir_temp_b_r, fir_temp_a_l, fir_temp_a_r);
      hb_interp2x_f32_4x_2nd(fir_temp_a_l, fir_temp_a_r, fir_temp_b_l, fir_temp_b_r);
      sample *= 4;

      // floatをint32_tに変換
      for (int i = 0; i < sample; i++){
        i2s_buf_l[i] = (int32_t)fir_temp_b_l[i];
        i2s_buf_r[i] = (int32_t)fir_temp_b_r[i];
      }
    }

    // pio送信形式に変換
    dma_sample = i2s_format_piodata(i2s_buf_l, i2s_buf_r, sample, dma_buf_a[dma_use], dma_buf_b[dma_use]);

    // dmaが終わるまで待機
    i2s_dma_transfer_blocking(dma_buf_a[dma_use], dma_buf_b[dma_use], dma_sample);
    dma_use ^= 1;
  }
}
