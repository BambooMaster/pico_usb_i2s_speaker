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
#include "pico/stdlib.h"
#include "arm_math.h"

#include "i2s.h"
#include "dsp.h"

// 44.1kHz Fpass:20kHz Fstop:24.1kHz Apass:0.001dB Astop:100dB
static const float fir_x8_1st_tap[] = {
    1.5665506e-05,   3.752056e-05,-3.334434041e-06,-5.414986663e-05,-1.333830255e-06,
  9.205318929e-05,1.057719055e-05,-0.0001451352728,-2.800235234e-05,0.0002184008772,
  5.573674935e-05,-0.0003161516797,-9.754864732e-05,  0.00044373743,0.0001577048533,
  -0.0006069466472,-0.000241337475,0.0008122323779,0.0003544786887,-0.001066644094,
  -0.0005041476688, 0.001377878943,0.0006984113134,-0.001754450263,-0.0009466964402,
   0.002205698518, 0.001259839511,-0.002742206678,-0.001650580554, 0.003376103938,
   0.002133995527,-0.004121766426,-0.002728339517, 0.004996836651, 0.003456274513,
  -0.006023811642,-0.004346848931, 0.007232617121, 0.005438688211,-0.008664827794,
  -0.006785372738,  0.01038078684, 0.008464802057, -0.01247231476, -0.01059638057,
    0.01508672163,   0.0133748604, -0.01847579703, -0.01714285463,  0.02310674265,
     0.0225658901, -0.02995031327, -0.03113060258,  0.04139752686,  0.04697142914,
   -0.06531880051, -0.08746524155,   0.1515951008,   0.4480950236,   0.4480950236,
     0.1515951008, -0.08746524155, -0.06531880051,  0.04697142914,  0.04139752686,
   -0.03113060258, -0.02995031327,   0.0225658901,  0.02310674265, -0.01714285463,
   -0.01847579703,   0.0133748604,  0.01508672163, -0.01059638057, -0.01247231476,
   0.008464802057,  0.01038078684,-0.006785372738,-0.008664827794, 0.005438688211,
   0.007232617121,-0.004346848931,-0.006023811642, 0.003456274513, 0.004996836651,
  -0.002728339517,-0.004121766426, 0.002133995527, 0.003376103938,-0.001650580554,
  -0.002742206678, 0.001259839511, 0.002205698518,-0.0009466964402,-0.001754450263,
  0.0006984113134, 0.001377878943,-0.0005041476688,-0.001066644094,0.0003544786887,
  0.0008122323779,-0.000241337475,-0.0006069466472,0.0001577048533,  0.00044373743,
  -9.754864732e-05,-0.0003161516797,5.573674935e-05,0.0002184008772,-2.800235234e-05,
  -0.0001451352728,1.057719055e-05,9.205318929e-05,-1.333830255e-06,-5.414986663e-05,
  -3.334434041e-06,   3.752056e-05,  1.5665506e-05
};

// 44.1kHz Fpass:20kHz Fstop:64.1kHz Apass:0.001dB Astop:100dB
static const float fir_x8_2nd_tap[] = {
  5.109819904e-05,0.0001867716637,0.0003726014111, 0.000414187205,1.489411687e-07,
  -0.001048396458,-0.002401694423,-0.003032318549,-0.001590578817, 0.002568376018,
   0.008202262223,  0.01182534639, 0.009035444818,-0.002556463238, -0.02020732127,
   -0.03494691476, -0.03414940089,-0.007441996597,  0.04672127217,   0.1176327318,
     0.1847466081,    0.225593552,    0.225593552,   0.1847466081,   0.1176327318,
    0.04672127217,-0.007441996597, -0.03414940089, -0.03494691476, -0.02020732127,
  -0.002556463238, 0.009035444818,  0.01182534639, 0.008202262223, 0.002568376018,
  -0.001590578817,-0.003032318549,-0.002401694423,-0.001048396458,1.489411687e-07,
   0.000414187205,0.0003726014111,0.0001867716637,5.109819904e-05
};

// 96kHz Fpass:40kHz Fstop:52.5kHz Apass:0.001dB Astop:100dB
static const float fir_x4_1st_tap[] = {
  -3.523814303e-05,-4.294828614e-05,4.982847531e-05,0.0001038974369,-6.852608931e-05,
  -0.0002174182009,7.017892494e-05,0.0003957182635,-2.846668576e-05,-0.0006556982407,
  -8.985168097e-05,  0.00100506912,0.0003338342067, -0.00144568237,-0.0007604789571,
   0.001961644739, 0.001440430526,-0.002521312097,-0.002449454274, 0.003066541627,
    0.00387188606,-0.003514180193,-0.005793711171, 0.003746321425, 0.008308733813,
  -0.003608463565, -0.01152137294, 0.002893017605,  0.01557054743,-0.001315793605,
   -0.02067297883,-0.001552238478,  0.02723730169,  0.00644916296, -0.03615242243,
   -0.01491540484,   0.0497116372,  0.03111260757, -0.07567728311, -0.07241614163,
      0.163821891,      0.4342767,      0.4342767,    0.163821891, -0.07241614163,
   -0.07567728311,  0.03111260757,   0.0497116372, -0.01491540484, -0.03615242243,
    0.00644916296,  0.02723730169,-0.001552238478, -0.02067297883,-0.001315793605,
    0.01557054743, 0.002893017605, -0.01152137294,-0.003608463565, 0.008308733813,
   0.003746321425,-0.005793711171,-0.003514180193,  0.00387188606, 0.003066541627,
  -0.002449454274,-0.002521312097, 0.001440430526, 0.001961644739,-0.0007604789571,
   -0.00144568237,0.0003338342067,  0.00100506912,-8.985168097e-05,-0.0006556982407,
  -2.846668576e-05,0.0003957182635,7.017892494e-05,-0.0002174182009,-6.852608931e-05,
  0.0001038974369,4.982847531e-05,-4.294828614e-05,-3.523814303e-05
};

// 96kHz Fpass:40kHz Fstop:52.5kHz Apass:0.001dB Astop:100dB
static const float fir_x4_2nd_tap[] = {
  -0.0002829428995,-0.002220998751,-0.0004318853607,  0.01359409746,  0.00944875367,
   -0.04742092267, -0.05078744888,   0.1535621583,   0.4244935215,   0.4244935215,
     0.1535621583, -0.05078744888, -0.04742092267,  0.00944875367,  0.01359409746,
  -0.0004318853607,-0.002220998751,-0.0002829428995
};

#define FIR_X8_1ST_TAP_NUM (sizeof(fir_x8_1st_tap) / sizeof(fir_x8_1st_tap[0]))
#define FIR_X8_2ND_TAP_NUM (sizeof(fir_x8_2nd_tap) / sizeof(fir_x8_2nd_tap[0]))
#define FIR_X4_1ST_TAP_NUM (sizeof(fir_x4_1st_tap) / sizeof(fir_x4_1st_tap[0]))
#define FIR_X4_2ND_TAP_NUM (sizeof(fir_x4_2nd_tap) / sizeof(fir_x4_2nd_tap[0]))

// x8 L
static arm_fir_interpolate_instance_f32 fir_inst_x8_1st_l;
static arm_fir_interpolate_instance_f32 fir_inst_x8_2nd_l;
static float fir_state_x8_1st_l[(FIR_X8_1ST_TAP_NUM / 2) + FIR_DEQUEUE_MAX_LEN - 1]     = {0.0f};
static float fir_state_x8_2nd_l[(FIR_X8_2ND_TAP_NUM / 4) + FIR_DEQUEUE_MAX_LEN * 2 - 1] = {0.0f};

// x8 R
static arm_fir_interpolate_instance_f32 fir_inst_x8_1st_r;
static arm_fir_interpolate_instance_f32 fir_inst_x8_2nd_r;
static float fir_state_x8_1st_r[(FIR_X8_1ST_TAP_NUM / 2) + FIR_DEQUEUE_MAX_LEN - 1]     = {0.0f};
static float fir_state_x8_2nd_r[(FIR_X8_2ND_TAP_NUM / 4) + FIR_DEQUEUE_MAX_LEN * 2 - 1] = {0.0f};

// x4 L
static arm_fir_interpolate_instance_f32 fir_inst_x4_1st_l;
static arm_fir_interpolate_instance_f32 fir_inst_x4_2nd_l;
static float fir_state_x4_1st_l[(FIR_X4_1ST_TAP_NUM / 2) + FIR_DEQUEUE_MAX_LEN - 1]     = {0.0f};
static float fir_state_x4_2nd_l[(FIR_X4_2ND_TAP_NUM / 2) + FIR_DEQUEUE_MAX_LEN * 2 - 1] = {0.0f};

// x4 R
static arm_fir_interpolate_instance_f32 fir_inst_x4_1st_r;
static arm_fir_interpolate_instance_f32 fir_inst_x4_2nd_r;
static float fir_state_x4_1st_r[(FIR_X4_1ST_TAP_NUM / 2) + FIR_DEQUEUE_MAX_LEN - 1]     = {0.0f};
static float fir_state_x4_2nd_r[(FIR_X4_2ND_TAP_NUM / 2) + FIR_DEQUEUE_MAX_LEN * 2 - 1] = {0.0f};

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

static int volume_to_gain(int16_t v){
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

  // firフィルタ初期化
  arm_fir_interpolate_init_f32(&fir_inst_x8_1st_l, 2, FIR_X8_1ST_TAP_NUM, fir_x8_1st_tap, fir_state_x8_1st_l, FIR_DEQUEUE_MAX_LEN);
  arm_fir_interpolate_init_f32(&fir_inst_x8_2nd_l, 4, FIR_X8_2ND_TAP_NUM, fir_x8_2nd_tap, fir_state_x8_2nd_l, FIR_DEQUEUE_MAX_LEN * 2);

  arm_fir_interpolate_init_f32(&fir_inst_x8_1st_r, 2, FIR_X8_1ST_TAP_NUM, fir_x8_1st_tap, fir_state_x8_1st_r, FIR_DEQUEUE_MAX_LEN);
  arm_fir_interpolate_init_f32(&fir_inst_x8_2nd_r, 4, FIR_X8_2ND_TAP_NUM, fir_x8_2nd_tap, fir_state_x8_2nd_r, FIR_DEQUEUE_MAX_LEN * 2);

  arm_fir_interpolate_init_f32(&fir_inst_x4_1st_l, 2, FIR_X4_1ST_TAP_NUM, fir_x4_1st_tap, fir_state_x4_1st_l, FIR_DEQUEUE_MAX_LEN);
  arm_fir_interpolate_init_f32(&fir_inst_x4_2nd_l, 2, FIR_X4_2ND_TAP_NUM, fir_x4_2nd_tap, fir_state_x4_2nd_l, FIR_DEQUEUE_MAX_LEN * 2);

  arm_fir_interpolate_init_f32(&fir_inst_x4_1st_r, 2, FIR_X4_1ST_TAP_NUM, fir_x4_1st_tap, fir_state_x4_1st_r, FIR_DEQUEUE_MAX_LEN);
  arm_fir_interpolate_init_f32(&fir_inst_x4_2nd_r, 2, FIR_X4_2ND_TAP_NUM, fir_x4_2nd_tap, fir_state_x4_2nd_r, FIR_DEQUEUE_MAX_LEN * 2);

  while (1){
    buf_length = i2s_get_queue_length();
    int input_sample_rate_hz = dsp_get_sample_rate_hz();
    // 0.5ms分ずつi2sに送る
    dequeue_len = input_sample_rate_hz / 2000;
    if (dequeue_len > FIR_DEQUEUE_MAX_LEN) {
      dequeue_len = FIR_DEQUEUE_MAX_LEN;
    }

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
    arm_q31_to_float(i2s_buf_l, fir_temp_a_l, sample);
    arm_q31_to_float(i2s_buf_r, fir_temp_a_r, sample);

    if (input_sample_rate_hz <= 48000){
      // ゲイン補正・音量処理
      arm_scale_f32(fir_temp_a_l, 8.0 * gain_l, fir_temp_b_l, sample);
      arm_scale_f32(fir_temp_a_r, 8.0 * gain_r, fir_temp_b_r, sample);

      // インターポレーション
      arm_fir_interpolate_f32(&fir_inst_x8_1st_l, fir_temp_b_l, fir_temp_a_l, sample);
      arm_fir_interpolate_f32(&fir_inst_x8_1st_r, fir_temp_b_r, fir_temp_a_r, sample);
      sample *= 2;
      arm_fir_interpolate_f32(&fir_inst_x8_2nd_l, fir_temp_a_l, fir_temp_b_l, sample);
      arm_fir_interpolate_f32(&fir_inst_x8_2nd_r, fir_temp_a_r, fir_temp_b_r, sample);
      sample *= 4;

      // floatをint32_tに変換
      arm_float_to_q31(fir_temp_b_l, i2s_buf_l, sample);
      arm_float_to_q31(fir_temp_b_r, i2s_buf_r, sample);
    }
    else{
      // ゲイン補正・音量処理
      arm_scale_f32(fir_temp_a_l, 4.0 * gain_l, fir_temp_b_l, sample);
      arm_scale_f32(fir_temp_a_r, 4.0 * gain_r, fir_temp_b_r, sample);

      // インターポレーション
      arm_fir_interpolate_f32(&fir_inst_x4_1st_l, fir_temp_b_l, fir_temp_a_l, sample);
      arm_fir_interpolate_f32(&fir_inst_x4_1st_r, fir_temp_b_r, fir_temp_a_r, sample);
      sample *= 2;
      arm_fir_interpolate_f32(&fir_inst_x4_2nd_l, fir_temp_a_l, fir_temp_b_l, sample);
      arm_fir_interpolate_f32(&fir_inst_x4_2nd_r, fir_temp_a_r, fir_temp_b_r, sample);
      sample *= 2;

      // floatをint32_tに変換
      arm_float_to_q31(fir_temp_b_l, i2s_buf_l, sample);
      arm_float_to_q31(fir_temp_b_r, i2s_buf_r, sample);
    }

    // pio送信形式に変換
    dma_sample = i2s_format_piodata(i2s_buf_l, i2s_buf_r, sample, dma_buf_a[dma_use], dma_buf_b[dma_use]);

    // dmaが終わるまで待機
    i2s_dma_transfer_blocking(dma_buf_a[dma_use], dma_buf_b[dma_use], dma_sample);
    dma_use ^= 1;
  }
}
