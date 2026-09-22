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

#include <stdio.h>
#include <stdatomic.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/dma.h"
#include "arm_math.h"

#include "i2s.h"
#include "dsp.h"

static int doorbell_dsp;

// 48*8 Lch
static arm_fir_interpolate_instance_f32 fir_inst_l_stage1;
static arm_fir_interpolate_instance_f32 fir_inst_l_stage2;
static float32_t fir_state_l_stage1[(FIR_1ST_140DB_TAPS / 2) + FIR_1ST_BLOCK_SIZE - 1] = {0.0f};
static float32_t fir_state_l_stage2[(FIR_2ND_140DB_TAPS / 4) + FIR_2ND_BLOCK_SIZE - 1] = {0.0f};

// 48*8 Rch
static arm_fir_interpolate_instance_f32 fir_inst_r_stage1;
static arm_fir_interpolate_instance_f32 fir_inst_r_stage2;
static float32_t fir_state_r_stage1[(FIR_1ST_140DB_TAPS / 2) + FIR_1ST_BLOCK_SIZE - 1] = {0.0f};
static float32_t fir_state_r_stage2[(FIR_2ND_140DB_TAPS / 4) + FIR_2ND_BLOCK_SIZE - 1] = {0.0f};

// 96*4 Lch
static arm_fir_interpolate_instance_f32 fir_inst_l_stage1_96k;
static arm_fir_interpolate_instance_f32 fir_inst_l_stage2_96k;
static float32_t fir_state_l_stage1_96k[(FIR_1ST_140DB_96_TAPS / 2) + FIR_1ST_96_BLOCK_SIZE - 1] = {0.0f};
static float32_t fir_state_l_stage2_96k[(FIR_2ND_140DB_96_TAPS / 2) + FIR_2ND_96_BLOCK_SIZE - 1] = {0.0f};

// 96*4 Rch
static arm_fir_interpolate_instance_f32 fir_inst_r_stage1_96k;
static arm_fir_interpolate_instance_f32 fir_inst_r_stage2_96k;
static float32_t fir_state_r_stage1_96k[(FIR_1ST_140DB_96_TAPS / 2) + FIR_1ST_96_BLOCK_SIZE - 1] = {0.0f};
static float32_t fir_state_r_stage2_96k[(FIR_2ND_140DB_96_TAPS / 2) + FIR_2ND_96_BLOCK_SIZE - 1] = {0.0f};

// core間通信用変数
static q31_t fir_out_buf_q31_r[FIR_DEQUEUE_MAX_LEN * 8];
static float32_t fir_buf_float_r_process[FIR_DEQUEUE_MAX_LEN * 8];
static atomic_int shared_sample;
static atomic_uint shared_freq;
static float gain_r;

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

static int volume_to_gain(int16_t v){
    int16_t vol_index;
    vol_index = -v >> 8;
    if (vol_index > 100) vol_index = 100;
    else if (vol_index < 0) vol_index = 0;
    return gain_table[vol_index];
}

void dsp_init(void){
    // デバッグLED init
    // gpio_init(14);
    // gpio_set_dir(14, GPIO_OUT);
    // gpio_init(15);
    // gpio_set_dir(15, GPIO_OUT);

    // doorbell init
    doorbell_dsp = multicore_doorbell_claim_unused((1 << NUM_CORES) - 1, true);
    multicore_doorbell_clear_current_core(doorbell_dsp);

    // 48*8 Lch init
    arm_fir_interpolate_init_f32(&fir_inst_l_stage1, 2, FIR_1ST_140DB_TAPS, fir_1st_140db, fir_state_l_stage1, FIR_1ST_BLOCK_SIZE);
    arm_fir_interpolate_init_f32(&fir_inst_l_stage2, 4, FIR_2ND_140DB_TAPS, fir_2nd_140db, fir_state_l_stage2, FIR_2ND_BLOCK_SIZE);

    // 48*8 Rch init
    arm_fir_interpolate_init_f32(&fir_inst_r_stage1, 2, FIR_1ST_140DB_TAPS, fir_1st_140db, fir_state_r_stage1, FIR_1ST_BLOCK_SIZE);
    arm_fir_interpolate_init_f32(&fir_inst_r_stage2, 4, FIR_2ND_140DB_TAPS, fir_2nd_140db, fir_state_r_stage2, FIR_2ND_BLOCK_SIZE);

    // 96*4 Lch init
    arm_fir_interpolate_init_f32(&fir_inst_l_stage1_96k, 2, FIR_1ST_140DB_96_TAPS, fir_1st_140db_96, fir_state_l_stage1_96k, FIR_1ST_96_BLOCK_SIZE);
    arm_fir_interpolate_init_f32(&fir_inst_l_stage2_96k, 2, FIR_2ND_140DB_96_TAPS, fir_2nd_140db_96, fir_state_l_stage2_96k, FIR_2ND_96_BLOCK_SIZE);

    // 96*4 Rch init
    arm_fir_interpolate_init_f32(&fir_inst_r_stage1_96k, 2, FIR_1ST_140DB_96_TAPS, fir_1st_140db_96, fir_state_r_stage1_96k, FIR_1ST_96_BLOCK_SIZE);
    arm_fir_interpolate_init_f32(&fir_inst_r_stage2_96k, 2, FIR_2ND_140DB_96_TAPS, fir_2nd_140db_96, fir_state_r_stage2_96k, FIR_2ND_96_BLOCK_SIZE);
}

void __not_in_flash_func(dsp_core0_task)(void){
    if (multicore_doorbell_is_set_current_core(doorbell_dsp)){
        // gpio_put(14, 1);

        uint32_t freq = atomic_load(&shared_freq);
        int sample = atomic_load(&shared_sample);
        static float32_t fir_buf_float_r_temp[FIR_DEQUEUE_MAX_LEN * 8];

        if (freq <= 48000){
            // 補完処理のゲイン補正
            arm_scale_f32(fir_buf_float_r_process, 8.0 * gain_r, fir_buf_float_r_process, sample);

            arm_fir_interpolate_f32(&fir_inst_r_stage1, fir_buf_float_r_process, fir_buf_float_r_temp, sample);
            sample *= 2;
            arm_fir_interpolate_f32(&fir_inst_r_stage2, fir_buf_float_r_temp, fir_buf_float_r_process, sample);
            sample *= 4;
        }
        else{
            // 補完処理のゲイン補正
            arm_scale_f32(fir_buf_float_r_process, 4.0 * gain_r, fir_buf_float_r_process, sample);

            arm_fir_interpolate_f32(&fir_inst_r_stage1_96k, fir_buf_float_r_process, fir_buf_float_r_temp, sample);
            sample *= 2;
            arm_fir_interpolate_f32(&fir_inst_r_stage2_96k, fir_buf_float_r_temp, fir_buf_float_r_process, sample);
            sample *= 2;
        }
        arm_float_to_q31(fir_buf_float_r_process, fir_out_buf_q31_r, sample);
        __dmb();

        multicore_doorbell_clear_current_core(doorbell_dsp);
        // gpio_put(14, 0);
    }
}

void __not_in_flash_func(dsp_core1_main)(void){
    int dma_sample;
    bool mute = false;
    int buf_length;
    static int32_t dma_buf_a[2][FIR_DEQUEUE_MAX_LEN * 2 * 8], dma_buf_b[2][FIR_DEQUEUE_MAX_LEN * 2 * 8];
    uint8_t dma_use = 0;
    int dequeue_len;
    uint32_t freq;
    float gain_l;

    int sample;
    static int32_t i2s_buf_l[FIR_DEQUEUE_MAX_LEN], i2s_buf_r[FIR_DEQUEUE_MAX_LEN];

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    while (1){
        static float32_t fir_buf_float_l_process[FIR_DEQUEUE_MAX_LEN * 8];
        static float32_t fir_buf_float_l_temp[FIR_DEQUEUE_MAX_LEN * 8];
        static q31_t fir_out_buf_q31_l[FIR_DEQUEUE_MAX_LEN * 8];

        // gpio_put(15, 1);

        buf_length = i2s_get_queue_length();
        freq = dsp_get_freq();
        dequeue_len = freq / 2000;
        if (dequeue_len > FIR_DEQUEUE_MAX_LEN) {
            dequeue_len = FIR_DEQUEUE_MAX_LEN;
        }
        // printf("%3d\n", buf_length);

        if (buf_length == 0 && mute == false){
            mute = true;
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
        }
        else if (buf_length >= (dequeue_len * 3) && mute == true){
            mute = false;
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
        }

        if (mute == false){
            sample = i2s_dequeue(i2s_buf_l, i2s_buf_r, dequeue_len);
            if (sample < dequeue_len){
                for (int i = sample; i < dequeue_len; i++){
                    i2s_buf_l[i] = 0;
                    i2s_buf_r[i] = 0;
                }
                mute = true;
                gpio_put(PICO_DEFAULT_LED_PIN, 0);
            }
        }
        else{
            for (int i = 0; i < dequeue_len; i++){
                i2s_buf_l[i] = 0;
                i2s_buf_r[i] = 0;
            }
        }
        sample = dequeue_len;

        // 音量設定L
        int16_t v;
        v = i2s_get_volume_l();
        gain_l = volume_to_gain(v);

        // 音量設定R
        v = i2s_get_volume_r();
        gain_r = volume_to_gain(v);

        // int32_tをfloat32_tに変換
        arm_q31_to_float(i2s_buf_l, fir_buf_float_l_process, sample);
        arm_q31_to_float(i2s_buf_r, fir_buf_float_r_process, sample);
        __dmb();

        // core0_task開始
        atomic_store(&shared_freq, freq);
        atomic_store(&shared_sample, dequeue_len);
        multicore_doorbell_set_other_core(doorbell_dsp);

        if (freq <= 48000){
            // 補完処理のゲイン補正
            arm_scale_f32(fir_buf_float_l_process, 8.0 * gain_l, fir_buf_float_l_process, sample);

            arm_fir_interpolate_f32(&fir_inst_l_stage1, fir_buf_float_l_process, fir_buf_float_l_temp, sample);
            sample *= 2;
            arm_fir_interpolate_f32(&fir_inst_l_stage2, fir_buf_float_l_temp, fir_buf_float_l_process, sample);
            sample *= 4;
        }
        else{
            // 補完処理のゲイン補正
            arm_scale_f32(fir_buf_float_l_process, 4.0 * gain_l, fir_buf_float_l_process, sample);

            arm_fir_interpolate_f32(&fir_inst_l_stage1_96k, fir_buf_float_l_process, fir_buf_float_l_temp, sample);
            sample *= 2;
            arm_fir_interpolate_f32(&fir_inst_l_stage2_96k, fir_buf_float_l_temp, fir_buf_float_l_process, sample);
            sample *= 2;
        }
        arm_float_to_q31(fir_buf_float_l_process, fir_out_buf_q31_l, sample);

        // core0_taskが終わるまで待機
        while(multicore_doorbell_is_set_other_core(doorbell_dsp)){
            tight_loop_contents();
        }

        // i2sバッファに格納
        dma_sample = i2s_format_piodata(fir_out_buf_q31_l, fir_out_buf_q31_r, sample, dma_buf_a[dma_use], dma_buf_b[dma_use]);
        // gpio_put(15, 0);

        i2s_dma_transfer_blocking(dma_buf_a[dma_use], dma_buf_b[dma_use], dma_sample);
        dma_use ^= 1;
    }
}

static atomic_uint dsp_freq = 44100;

void dsp_set_freq(uint32_t freq){
    switch(freq){
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
    atomic_store(&dsp_freq, freq);
}

uint32_t dsp_get_freq(void){
    return atomic_load(&dsp_freq);
}