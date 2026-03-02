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
static q31_t fir_out_buf_q31_r[I2S_DEQUEUE_LEN * 8];
static float32_t fir_buf_float_r_process[I2S_DEQUEUE_LEN * 8];

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

        uint32_t freq = dsp_get_freq();
        int sample = I2S_DEQUEUE_LEN;
        static float32_t fir_buf_float_r_temp[I2S_DEQUEUE_LEN * 8];

        if (freq <= 48000){
            // 補完処理のゲイン補正
            arm_scale_f32(fir_buf_float_r_process, 8.0, fir_buf_float_r_process, sample);

            arm_fir_interpolate_f32(&fir_inst_r_stage1, fir_buf_float_r_process, fir_buf_float_r_temp, sample);
            sample *= 2;
            arm_fir_interpolate_f32(&fir_inst_r_stage2, fir_buf_float_r_temp, fir_buf_float_r_process, sample);
            sample *= 4;
        }
        else{
            // 補完処理のゲイン補正
            arm_scale_f32(fir_buf_float_r_process, 4.0, fir_buf_float_r_process, sample);

            arm_fir_interpolate_f32(&fir_inst_r_stage1_96k, fir_buf_float_r_process, fir_buf_float_r_temp, sample);
            sample *= 2;
            arm_fir_interpolate_f32(&fir_inst_r_stage2_96k, fir_buf_float_r_temp, fir_buf_float_r_process, sample);
            sample *= 2;
        }
        arm_float_to_q31(fir_buf_float_r_process, fir_out_buf_q31_r, sample);

        multicore_doorbell_clear_current_core(doorbell_dsp);
        // gpio_put(14, 0);
    }
}

void __not_in_flash_func(dsp_core1_main)(void){
    int dma_sample[2];
    bool mute = false;
    int buf_length;
    static int32_t dma_buf_a[2][I2S_DEQUEUE_LEN * 2 * 8], dma_buf_b[2][I2S_DEQUEUE_LEN * 2 * 8];
    uint8_t dma_use = 0;
    uint32_t freq;

    int sample;
    static int32_t i2s_rx_buf_l[I2S_DEQUEUE_LEN], i2s_rx_buf_r[I2S_DEQUEUE_LEN];

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    while (1){
        static float32_t fir_buf_float_l_process[I2S_DEQUEUE_LEN * 8];
        static float32_t fir_buf_float_l_temp[I2S_DEQUEUE_LEN * 8];
        static q31_t fir_out_buf_q31_l[I2S_DEQUEUE_LEN * 8];

        // gpio_put(15, 1);

        buf_length = i2s_get_queue_length();
        // printf("%3d\n", buf_length);

        if (buf_length == 0 && mute == false){
            mute = true;
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
        }
        else if (buf_length >= (I2S_DEQUEUE_LEN * 2) && mute == true){
            mute = false;
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
        }

        if (mute == false){
            sample = i2s_dequeue(i2s_rx_buf_l, i2s_rx_buf_r, I2S_DEQUEUE_LEN);
            if (sample < I2S_DEQUEUE_LEN){
                for (int i = sample; i < I2S_DEQUEUE_LEN; i++){
                    i2s_rx_buf_l[i] = 0;
                    i2s_rx_buf_r[i] = 0;
                }
                mute = true;
                gpio_put(PICO_DEFAULT_LED_PIN, 0);
            }
        }
        else{
            for (int i = 0; i < I2S_DEQUEUE_LEN; i++){
                i2s_rx_buf_l[i] = 0;
                i2s_rx_buf_r[i] = 0;
            }
        }
        sample = I2S_DEQUEUE_LEN;

        // int32_tをfloat32_tに変換
        arm_q31_to_float(i2s_rx_buf_l, fir_buf_float_l_process, sample);
        arm_q31_to_float(i2s_rx_buf_r, fir_buf_float_r_process, sample);
        freq = dsp_get_freq();

        // core0_task開始
        multicore_doorbell_set_other_core(doorbell_dsp);

        if (freq <= 48000){
            // 補完処理のゲイン補正
            arm_scale_f32(fir_buf_float_l_process, 8.0, fir_buf_float_l_process, sample);

            arm_fir_interpolate_f32(&fir_inst_l_stage1, fir_buf_float_l_process, fir_buf_float_l_temp, sample);
            sample *= 2;
            arm_fir_interpolate_f32(&fir_inst_l_stage2, fir_buf_float_l_temp, fir_buf_float_l_process, sample);
            sample *= 4;
        }
        else{
            // 補完処理のゲイン補正
            arm_scale_f32(fir_buf_float_l_process, 4.0, fir_buf_float_l_process, sample);

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
        dma_sample[dma_use] = i2s_format_piodata(fir_out_buf_q31_l, fir_out_buf_q31_r, sample, dma_buf_a[dma_use], dma_buf_b[dma_use]);
        // gpio_put(15, 0);

        i2s_dma_transfer_bloking(dma_buf_a[dma_use], dma_buf_b[dma_use], dma_sample[dma_use]);
        dma_use ^= 1;
    }
}

static atomic_uint dsp_freq = 44100;

void dsp_set_freq(uint32_t freq){
    atomic_store(&dsp_freq, freq);
}

uint32_t dsp_get_freq(void){
    return atomic_load(&dsp_freq);
}