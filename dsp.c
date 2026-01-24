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
static q31_t buff_r[I2S_DEQUEUE_LEN * 8];
static float32_t dsp_buff_r_1[I2S_DEQUEUE_LEN * 8];

//48*8
static arm_fir_interpolate_instance_f32 s_r_1st;
static arm_fir_interpolate_instance_f32 s_r_2nd;
static float32_t state_r_1st[(FIR_1ST_140DB_TAPS / 2) + FIR_1ST_BLOCK_SIZE - 1] = {0.0f};
static float32_t state_r_2nd[(FIR_2ND_140DB_TAPS / 4) + FIR_2ND_BLOCK_SIZE - 1] = {0.0f};

//96*4
static arm_fir_interpolate_instance_f32 s_r_1st_96;
static arm_fir_interpolate_instance_f32 s_r_2nd_96;
static float32_t state_r_1st_96[(FIR_1ST_140DB_96_TAPS / 2) + FIR_1ST_96_BLOCK_SIZE - 1] = {0.0f};
static float32_t state_r_2nd_96[(FIR_2ND_140DB_96_TAPS / 2) + FIR_2ND_96_BLOCK_SIZE - 1] = {0.0f};

void dsp_init(void){
    //doorbell init
    doorbell_dsp = multicore_doorbell_claim_unused((1 << NUM_CORES) - 1, true);
    multicore_doorbell_clear_current_core(doorbell_dsp);

    //48*8
    arm_fir_interpolate_init_f32(&s_r_1st, 2, FIR_1ST_140DB_TAPS, fir_1st_140db, state_r_1st, FIR_1ST_BLOCK_SIZE);
    arm_fir_interpolate_init_f32(&s_r_2nd, 4, FIR_2ND_140DB_TAPS, fir_2nd_140db, state_r_2nd, FIR_2ND_BLOCK_SIZE);

    //96*4
    arm_fir_interpolate_init_f32(&s_r_1st_96, 2, FIR_1ST_140DB_96_TAPS, fir_1st_140db_96, state_r_1st_96, FIR_1ST_96_BLOCK_SIZE);
    arm_fir_interpolate_init_f32(&s_r_2nd_96, 2, FIR_2ND_140DB_96_TAPS, fir_2nd_140db_96, state_r_2nd_96, FIR_2ND_96_BLOCK_SIZE);
}

void __not_in_flash_func(dsp_core0_task)(void){
    if (multicore_doorbell_is_set_current_core(doorbell_dsp)){
        uint32_t freq = dsp_get_freq();
        int sample = I2S_DEQUEUE_LEN;
        float32_t dsp_buff_r_2[I2S_DEQUEUE_LEN * 8];
        #if 1
        if (freq <= 48000){
            arm_scale_f32(dsp_buff_r_1, 8.0, dsp_buff_r_1, sample);

            arm_fir_interpolate_f32(&s_r_1st, dsp_buff_r_1, dsp_buff_r_2, sample);
            sample *= 2;
            arm_fir_interpolate_f32(&s_r_2nd, dsp_buff_r_2, dsp_buff_r_1, sample);
            sample *= 4;
        }
        else{
            arm_scale_f32(dsp_buff_r_1, 4.0, dsp_buff_r_1, sample);

            arm_fir_interpolate_f32(&s_r_1st_96, dsp_buff_r_1, dsp_buff_r_2, sample);
            sample *= 2;
            arm_fir_interpolate_f32(&s_r_2nd_96, dsp_buff_r_2, dsp_buff_r_1, sample);
            sample *= 2;
        }
        arm_float_to_q31(dsp_buff_r_1, buff_r, sample);
        #else
        if (freq <= 48000){
            for (int i = 0, j = 0; i < sample; i++){
                dsp_buff_r_2[j++] = dsp_buff_r_1[i];
                dsp_buff_r_2[j++] = dsp_buff_r_1[i];
                dsp_buff_r_2[j++] = dsp_buff_r_1[i];
                dsp_buff_r_2[j++] = dsp_buff_r_1[i];
                dsp_buff_r_2[j++] = dsp_buff_r_1[i];
                dsp_buff_r_2[j++] = dsp_buff_r_1[i];
                dsp_buff_r_2[j++] = dsp_buff_r_1[i];
                dsp_buff_r_2[j++] = dsp_buff_r_1[i];
            }
            sample *= 8;
        }
        else {
            for (int i = 0, j = 0; i < sample; i++){
                dsp_buff_r_2[j++] = dsp_buff_r_1[i];
                dsp_buff_r_2[j++] = dsp_buff_r_1[i];
                dsp_buff_r_2[j++] = dsp_buff_r_1[i];
                dsp_buff_r_2[j++] = dsp_buff_r_1[i];
            }
            sample *= 4;
        }
        arm_float_to_q31(dsp_buff_r_2, buff_r, sample);
        #endif
        multicore_doorbell_clear_current_core(doorbell_dsp);
    }
}

void __not_in_flash_func(dsp_core1_main)(void){
    int dma_sample[2];
    bool mute = false;
    int buf_length;
    static int32_t dma_buf_a[2][I2S_DEQUEUE_LEN * 2 * 8], dma_buf_b[2][I2S_DEQUEUE_LEN * 2 * 8];
    uint8_t dma_use = 0;
    
    int sample;
    int32_t buf_l[I2S_DEQUEUE_LEN], buf_r[I2S_DEQUEUE_LEN];

    uint32_t freq;

    //48*8
    arm_fir_interpolate_instance_f32 s_l_1st;
    arm_fir_interpolate_instance_f32 s_l_2nd;
    float32_t state_l_1st[(FIR_1ST_140DB_TAPS / 2) + FIR_1ST_BLOCK_SIZE - 1] = {0.0f};
    float32_t state_l_2nd[(FIR_2ND_140DB_TAPS / 4) + FIR_2ND_BLOCK_SIZE - 1] = {0.0f};
    arm_fir_interpolate_init_f32(&s_l_1st, 2, FIR_1ST_140DB_TAPS, fir_1st_140db, state_l_1st, FIR_1ST_BLOCK_SIZE);
    arm_fir_interpolate_init_f32(&s_l_2nd, 4, FIR_2ND_140DB_TAPS, fir_2nd_140db, state_l_2nd, FIR_2ND_BLOCK_SIZE);

    //96*4
    arm_fir_interpolate_instance_f32 s_l_1st_96;
    arm_fir_interpolate_instance_f32 s_l_2nd_96;
    float32_t state_l_1st_96[(FIR_1ST_140DB_96_TAPS / 2) + FIR_1ST_96_BLOCK_SIZE - 1] = {0.0f};
    float32_t state_l_2nd_96[(FIR_2ND_140DB_96_TAPS / 2) + FIR_2ND_96_BLOCK_SIZE - 1] = {0.0f};
    arm_fir_interpolate_init_f32(&s_l_1st_96, 2, FIR_1ST_140DB_96_TAPS, fir_1st_140db_96, state_l_1st_96, FIR_1ST_96_BLOCK_SIZE);
    arm_fir_interpolate_init_f32(&s_l_2nd_96, 2, FIR_2ND_140DB_96_TAPS, fir_2nd_140db_96, state_l_2nd_96, FIR_2ND_96_BLOCK_SIZE);

    // gpio_init(15);
    // gpio_set_dir(15, GPIO_OUT);

    // int words_per_frame = 2;
    // if (i2s_mode == MODE_PT8211_DUAL || i2s_mode == MODE_I2S_DUAL) {
    //     words_per_frame = 4;
    // }

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    while (1){
        static float32_t dsp_buff_l_1[I2S_DEQUEUE_LEN * 8];
        static float32_t dsp_buff_l_2[I2S_DEQUEUE_LEN * 8];
        static q31_t buff_l[I2S_DEQUEUE_LEN * 8];

        // gpio_put(15, 1);

        buf_length = i2s_get_queue_length();
        // printf("%3d\n", buf_length);
        // if (buf_length > I2S_DEQUEUE_LEN * 2 && buf_length < I2S_DEQUEUE_LEN * 7){
        //     gpio_put(15, 1);
        // }
        // else{
        //     gpio_put(15, 0);
        // }

        if (buf_length == 0 && mute == false){
            mute = true;
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
        }
        else if (buf_length >= (I2S_DEQUEUE_LEN * 2) && mute == true){
            mute = false;
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
        }

        if (mute == false){
            sample = i2s_dequeue(buf_l, buf_r, I2S_DEQUEUE_LEN);
            if (sample < I2S_DEQUEUE_LEN){
                for (int i = sample; i < I2S_DEQUEUE_LEN; i++){
                    buf_l[i] = 0;
                    buf_r[i] = 0;
                }
                mute = true;
                gpio_put(PICO_DEFAULT_LED_PIN, 0);
            }
        }
        else{
            for (int i = 0; i < I2S_DEQUEUE_LEN; i++){
                buf_l[i] = 0;
                buf_r[i] = 0;
            }
        }
        sample = I2S_DEQUEUE_LEN;

        //int32_tをfloat32_tに変換
        arm_q31_to_float(buf_l, dsp_buff_l_1, sample);
        arm_q31_to_float(buf_r, dsp_buff_r_1, sample);
        freq = dsp_get_freq();

        //core0_task開始
        multicore_doorbell_set_other_core(doorbell_dsp);

        if (freq <= 48000){
            arm_scale_f32(dsp_buff_l_1, 8.0, dsp_buff_l_1, sample);

            arm_fir_interpolate_f32(&s_l_1st, dsp_buff_l_1, dsp_buff_l_2, sample);
            sample *= 2;
            arm_fir_interpolate_f32(&s_l_2nd, dsp_buff_l_2, dsp_buff_l_1, sample);
            sample *= 4;
        }
        else{
            arm_scale_f32(dsp_buff_l_1, 4.0, dsp_buff_l_1, sample);

            arm_fir_interpolate_f32(&s_l_1st_96, dsp_buff_l_1, dsp_buff_l_2, sample);
            sample *= 2;
            arm_fir_interpolate_f32(&s_l_2nd_96, dsp_buff_l_2, dsp_buff_l_1, sample);
            sample *= 2;
        }
        arm_float_to_q31(dsp_buff_l_1, buff_l, sample);

        //core0_taskが終わるまで待機
        while(multicore_doorbell_is_set_other_core(doorbell_dsp)){
            tight_loop_contents();
        }

        //i2sバッファに格納
        dma_sample[dma_use] = i2s_format_piodata(buff_l, buff_r, sample, dma_buf_a[dma_use], dma_buf_b[dma_use]);
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