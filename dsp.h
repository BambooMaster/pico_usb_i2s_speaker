#ifndef DSP_H
#define DSP_H

#define FIR_DEQUEUE_MAX_LEN   (96 / 2)

void dsp_set_sample_rate_hz(uint32_t sample_rate_hz);

uint32_t dsp_get_sample_rate_hz(void);

void core1_main_dsp(void);

#endif
