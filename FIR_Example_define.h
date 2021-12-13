/*
 * FIRFilter.h
 *
 *  Created on: 2021/11/05
 *      Author: johnson
 */

#ifndef FIRFILTER_H_
#define FIRFILTER_H_
#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint16_t *buffer;
    uint16_t head;
    uint16_t tail;
    uint16_t max;
    bool full;
} circular_buf_t;

typedef circular_buf_t *circular_handle_t;
/*

 FIR filter designed with
 http://t-filter.appspot.com

 sampling frequency: 10000 Hz

 * 0 Hz - 200 Hz
 gain = 1
 desired ripple = 0.1 dB
 actual ripple = 0.060686422707968264 dB

 * 500 Hz - 5000 Hz
 gain = 0
 desired attenuation = -40 dB
 actual attenuation = -41.69131531643887 dB

 */

#define FILTER_ORDER_NUM 32

static double FIRfilter[FILTER_ORDER_NUM] =
{
 0.003319179996254201,
 -0.0026148899915180547,
 -0.005023674117563195,
 -0.008347712523873929,
 -0.01153725527362621,
 -0.013292802258598593,
 -0.012194067673091944,
 -0.0069520702402888525,
 0.00327815259040964,
 0.01862562106778208,
 0.03832759472889796,
 0.06073090875605592,
 0.08346968558720541,
 0.1038338423147242,
 0.11917576731162036,
 0.12742272672867183,
 0.12742272672867183,
 0.11917576731162036,
 0.1038338423147242,
 0.08346968558720541,
 0.06073090875605592,
 0.03832759472889796,
 0.01862562106778208,
 0.00327815259040964,
 -0.0069520702402888525,
 -0.012194067673091944,
 -0.013292802258598593,
 -0.01153725527362621,
 -0.008347712523873929,
 -0.005023674117563195,
 -0.0026148899915180547,
 0.003319179996254201,
};

// Initial circular buffer
// @param cbuf circular buffer pointer
// @param size buffer size
circular_handle_t circular_buf_init(uint16_t *buffer, uint16_t size);

// Put in ADC value into circular buffer
// @param cbuf circular buffer pointer
// @param sample ADC read value
void circular_buf_put(circular_handle_t cbuf, uint16_t sample);

// Calculate filter output
// @param cbuf
// @param filter
// @return output
double filter_process(circular_handle_t cbuf, double *filter);

#endif /* FIRFILTER_H_ */
