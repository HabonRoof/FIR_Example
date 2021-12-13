/*
 * FIRFilter.c
 *
 *  Created on:  2021/11/05
 *      Author: johnson
 */
 #include "FIRFilter.h"
#include <stdlib.h>
// Check if circular buffer is empty
// @param cbuf
// @return true if cbuf is empty, false if not empty
bool circular_buf_empty(circular_handle_t cbuf){

    return(!cbuf->full && (cbuf->head == cbuf->tail));
};

// Check if circular buffer is full
// @param cbuf
// @return bool true if circular buffer is full, false if not full
bool circular_buf_full(circular_handle_t cbuf){

    return cbuf->full;
};

// Initial circular buffer
// @param cbuf circular buffer pointer
// @param size buffer size
circular_handle_t circular_buf_init(uint16_t *buffer, uint16_t size){

    circular_handle_t cbuf = malloc(sizeof(circular_buf_t));

    cbuf->buffer = buffer;
    cbuf->max = size;
    cbuf->head = 0;
    cbuf->tail = 0;
    cbuf->full = false;


    return cbuf;
};

// Increase circulae buffer pointer position
// @param cbuf
void increase_pointer(circular_handle_t cbuf){


    // Increade head pointer
    // If reach max capacity, reset to zero
    if(++(cbuf->head) == cbuf->max)
        cbuf->head = 0;

    // Increase tail pointer if buffer is full
    // If reach max capacity, reset to zero
    if(cbuf->full){
        if(++(cbuf->tail) == cbuf-> max)
            cbuf->tail = 0;
    };
    // If head == tail means buffer is full
    // set the flag to true
    cbuf->full = (cbuf->head == cbuf->tail);
};

// Put data into circular buffer
// @param cbuf circular buffer pointer
// @param data ADC read data
void circular_buf_put(circular_handle_t cbuf, uint16_t data){

    cbuf->buffer[cbuf->head] = data;
    increase_pointer(cbuf);     // Increase pointer when new data update
};

// Digital filter process
// @param cbuf
// @param *filter
// @return float output of filter
double filter_process(circular_handle_t cbuf, double *filter){
    int filterOrder = 0;
    int bufferIndex = cbuf->head;   // Current buffer head index
    double output = 0.0f;

    for(filterOrder = 0; filterOrder < cbuf->max; filterOrder++){
        // Calculate the FIR filter order coefficient and buffer value
        output = output + cbuf->buffer[bufferIndex] * filter[filterOrder];

        if(bufferIndex == 0)    // The coefficient index increase, buffer index decrease
            bufferIndex = cbuf->max - 1;
        else
            bufferIndex--;
    }
    return output;
};


