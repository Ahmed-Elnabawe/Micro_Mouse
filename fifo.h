/*
 * fifo.h
 *
 *  Created on: Aug 16, 2024
 *      Author: Lenovo
 */

#ifndef FIFO_H_
#define FIFO_H_
#include "stdint.h"
#include "stdio.h"
#include "cells.h"
// chose the element type
#define element_type cell_t
#define fifo_size 5000

typedef struct {
	uint32_t count ;
	uint32_t length ;
	element_type * base;
	element_type * head;
	element_type * tail;
}FIFO_buffer_t;
typedef enum {
	FIFO_no_error,
	FIFO_null,
	FIFO_empty,
	FIFO_full
}FIFO_statuas_t;

// APIs
FIFO_statuas_t FIFO_enqueue (FIFO_buffer_t* buffer ,element_type item);
FIFO_statuas_t FIFO_dequeue (FIFO_buffer_t* buffer ,element_type* item);
FIFO_statuas_t FIFO_is_full (FIFO_buffer_t* buffer);
void FIFO_print (FIFO_buffer_t* buffer );
FIFO_statuas_t FIFO_is_empty (FIFO_buffer_t* buffer);
FIFO_statuas_t FIFO_init (FIFO_buffer_t* buffer ,element_type* buf,uint32_t length);

#endif /* FIFO_H_ */
