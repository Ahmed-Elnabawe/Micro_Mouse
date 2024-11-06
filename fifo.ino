/*
 * fifo.c
 *
 *  Created on: Aug 16, 2024
 *      Author: Lenovo
 */


#include "fifo.h"
FIFO_statuas_t FIFO_enqueue (FIFO_buffer_t* buffer ,element_type item)
{
	// check if the buffer is valid
	if(!buffer->base || !buffer->head || !buffer->tail)
		return FIFO_null;
	// check if the buffer is full
	if(FIFO_is_full(buffer)== FIFO_full)
		return FIFO_full;
	// add item

	if(buffer->head==(buffer->base+(buffer->length* sizeof(element_type))))
	{
		buffer->head=buffer->base;
		*(buffer->head) = item ;
		buffer->count++;
		buffer->head++;
	}
	else
	{
		*(buffer->head) = item ;
		buffer->count++;
		buffer->head++;
	}
	return FIFO_no_error;
}
FIFO_statuas_t FIFO_dequeue (FIFO_buffer_t* buffer ,element_type* item)
{
	// check if the buffer is valid
	if(!buffer->base || !buffer->head || !buffer->tail)
		return FIFO_null;
	// check if the buffer is empty
	if(buffer->count== 0)
		return FIFO_empty;
	*item=*(buffer->tail) ;
	buffer->count--;
	if(buffer->tail==(buffer->base+(buffer->length* sizeof(element_type))))
		buffer->tail=buffer->base;
	else
		buffer->tail++;
	return FIFO_no_error;
}
FIFO_statuas_t FIFO_is_full (FIFO_buffer_t* buffer)
{
	// check if the buffer is valid
	if(!buffer->base || !buffer->head || !buffer->tail)
		return FIFO_null;
	// check if the buffer is full
	if(buffer->count == buffer->length)
		return FIFO_full;
	return FIFO_no_error;
}
void FIFO_print (FIFO_buffer_t* buffer )
{
	uint32_t i;
	element_type* temp= buffer->tail;
	// check if the buffer is empty
	if(buffer->count== 0)
	{
		printf("fifo is empty\n");
	}
	else{
		printf("============= print fifo buffef ============= \n");
		for (i = 0; i < buffer->count ; ++i) {

			if(temp==(buffer->base+(buffer->length* sizeof(element_type))))
			{
				temp=buffer->base;
				printf("\t %x \n " ,*temp);
				temp++;
			}
			else
			{
				printf("\t %x \n " ,*temp);
				temp++;
			}
		}
		printf("============= print done =============\n");
	}



}
FIFO_statuas_t FIFO_init (FIFO_buffer_t* buffer ,element_type* buf ,uint32_t length )
{
	// check if the buffer is valid
	if(buf==NULL)
		return FIFO_null;

	buffer->count=0;
	buffer->base=buf;
	buffer->head=buf;
	buffer->tail=buf;
	buffer->length=length;
	return FIFO_no_error;
}
FIFO_statuas_t FIFO_is_empty (FIFO_buffer_t* buffer){
	// check if the buffer is valid
	//if(!buffer->base || !buffer->head || !buffer->tail)
	//	return FIFO_null;
	// check if the buffer is empty
	if(buffer->count== 0)
		return FIFO_empty;
	return FIFO_no_error;
}
