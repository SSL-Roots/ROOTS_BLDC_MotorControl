#include	"fifo.h"

void	initializeFIFO( FIFO *p_fifo ){
	p_fifo -> head_read		= 0;
	p_fifo -> head_write	= 0;
}


ErrAddFIFO	addFIFO( FIFO *p_fifo, fifo_type data )
{
	unsigned char	next = (p_fifo -> head_write+ 1) & (FIFO_SIZE - 1);

	if( next == p_fifo->head_read ){
		return	FIFO_OVERFLOWED;
	}

	p_fifo->data[p_fifo->head_write]	= data;
	p_fifo->head_write	= next;

	return	ADD_SUCCESS;
	
}

ErrGetFIFO	getFIFO( FIFO *p_fifo, fifo_type *buf )
{
	if( p_fifo->head_read == p_fifo->head_write ){
		return	FIFO_EMPTY;
	}

	*buf	= p_fifo->data[p_fifo->head_read];
	p_fifo->head_read	= (p_fifo->head_read + 1) & (FIFO_SIZE - 1 );

	return	GET_SUCCESS;
}


ErrGetAllFIFO	getAllFIFO( FIFO *p_fifo, fifo_type *buf, unsigned short buf_size, unsigned short *num_of_data )
{
	unsigned short	i = 0;

	while( getFIFO(p_fifo, buf+i) == GET_SUCCESS ){
		i++;
		if( i >= buf_size ){
			*num_of_data	= i;
			return	BUFFER_SIZE_IS_NOT_ENOUGH;
		}
	}

	*num_of_data	= i;
	return	SUCCESS_GET_ALL_FIFO;
}


void	clearFIFO( FIFO *p_fifo )
{
	p_fifo->head_read	= p_fifo->head_write;
}


unsigned short	getFillFIFO(FIFO *p_fifo)
{
	return	(FIFO_SIZE + p_fifo->head_write - p_fifo->head_read) % FIFO_SIZE;
}


bool	isEmptyFIFO(FIFO *p_fifo)
{
	return	(p_fifo->head_write == p_fifo->head_read) ? true : false;
}

