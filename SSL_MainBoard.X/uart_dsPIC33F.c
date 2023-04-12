/*
 * ************************************************
 * fifo for RoboCup
 *
 * Device	: dsPIC
 *
 * Author	: 日下 諒, 石倉万希斗
 *
 * Created on 2013/01/24
 * ************************************************
 */

#include <xc.h>
#include <uart.h>
#include "fifo.h"
#include "uart_dsPIC33F.h"


unsigned int UART_config1 = UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE & UART_MODE_SIMPLEX & UART_UEN_00 & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE & UART_BRGH_SIXTEEN & UART_NO_PAR_8BIT & UART_1STOPBIT;

unsigned int UART_config2 = UART_INT_TX & UART_IrDA_POL_INV_ZERO & UART_SYNC_BREAK_DISABLED & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR;

unsigned int UART_cofigINT = UART_RX_INT_EN & UART_RX_INT_PR4 & UART_TX_INT_EN & UART_TX_INT_PR4;



/* ---------------------------------- */
FIFO	fifo_transfer, fifo_receive;
/* ---------------------------------- */

void    initializeUartFunc(unsigned long xtal_freq,unsigned long baudrate)
{
	long	brg;
    unsigned int brg_int;

	initializeFIFO( &fifo_transfer );
    initializeFIFO( &fifo_receive );
/*
 *
 *     小数点以下1桁を四捨五入する処理をしてるので複雑に見えるが
 *     やってることはデータシートの式通り 
 *     brg	= ( ((xtal_freq / 2) / (16 * baudrate)) - 1 ) * 10 + 5
 *            = ( ((10 * xtal_freq / 2) / (16 * baudrate)) - 10 ) + 5 
 *            = ((5 * xtal_freq) / (16 * baudrate)) - 5
 *            = 5 * (xtal_freq / (16 * baudrate) -1)
 */

	brg	= 5 * (xtal_freq / (16 * baudrate) - 1);
	brg	/= 10;
    brg_int = brg;

    OpenUART1(UART_config1,UART_config2,brg_int);

    ConfigIntUART1(UART_cofigINT);

    return;

}
/* ---------------------------------- */
void __attribute__(( interrupt, auto_psv )) _U1RXInterrupt(void)
{
    unsigned char	buffer;

	IFS0bits.U1RXIF = 0;

    buffer	= ReadUART1();
    addFIFO( &fifo_receive, buffer );

    return;
}


void __attribute__(( interrupt, auto_psv )) _U1TXInterrupt(void)
{
	ErrGetFIFO	err;
	char	data;

	err	= getFIFO( &fifo_transfer, (fifo_type*)&data );

	_U1TXIF	= 0;
	if( err == GET_SUCCESS ){
		WriteUART1(data);
	}
}

/* ---------------------------------- */
void	putsUart( char* str )
{
    unsigned int	i = 0;

    while( *(str + i) != '\0' ){
	    putcUart( *(str + i) );
	    i ++;
    }
}


void	putcUart(char data )
{
	if( U1STAbits.UTXBF == 0 ){	//送信のハードウェアFIFOに空きがある
		WriteUART1( data );
	}else{
		while( addFIFO( &fifo_transfer, data ) == FIFO_OVERFLOWED ); /* 送信ストリームにデータをセット*/ 
	}
}
/* ---------------------------------- */


/* ---------------------------------- */
ErrGetcUart	getcUart(unsigned char* data )
{
	ErrGetFIFO	err;

    err	= getFIFO( &fifo_receive, data );

    if( err == FIFO_EMPTY ){
	    return	BUFFER_EMPTY;
    }else{
		return	SUCCESS_GETC;
    }
}
/* ---------------------------------- */
