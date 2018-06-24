/*
 * ************************************************
 * uart for RoboCup
 * Ver 1.3.1
 *
 * Device   : dsPIC33F(28ピン)
 *
 * Author   : 日下 諒,石倉万希斗
 * Created on 2013/01/23
 * ************************************************
 */

/*
 * 使い方
 *
 * 1.使いたいファイルで,"uart.h"をインクルードする
 * 2.プロジェクトに，"uart.c"と"fifo.c"を導入する (※ FIFOライブラリは、別途導入してください)
 * 3.initializeUart()を初期設定時に呼び出す(この関数はマクロ関数です)
 * 
 * 注意事項
 * initilizeUart関数を実行すると，受信および送信割り込みがイネーブルされます．
 */


#ifndef	_UART_H_
#define	_UART_H_

/* ---------------------------------- */

/* initializeUart( pin_num_of_tx, pin_num_of_rx, xtal_freq, baudrate ); */
/*UARTの初期設定をする関数(マクロ関数)
 * 引数
 * unsigned char pin_num_of_tx  :TXに設定するRPピンの番号
 * unsigned char pin_num_of_rx  :RXに設定するRPピンの番号
 * unsigned long baudrate       :ボーレイトの値
 * unsigned long xtal_freq      :動作周波数
 */
#define	initializeUart( pin_num_of_tx, pin_num_of_rx, xtal_freq, baudrate ) \
	initializeUartFunc( (xtal_freq), (baudrate) ); \
	TRISB	&= ~(0x0001 << (pin_num_of_tx - 32)); \
	_RP##pin_num_of_tx##R	= 0b00001; \
	TRISB	|= 0x0001 << (pin_num_of_rx - 32); \
    _U1RXR = (pin_num_of_rx)

void    initializeUartFunc(unsigned long xtal_freq,unsigned long baudrate);

void __attribute__(( interrupt, auto_psv )) _U1RXInterrupt(void);

void	putcUart( char data );
	/* 1バイトデータをUARTで送信する関数
	 * 引数
	 * char data	: 送信する1バイトデータ
	 * 返り値
	 * なし
	 */

void	putsUart( char* str );
	/* 文字列を送信する
	 * 引数
	 * char* str	: 送信する文字列
	 * 返り値
	 * なし
	 */



/*
 *ErrGetcUart	getcUart(unsigned char* data );
 *受信バッファから1バイト取り出す
 */
typedef enum ErrGetcUart {
	SUCCESS_GETC,
	BUFFER_EMPTY
} ErrGetcUart;
ErrGetcUart	getcUart(unsigned char* data );
/* ---------------------------------- */


#endif
