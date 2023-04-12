/*
 *組込み用FIFOライブラリ 
 *石倉万希斗
 *
 *Ver 2.2.0
 */


/*
 * * 導入方法 *
 * 1."fifo.c"をコンパイル対象のファイルに入れる
 * 2."fifo.h"(このファイル)の FIFO_SIZE を任意に設定する(必ず2のべき乗数にしてください (2,4,8,16...)
 * 3."fifo.h"(このファイル)の fifo_type を任意に設定する
 * 
 * * 使用方法 *
 * 1. 初期化
 *   FIFO	hogehoge;	//FIFO構造体生成
 *   initializeFIFO( &hogehoge );
 *
 * ================================================================
 * データ追加(1個)
 *    addFIFO( &hogehoge, data );
 *    返り値
 *   	ADD_SUCCESS		: 追加成功
 *		FIFO_OVERFLOWED : FIFOがいっぱい(失敗)
 *
 * ================================================================
 * データ取得(1個)
 *    getFIFO( &hogehoge, &buffer);
 *    返り値
 *   	GET_SUCCESS	: 追加成功
 *		FIFO_EMPTY	: FIFOがからっぽ(失敗)
 *
 * ================================================================
 * FIFO内の全ての全てのデータを取得する
 *	ErrGetAllFIFO	getAllFIFO(
 *		 FIFO *p_fifo,					//FIFO構造体アドレス
 *		 fifo_type *buf,				//データ保存先配列のアドレス
 *		 unsigned short buf_size,		//データ保存先配列の最大要素数
 *		 unsigned short *num_of_data	//FIFOから取得したデータ数
 *		 );
 *	戻り値
 *		SUCCESS_GET_ALL_FIFO		: データの取得に成功
 *		BUFFER_SIZE_IS_NOT_ENOUGH	: データ保存先配列の要素数がFIFO内のデータ数より小さい
 *
 * ================================================================
 * FIFO内のデータを全て削除する
 * void	clearFIFO(
 *		FIFO	*p_fifo	//FIFO構造体アドレス
 *		);
 * ================================================================
 * FIFO内のデータ数を取得する
 * unsigned short getFillFIFO(
 *		FIFO	*p_fifo	//FIFO構造体アドレス
 *		);
 *	戻り値
 *		入っているデータ数
  * ================================================================
 * FIFOが空かどうかを返す
 * bool isEmptyFIFO(
 *		FIFO	*p_fifo	//FIFO構造体アドレス
 *		);
 *	戻り値
 *		空ならtrue, それ以外ならfalse
 * ================================================================
 */

#ifndef _FIFO_H_
#define _FIFO_H_

#include	<stdbool.h>

/*  ---- ユーザ変更部ここから ----- */
#define	FIFO_SIZE	16  /*必ず2のべき乗に数にすること*/
typedef	unsigned char	fifo_type; /* FIFOに格納するデータ型 */ 
/*  ---- ユーザ変更部ここまで ----- */



/* FIFO構造体 */
typedef struct {
	fifo_type	data[FIFO_SIZE];
	unsigned char	head_write, head_read;
}FIFO;


/* 関数プロトタイプ */
void	initializeFIFO( FIFO *p_fifo );

typedef enum ErrAddFIFO {  /* addFIFO()の戻り値 */ 
	ADD_SUCCESS,
	FIFO_OVERFLOWED
} ErrAddFIFO;
ErrAddFIFO	addFIFO( FIFO *p_fifo, fifo_type data );



typedef enum ErrGetFIFO { /* getFIFO()の戻り値 */ 
	GET_SUCCESS,
	FIFO_EMPTY
} ErrGetFIFO;
ErrGetFIFO	getFIFO( FIFO *p_fifo, fifo_type *buf );


typedef	enum ErrGetAllFIFO{ /*getAllFIFO()の戻り値 */
	SUCCESS_GET_ALL_FIFO,
	BUFFER_SIZE_IS_NOT_ENOUGH
} ErrGetAllFIFO;
ErrGetAllFIFO	getAllFIFO( FIFO *p_fifo, fifo_type *buf, unsigned short buf_size, unsigned short *num_of_data );
	
void	clearFIFO( FIFO *p_fifo );

unsigned short	getFillFIFO(FIFO *p_fifo);
bool	isEmptyFIFO(FIFO *p_fifo);
	
	

/* ここから下は使ってない関数 */
//unsigned int fifo_how_fill(fifo_t *inst);
//unsigned int fifo_how_empty(fifo_t *inst);
//int addBlockFIFO(fifo_t *inst, fifo_data_t *ptr, unsigned int size);
//int getBlockFIFO(fifo_t *inst, fifo_data_t *ptr, unsigned int size);

#endif /* _FIFO_H_ */

