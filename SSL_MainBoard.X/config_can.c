/* 
 * File:   config_can.c
 * Author: y.kek.na0710
 *
 * Created on 2017/03/26, 21:01
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <xc.h>
#include "config_can.h"
#include "BLDC_Drive_Control.h"
#include "MotorDriverStatus.h"

#define F_RX_CAN _LATA3
/*
 * 
 */
#define NUM_OF_ECAN_BUFS 10 // バ ッ フ ァ の 数
//4*8 の2 次 元 配 列 の 型 を 用 意
//typedef unsigned int ECAN1MSGBUF[NUM_OF_ECAN_BUFS][8]; // D M A 領 域 に 配 置 し た 変 数 を 宣 言
//ECAN1MSGBUF ecan1msgBuf __attribute__( (space(dma),aligned(NUM_OF_ECAN_BUFS*16)));

//unsigned int ecan1msgBuf[NUM_OF_ECAN_BUFFERS][8]
//__attribute__((aligned(NUM_OF_ECAN_BUFFERS * 16)));

/*  関数    */
void initCANINT(void);
void initDMA(void);
void initCANFilter(void);
void exchangeOrderData(void);
void test(void);
void initTimer1(void);

/*  変数    */
OrderMotVel order;
#ifdef MAIN_BOARD_VER5
    char MD_Nomber;
#endif
unsigned int ecan1txmsgBuf[NUM_OF_ECAN_BUFS][8] __attribute__((space(xmemory)));
unsigned int ecan1rxmsgBuf[NUM_OF_ECAN_BUFS][8] __attribute__((space(xmemory)));
static short checkRXCounter = 0;
static short checkRXCounter_last = 0;
static char count = 0;


void initCAN(void)
{
    /* CAN設定モード*/
    C1CTRL1bits.REQOP   = CONFIG_MODE;
    while(C1CTRL1bits.REQOP != CONFIG_MODE);
    /*↑コンフィグモードにすることでCiCFG#レジスタを設定できる*/
    //C1CTRL1bits.WIN     = FILTER_WINDOW;
    C1CTRL1bits.WIN     = BUFFER_WINDOW;;

    /* ボーレートの設定 */
    C1CTRL1bits.CANCKS  = FCAN_IS_FP;
    C1CTRL1bits.CSIDL = 0;
    C1CTRL1bits.CANCAP = 0;

    C1CFG1bits.SJW      = SJW_2TQ;
    //C1CFG1bits.BRP      = CAN_BAUD_PRESCALE(2); //500kbps
    //C1CFG1bits.BRP      = CAN_BAUD_PRESCALE(10);  //100kbps
    C1CFG1bits.BRP      = CAN_BAUD_PRESCALE(1);  //1Mbps
    C1CFG2bits.WAKFIL   = NO_WAKEUP_FIL;
    C1CFG2bits.SEG2PHTS = FREE_PROGRAMMABLE;
    C1CFG2bits.SEG2PH   = CAN_SEGMENT_BIT(6);
    C1CFG2bits.SAM      = BUSLINE_SAMPLING_1;
    C1CFG2bits.SEG1PH   = CAN_SEGMENT_BIT(8);
    C1CFG2bits.PRSEG    = CAN_SEGMENT_BIT(5);

    C1CTRL1bits.REQOP   = NORMAL_MODE;
    while(C1CTRL1bits.OPMODE != NORMAL_MODE);
    //C1FCTRLbits.DMABS   = DMA_BUF_SIZE_32;
    //C1FCTRLbits.FSA     = 31;   /* FSA(FIFO Start Area) = 0~31*/
    C1FCTRL = 0xC01F;   // No FIFO, 32 Buffers

    initDMA();
    initCANFilter();


    C1TR01CONbits.TXEN0 = 1;        //1:送信バッッファ　0:受信バッファ
    C1TR01CONbits.TX0PRI = 0b01;    //優先度
    C1RXFUL1 = C1RXFUL2 = C1RXOVF1 = C1RXOVF2 = 0x00000;
    
  //  C1CTRL1bits.REQOP   = NORMAL_MODE;
  //  while(C1CTRL1bits.OPMODE != NORMAL_MODE);
    /* Write to message buffer 0 */
    /* CiTRBnSID = 0bxxx1 0010 0011 1100
       IDE = 0b0
       SRR = 0b0
       SID<10:0>= 0b100 1000 1111
     * SID<10:0>= 0b001 0000 0000
     */
#ifndef MAIN_BOARD_VER5
    #ifdef MotNum_0
        //return order.Mot0OrderVel;
       ecan1txmsgBuf[0][0] = 0x0400;//ID:100
    #endif
    #ifdef MotNum_1
        ecan1txmsgBuf[0][0] = 0x0404;//ID:101
    #endif
    #ifdef MotNum_2
        ecan1txmsgBuf[0][0] = 0x0408;//ID:102
    #endif
    #ifdef MotNum_3
        ecan1txmsgBuf[0][0] = 0x040C;//ID:103
    #endif
#else
        switch(MD_Nomber){
            case 1:
                ecan1txmsgBuf[0][0] = 0x040C;//ID:103
                break;
            case 2:
                ecan1txmsgBuf[0][0] = 0x0404;//ID:101
                break;
            case 3:
                ecan1txmsgBuf[0][0] = 0x0408;//ID:102
                break;
            default:
                break;
        }
#endif
    
    /* CiTRBnEID = 0bxxxx 0000 0000 0000
       EID<17:6> = 0b0000 0000 0000 */
    ecan1txmsgBuf[0][1] = 0x0000;
/* CiTRBnDLC = 0b0000 0000 xxx0 1111
       EID<17:6> = 0b000000
       RTR = 0b0
       RB1 = 0b0
       RB0 = 0b0
       DLC = 0b1111 */
    ecan1txmsgBuf[0][2] = 0x0006;
/* Write message data bytes */
    ecan1txmsgBuf[0][3] = 0xaaaa;
    ecan1txmsgBuf[0][4] = 0xaaaa;
    ecan1txmsgBuf[0][5] = 0xaaaa;
//    ecan1msgBuf[0][6] = 0xabcd;
/* Request message buffer 0 transmission */


//    C1TR01CONbits.TXREQ0 = 1;

    initCANINT();
    initTimer1();
#ifdef MD_ver4
    TRISBbits.TRISB6 = 1;
    RPINR26 = 0;
    RPINR26bits.C1RXR = 0x26;
    RPOR2bits.RP39R = 0b001110;
#endif
#ifdef MD_ver5
    #ifndef MAIN_BOARD_VER5
    TRISBbits.TRISB9 = 1;
    RPINR26 = 0;
    RPINR26bits.C1RXR = 0x29;
    RPOR3bits.RP40R = 0b001110;
    #else
    TRISBbits.TRISB8 = 1;
    RPINR26 = 0;
    RPINR26bits.C1RXR = 0x28;
    RPOR3bits.RP41R = 0b001110;
    #endif
#endif

}
void initDMA(void)
{
    DMAPWC = 0;
    DMARQC = 0;
    /*          TX                                  */
//    DMA1CONbits.SIZE    = DMA_TRANS_SIZE_WORD;
//    DMA1CONbits.DIR     = DMA_TRANS_DIR_READ;
//    DMA1CONbits.AMODE   = DMA_PERI_INDIRECT_ADDRES;
//    DMA1CONbits.MODE    = DMA_CONTINU_DIS_PINPON;
//    DMA1REQbits.IRQSEL  = DMA_MODULE_CAN;
//    DMA1CNT             = 7;
//    DMA1PAD = (volatile unsigned int)&C1RXD;
//    DMA1STAH = (unsigned int) &ecan1MsgBuf;
//    DMA1STAL = (unsigned int) &ecan1MsgBuf;
//    DMA1CONbits.CHEN    = DMA_CH_ON;

    DMA0CONbits.SIZE = 0;
    DMA0CONbits.DIR =  1;
    DMA0CONbits.AMODE = 0b10;
    DMA0CONbits.MODE = 0b00;
    DMA0REQ = 70;
    DMA0CNT = 7;
    DMA0PAD = (volatile unsigned int) & C1TXD;
//    DMA0STAL = __builtin_dmaoffset(&ecan1msgBuf);
//    DMA0STAH = __builtin_dmapage(&ecan1msgBuf);

    DMA0STAL = (uint16_t) & ecan1txmsgBuf;
    DMA0STAH = (uint16_t) & ecan1txmsgBuf;

    DMA0CONbits.CHEN = 1;

   /*          RX                                  */
    DMA1CONbits.SIZE = 0;
    DMA1CONbits.DIR =  0;
    DMA1CONbits.AMODE = 0b10;
    DMA1CONbits.MODE = 0b00;
    DMA1REQ = 34;
    DMA1CNT = 7;
    DMA1PAD = (volatile unsigned int) & C1RXD;
//    DMA0STAL = __builtin_dmaoffset(&ecan1msgBuf);
//    DMA0STAH = __builtin_dmapage(&ecan1msgBuf);

    DMA1STAL = (uint16_t) & ecan1rxmsgBuf[1][0];
    DMA1STAH = (uint16_t) & ecan1rxmsgBuf[1][0];

    DMA1CONbits.CHEN = 1;

//現状以下の割り込みを許可するとCANがStuffErrorを起こす
//    IEC0bits.DMA0IE = 1;
//    IEC0bits.DMA1IE = 1;
}

void initCANFilter(void)
{
    C1CTRL1bits.WIN      = FILTER_WINDOW;
    /* ************************************** */
    /*              フィルタ設定                */
    /* ************************************** */

    C1FMSKSEL1bits.F0MSK = MASK_REG0_SELECT;
    C1RXM0SIDbits.SID = 0x7FF;
    C1RXF0SIDbits.SID = 0x1AA;
    /* フィルタの受信識別子設定*/
    C1RXM0SIDbits.MIDE = 0x1;
    C1RXF0SIDbits.EXIDE= STANDARD_ADDRESS;
    C1BUFPNT1bits.F0BP = 0xA;
    C1FEN1bits.FLTEN0=0x1;

    /* ************************************** */
    C1CTRL1bits.WIN= BUFFER_WINDOW;
}

void initCANINT(void)
{
    /* 初期化 */
    C1INTE = 0;
    C1INTF = 0;

    /* 割り込み有効設定 */
    IEC2bits.C1IE = 1;      //ECAN1割り込み許可
    C1INTEbits.RBIE = 1;

    /* 割り込み優先度設定 */
    IPC8bits.C1IP = 0b110;  //優先度6

}

void __attribute__(( interrupt, auto_psv)) _C1Interrupt(void)
{
    //F_RX_CAN = 1;
    if(C1INTFbits.RBIF == 1)
    {

//        LED_CAN_ENABLE = 0;

               
        if(C1RXFUL1bits.RXFUL10 == 1){
                LED_CAN_ENABLE = 1;
                checkRXCounter++;
                exchangeOrderData();
//                ecan1txmsgBuf[0][3] = (unsigned int)order.Mot0OrderVel;
//                ecan1txmsgBuf[0][4] = (unsigned int)order.Mot1OrderVel;
//                C1TR01CONbits.TXREQ0 = 1;
                C1RXFUL1bits.RXFUL10 = 0;
        }
        C1INTFbits.RBIF = 0;

    }
    IFS2bits.C1IF = 0;
    //F_RX_CAN = 0;
}

void exchangeOrderData(void)
{
    //もう少し汎用的にしたい
    memcpy(&order,&ecan1rxmsgBuf[1][3],sizeof(order));
    reqWheel_speed_can_1 = (short)order.Mot1OrderVel;
    reqWheel_speed_can_2 = (short)order.Mot2OrderVel;
    reqWheel_speed_can_0 = (short)order.Mot3OrderVel;
}

signed short getOrder(void)
{
#ifndef MAIN_BOARD_VER5
    #ifdef MotNum_0
        //return order.Mot0OrderVel;
        return order.Mot3OrderVel;
    #endif
    #ifdef MotNum_1
        return order.Mot1OrderVel;
    #endif
    #ifdef MotNum_2
        return order.Mot2OrderVel;
    #endif
    #ifdef MotNum_3
        return order.Mot3OrderVel;
    #endif

#else
        switch(MD_Nomber){
            case 1:
                return order.Mot3OrderVel;
                break;
            case 2:
                return order.Mot1OrderVel;
                break;
            case 3:
                return order.Mot2OrderVel;
                break;
            default:
                return 0;
                break;
        }
#endif
    
    return 0;
}

void initTimer1(void)
{
    T1CON = 0;              // Timer reset
    IFS0bits.T1IF = 0;      // Reset Timer1 interrupt flag
    IPC0bits.T1IP = 0b111;  //優先度7
    IEC0bits.T1IE = 1;      // Enable Timer1 interrupt
    TMR1 = 0x0000;
    PR1 = 0x9D0;           // Timer1 period register = ?????
    T1CONbits.TCKPS = 2;    // 1:64
    T1CONbits.TON = 1;      // Enable Timer1 and start the counter
    /*Timer 50msec*/
}

void __attribute__ ( (interrupt, no_auto_psv) ) _T1Interrupt( void )
{
    IFS0bits.T1IF = 0;
    T1CONbits.TON = 0;
    
    count ++;
    //CAN送信用
    tx_MotRevReal  = getWheelAngularVelocity();
    tx_MotRevOrder = getOrder();
    
    ecan1txmsgBuf[0][3] = (unsigned int)tx_MotRevOrder;
    ecan1txmsgBuf[0][4] = (unsigned int)tx_MotRevReal;
    ecan1txmsgBuf[0][5] = (unsigned int)tx_MotDutyOrder;
    
    
    C1TR01CONbits.TXREQ0 = 1; 
    if(count > 100){
        /*50msecに一回CANを受信しているか確認する*/
        if(checkRXCounter != checkRXCounter_last)
        {
            //LED_CAN_ENABLE = 1;
        }
        else if(checkRXCounter == checkRXCounter_last)
        {
            //LED_CAN_ENABLE = 0;
        }

        checkRXCounter_last = checkRXCounter;
        count = 0;
    }

    TMR1 = 0;
    T1CONbits.TON = 1;
    LED_CAN_ENABLE = 0;
    /* reset Timer 1 interrupt flag */
}