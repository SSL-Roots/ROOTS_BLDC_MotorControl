#include <xc.h>
#include "initialize.h"
#include "config_can.h"
#include "uart_dsPIC33F.h"

static void initializeOsc(void){

//    RCONbits.SWDTEN = 0;
//
//    _PLLPOST = 0b00;    /* N2:  PLL出力分周 = 2 */
//    _PLLPRE = 0b0000;   /* N1:  PLL入力プリスケーラ = 2*/
//    PLLFBD  = 0x0029;   /* M :  PLL倍率 = 43 */
//
//    while(!OSCCONbits.LOCK); /* waiting PLL Lock */
    /*動作クロック設定*/
    RCONbits.SWDTEN = 0;


    //FRC Oscillator Tuning
#ifdef STM32
    _TUN        = 0b011110;    //FRC = 7.37MHz nominal(0b000000) 7.474MHz(0b011110)
#endif
#ifdef LPC4088
    _TUN        = 0b000000;    //FRC = 7.37MHz nominal(0b000000) 7.474MHz(0b011110)
#endif
    // FRCDIV (default = 1　PostScaler)
    // PostScaler = 1: FIN = 7.37MHz
    //FPLLO = FIN * M/(N1*N2)
    //M  = PLLDIV + 2
    //N1 = PLLPRE + 2
    //N2 = 2*(PLLPOST + 1)
    //且つ以下の条件を満たしていること
    //Fvco = FIN * M / N1 (今回は,Fcvo = 168.165MHz)
    //120MHz < Fcvo < 340MHz
    _PLLPOST    = 0;
    _PLLPRE     = 0;
#ifdef STM32
    //_PLLDIV     = 41;
    _PLLDIV     = 41;
#endif
#ifdef LPC4088
    _PLLDIV     = 42;
#endif

    while(!OSCCONbits.LOCK);
}

static void initializeIO(void){
#ifndef MAIN_BOARD_VER5
    ANSELA  = 0x0000;
    PORTA   = 0x0000;
    TRISA   = 0x0000;

    ANSELB  = 0x0000;
    PORTB   = 0x0000;
    TRISB   = 0x0000;
#else
    //180319_horie プルアップ禁止設定（HALL）
    ANSELA  = 0x0000;
    PORTA   = 0x0000;
    TRISA   = 0x000C;
    CNPUAbits.CNPUA4 = 0;
    CNPDAbits.CNPDA4 = 0;

    ANSELB  = 0x0000;
    PORTB   = 0x0000;
    TRISB   = 0x0000;
    CNPUBbits.CNPUB4 = 0;
    CNPUBbits.CNPUB5 = 0;
    CNPDBbits.CNPDB4 = 0;
    CNPDBbits.CNPDB5 = 0;
#endif
}

#ifdef MAIN_BOARD_VER5
static void getMDNomber(void){
    
    char portA2;
    char portA3;
    
    portA2 = (char)PORTAbits.RA2;
    portA3 = (char)PORTAbits.RA3;
    
    if( (portA2 == 0) && (portA3 == 0) ){
        MD_Nomber  = 1;
    }
    else if( (portA2 == 1) && (portA3 == 0) ){
        MD_Nomber  = 2;
    }
    else if( (portA2 == 0) && (portA3 == 1) ){
        MD_Nomber  = 3;
    }

}
#endif


void initializeSystems(void){
    initializeOsc();
    initializeIO();

#ifdef DEBUG_MODE
    initializeUart( 35, 34, FCY*2, UART_BAUDRATE);
#endif
    
    
#ifdef MAIN_BOARD_VER5
    getMDNomber();
#endif
    initCAN();
}

