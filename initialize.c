#include <xc.h>
#include "initialize.h"
#include "config_can.h"

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
    _PLLDIV     = 41;
#endif
#ifdef LPC4088
    _PLLDIV     = 42;
#endif

    while(!OSCCONbits.LOCK);
}

static void initializeIO(void){
    ANSELA  = 0x0000;
    PORTA   = 0x0000;
    TRISA   = 0x0000;

    ANSELB  = 0x0000;
    PORTB   = 0x0000;
    TRISB   = 0x0000;
}

void initializeSystems(void){
    initializeOsc();
    initializeIO();
    initCAN();
}

