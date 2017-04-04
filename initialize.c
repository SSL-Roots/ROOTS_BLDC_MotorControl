#include <xc.h>
#include "initialize.h"

static void initializeOsc(void){
    RCONbits.SWDTEN = 0;

    _PLLPOST = 0b00;    /* N2:  PLL出力分周 = 2 */
    _PLLPRE = 0b0000;   /* N1:  PLL入力プリスケーラ = 2*/
    PLLFBD  = 0x0029;   /* M :  PLL倍率 = 43 */

    while(!OSCCONbits.LOCK); /* waiting PLL Lock */
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
}

