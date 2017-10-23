#define	FCY 39613750
#include <xc.h>
#include <libpic30.h>

#include <hspwm.h>
#include <timer.h>

#include "config.h"
#include "initialize.h"

#include "BLDC_Drive_Control.h"
#include "MotorDriverStatus.h"

#include "../ROOTS_BLDC_MotorControl/config_can.h"

void main(void){
    char stat;
    signed short order = 0;
    float order_float = 0.0;
    initializeSystems();
    configBLDCSystem();
    while(1){
        order = getOrder();
        order_float = (float)order * 0.05;
        setReffernceAngularVelocity(order_float);

//        setReffernceAngularVelocity(-6.28*2);
//        __delay_ms(2000);
//        setReffernceAngularVelocity(-6.28*4);
//        __delay_ms(2000);
//        setReffernceAngularVelocity(-6.28*6);
//        __delay_ms(2000);
//        setReffernceAngularVelocity(-6.28*8);
//        __delay_ms(2000);
//        setReffernceAngularVelocity(6.28*2);
//        __delay_ms(2000);
//        setReffernceAngularVelocity(6.28*4);
//        __delay_ms(2000);
//        setReffernceAngularVelocity(6.28*6);
//        __delay_ms(2000);
//        setReffernceAngularVelocity(6.28*8);
//        __delay_ms(2000);
    }

}
