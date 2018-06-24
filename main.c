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
#include "xprintf.h"
#include "uart_dsPIC33F.h"

signed short order_signed;


static char counter;

int main(void){
//    char stat;
    
    float order_float = 0.0;
    initializeSystems();
    configBLDCSystem();
    
#ifdef DEBUG_MODE
       xdev_out(putcUart);
#endif
        
    while(1){
        order_signed = getOrder();
        reqWheel_speed = order_signed / 10;
        resWheel_speed = getWheelAngularVelocity();
        
        order_float = (float)order_signed * 0.05;
        order_float_int = (int)(order_float * 1000);
        
        setReffernceAngularVelocity(order_float);
//        C1TR01CONbits.TXREQ0 = 1; 
        
        while(C1TR01CONbits.TXREQ0 == 1)
        {
            #ifdef DEBUG_MODE
              counter++;
              if(counter > 100)
              {
                counter = 0;
                //xprintf("reqWheel_speed: %d \n resWheel_speed:%d \n",reqWheel_speed,resWheel_speed);
                xprintf("reqWheel_speed_PID: %d \n resWheel_speed_PID_int: %d \n",reqWheel_speed_PID_int,resWheel_speed_PID_int);
                //xprintf("P: %d \n I: %d \n Out: %d \n",output_int[0],output_int[1],output_int[3]);
                //xprintf("Out: %d \n",output_int[3]);
                //xprintf("rotate: %d \n",rotate_chaeck);
                xprintf("out:%d\n  rotate_chaeck:%d\n ",output_int[3],rotate_chaeck);
                //xprintf("0: %d \n 1: %d \n 2: %d \n",reqWheel_speed_can_0,reqWheel_speed_can_1,reqWheel_speed_can_2);
                  
              }
            #endif
              
    //        LED1 = 0;
    //__        LED2 = 0;
            if(C1TR01CONbits.TXERR0 == 1)
            {
    //           LED1 = 1;
                C1TR01CONbits.TXREQ0 = 0;
            }
            if(C1TR01CONbits.TXABT0 == 1)
            {
    //            LED2 = 1;
                C1TR01CONbits.TXREQ0 = 0;
            }
        }
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

       return 0;
}
