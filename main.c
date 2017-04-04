#define	FCY 39613750
#include <xc.h>
#include <libpic30.h>

#include <hspwm.h>
#include <timer.h>

#include "config.h"
#include "initialize.h"

#include "BLDC_Drive_Control.h"


void main(void){
    initializeSystems();
    configBLDCSystem();
    while(1){
        setReffernceAngularVelocity(-6.28*1);
    }
}
