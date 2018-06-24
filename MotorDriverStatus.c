#include <xc.h>
#include "MotorDriverStatus.h"

static Status_char driver_status;

short reqWheel_speed;
short reqWheel_speed_can_0;
short reqWheel_speed_can_1;
short reqWheel_speed_can_2;
short resWheel_speed;
float reqWheel_speed_PID;
float resWheel_speed_PID;
float Output;
int reqWheel_speed_PID_int;
int resWheel_speed_PID_int;
int Output_int;
int Output_Duty;
int order_float_int;
int ref_omega_int;
int output_int[4];
char rotate_chaeck;

static void setDriving(void){
    driver_status.status.Driving    = 1;
}
static void clearDriving(void){
    driver_status.status.Driving    = 0;
}

static void setTimeout(void){
    driver_status.status.Timeout    = 1;
}
static void clearTimeout(void){
    driver_status.status.Timeout    = 0;
}

static void setFullSpeed(void){
    driver_status.status.FullSpeed    = 1;
}
static void clearFullSpeed(void){
    driver_status.status.FullSpeed    = 0;
}

static void setOC(void){
    driver_status.status.OC    = 1;
}
static void clearOC(void){
    driver_status.status.OC    = 0;
}

void stateFlagDriving(unsigned char flag){
    if(flag == 0){
        clearDriving();
    }else if(flag == 1){
        setDriving();
    }
}

void stateFlagTimeout(unsigned char flag){
    if(flag == 0){
        clearTimeout();
    }else if(flag == 1){
        setTimeout();
    }
}

void stateFlagFullSpeed(unsigned char flag){
    if(flag == 0){
        clearFullSpeed();
    }else if(flag == 1){
        setFullSpeed();
    }
}

void stateFlagOC(unsigned char flag){
    if(flag == 0){
        clearOC();
    }else if(flag == 1){
        setOC();
    }
}

unsigned char getMDStatus(void){
    return driver_status.c_status;
}