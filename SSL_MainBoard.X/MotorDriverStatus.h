/* 
 * File:   MotorDriverStatus.h
 * Author: Tomoaki
 *
 * Created on 2017/04/26, 19:50
 */

#ifndef MOTORDRIVERSTATUS_H
#define	MOTORDRIVERSTATUS_H

unsigned char getMDStatus(void);

void stateFlagDriving(unsigned char flag);
void stateFlagTimeout(unsigned char flag);
void stateFlagFullSpeed(unsigned char flag);
void stateFlagOC(unsigned char flag);

extern short reqWheel_speed;
extern short resWheel_speed;
extern float reqWheel_speed_PID;
extern float resWheel_speed_PID;
extern float Output;
extern int Output_Duty;

extern int reqWheel_speed_PID_int;
extern short reqWheel_speed_can_0;
extern short reqWheel_speed_can_1;
extern short reqWheel_speed_can_2;
extern int resWheel_speed_PID_int;
extern int Output_int;
extern int Output_Duty;
extern int order_float_int;
extern int ref_omega_int;

extern int output_int[4];

extern char rotate_chaeck;

/**************************************
 
 0x00000000
 * ||||||||
 * ||    |L ----OC
 * ||    L------FullSpeed
 * |L-----------Timeout
 * L------------Driving
 *************************************/

typedef struct MDStatus{
  unsigned OC:1;
  unsigned FullSpeed:1;
  unsigned Unimplemented:4;
  unsigned Timeout:1;
  unsigned Driving:1;
} MDStatus;

typedef union Status_char{
    MDStatus status;
    unsigned char c_status;
}Status_char;

#endif	/* MOTORDRIVERSTATUS_H */

