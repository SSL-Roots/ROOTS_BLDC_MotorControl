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

