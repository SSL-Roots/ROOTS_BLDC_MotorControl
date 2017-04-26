/* 
 * File:   BLDC_Drive_Control.h
 * Author: Tomoaki
 *
 * Created on 2017/03/26, 22:02
 */

#ifndef BLDC_DRIVE_CONTROL_H
#define	BLDC_DRIVE_CONTROL_H

void configBLDCSystem(void); //設定
void driveBLDCSystem(void);  //モータを回す

void setReffernceAngularVelocity(float omega ); //rad/s, 車輪CWが正
void setMeasuredAngularVelocity( float omega );    //rad/s，車輪の回転数をセット
void setPIDGain(float Kp, float Ki, float Kd);      //制御自体はパルス数で実行
void setRotationDirection(int direction);

short getWheelAngularVelocity(void);
unsigned char getMotorDriverStatus(void);


#endif	/* BLDC_DRIVE_CONTROL_H */

