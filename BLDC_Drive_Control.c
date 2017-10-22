#include <xc.h>
#include <hspwm.h>
#include <timer.h>
#include <pps.h>

#include "BLDC_Drive_Control.h"
#include "hspwm_config.h"
#include "MotorDriverStatus.h"

//#define MD_ver4
#define MD_ver5


//設定関数
static void configHallSensorPort(void);
static void configCurrentFault(void);
static void configHSPWM(void);
static void configQEI(void);
static void configTimer2(void);
static void configTimer3(void);
static int CL_count = 0;
static int CL_fault_Flag = 0;
//モータ駆動関係
static unsigned int getHallPosition(void);
#ifdef MD_ver4
#define HALL1   PORTAbits.RA4
#define HALL2   PORTBbits.RB1
#define HALL3   PORTBbits.RB0
#endif

#ifdef MD_ver5
#define HALL1   PORTBbits.RB4
#define HALL2   PORTAbits.RA4
#define HALL3   PORTBbits.RB5
#endif

#define HALL_LOW    0
#define HALL_HIGH   1
static void ctrlGateOverride(unsigned int pwm_num, unsigned int H_over_en, unsigned int L_over_en, unsigned int override_gate_signal);
#define GATE_OVERRIDE_EN    1
#define GATE_OVERRIDE_DIS   0
#define GATE_OVR_H0L0       0
#define GATE_OVR_H0L1       1
#define GATE_OVR_H1L0       2
#define GATE_OVR_H1L1       3
static void driveTreePhaseInverter( unsigned int pattern, unsigned int duty, unsigned int rotate_direction );
#define ROTATE_CCW  0   //車輪CWを正
#define ROTATE_CW   1
static float getPID(float Kp, float Ki, float Kd, float reffernce, float measuered);    //ref&measured -> rad/s
static float rotate_Kp  = 0;
static float rotate_Ki  = 0;
static float rotate_Kd  = 0;
static float ref_omega  = 0;
static float mes_omega  = 0;
static float getAngularVelocity(int cnt_encoder);   //エンコーダのカウントから車輪の回転角を返す．
#define SMP_RATE    2000.0    //sampling rate 2kHz->500us
#define GEAR_RATIO  2.4  //60/25
#define ENCODER_RESOLUTION  0.0015339 //pi/2048   rad/cnt

static int count = 0;

//**************************************************************
//**************************************************************
//                   各種設定
//**************************************************************
//**************************************************************
extern void configBLDCSystem(void){
    configHallSensorPort();
#ifdef MD_ver5
    configCurrentFault();
#endif
    configQEI();
    configTimer2();
    configHSPWM();
    setPIDGain(0.00003, 0.000003, 0.0);
}

//**************************************************************
//**************************************************************
//                   BLDC駆動
//**************************************************************
//**************************************************************
extern void driveBLDCSystem(void){
//    int duty = 1024.0 * getPID(rotate_Kp, rotate_Ki, rotate_Kd, (int)ref_omega, mes_omega );
    int duty = 1024 * getPID(rotate_Kp, rotate_Ki, rotate_Kd, ref_omega, mes_omega );
//    int duty = 1024.0 * getPID(rotate_Kp, rotate_Ki, rotate_Kd, 5, mes_omega );

    int rotate;
    if(duty > 0){
        rotate  = ROTATE_CW;
    }else{
        rotate  = ROTATE_CCW;
        duty    = -1*duty;
    }
    driveTreePhaseInverter( getHallPosition(), 800, 0 );
    stateFlagDriving( 1 );
//    driveTreePhaseInverter(getHallPosition(), 50, rotate);
}

extern void setPIDGain(float Kp, float Ki, float Kd){
    rotate_Kp   = Kp;
    rotate_Ki   = Ki;
    rotate_Kd   = Kd;
}

extern void setReffernceAngularVelocity(float omega){   //車輪CWが正
    ref_omega   = -1.0 * omega / SMP_RATE * GEAR_RATIO / ENCODER_RESOLUTION;  //カウント数　= 車輪の回転数 / サンプリング周波数 * 増速比 *エンコーダの分解能
}
extern void setMeasuredAngularVelocity(float omega){
    mes_omega   = omega;
}
void __attribute__((  interrupt, auto_psv))  _T2Interrupt(void){
    int velocity = 0;
    int duty;
    IFS0bits.T2IF   = 0;
    velocity    = VEL1CNT;
    count   += velocity;
//    if( count>9083 ){
//        LATAbits.LATA1  = 1;
//    }else{
//        LATAbits.LATA1  = 0;
//    }

    
//    setMeasuredAngularVelocity( getAngularVelocity( velocity ) );
    setMeasuredAngularVelocity((float)velocity);
    driveBLDCSystem();
}

void __attribute__((  interrupt, auto_psv))  _PWM1Interrupt(void){
    IFS5bits.PWM1IF = 0;
    PWMCON1bits.FLTSTAT = 0;
    stateFlagOC(1);
    stateFlagDriving(0);
}


//**************************************************************
//アクセス関数
short getWheelAngularVelocity(void){
    return (short)-1*mes_omega * SMP_RATE * ENCODER_RESOLUTION / GEAR_RATIO;    //車輪回転数 = パルス数*サンプリング周波数 * エンコーダ分解能 / ギヤ比
}

unsigned char getMotorDriverStatus(void){
    return getMDStatus();
}



//**************************************************************
//BLDC駆動
static void ctrlGateOverride(unsigned int pwm_num, unsigned int H_over_en, unsigned int L_over_en, unsigned int override_gate_signal){
    switch(pwm_num){
        case 1:
            IOCON1bits.OVRENH   = H_over_en;
            IOCON1bits.OVRENL   = L_over_en;
            IOCON1bits.OVRDAT   = override_gate_signal;
            break;
        case 2:
            IOCON2bits.OVRENH   = H_over_en;
            IOCON2bits.OVRENL   = L_over_en;
            IOCON2bits.OVRDAT   = override_gate_signal;
            break;
        case 3:
            IOCON3bits.OVRENH   = H_over_en;
            IOCON3bits.OVRENL   = L_over_en;
            IOCON3bits.OVRDAT   = override_gate_signal;
            break;
    }
}

static unsigned int getHallPosition(void){
    if( (HALL1==HALL_HIGH) && (HALL2==HALL_LOW) && (HALL3==HALL_HIGH) ){
        return 6;
    }
    else if( (HALL1==HALL_HIGH) && (HALL2==HALL_LOW) && (HALL3==HALL_LOW) ){
        return 5;
    }
    else if( (HALL1==HALL_HIGH) && (HALL2==HALL_HIGH) && (HALL3==HALL_LOW) ){
        return 4;
    }
    else if( (HALL1==HALL_LOW) && (HALL2==HALL_HIGH) && (HALL3==HALL_LOW) ){
        return 3;
    }
    else if( (HALL1==HALL_LOW) && (HALL2==HALL_HIGH) && (HALL3==HALL_HIGH) ){
        return 2;
    }
    else if( (HALL1==HALL_LOW) && (HALL2==HALL_LOW) && (HALL3==HALL_HIGH) ){
        return 1;
    }else{
        return 0;
    }
}

static void driveTreePhaseInverter( unsigned int pattern, unsigned int duty, unsigned int rotate_direction ){
    if(rotate_direction == ROTATE_CCW){
        switch(pattern){
            case 6: //V1-2:+
                ctrlGateOverride( 1, GATE_OVERRIDE_DIS, GATE_OVERRIDE_DIS, GATE_OVR_H0L0 );  //PWM1 high side:PWM, low side:0
                ctrlGateOverride( 2, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L1 );   //PWM2 high side:0, low side:1
                ctrlGateOverride( 3, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L0 );   //PWM3 high side:0, low side:0
                break;
            case 1: //V3-1:-
                ctrlGateOverride( 1, GATE_OVERRIDE_DIS, GATE_OVERRIDE_DIS, GATE_OVR_H0L0 );  //PWM1 high side:PWM, low side:0
                ctrlGateOverride( 2, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L0 );   //PWM2 high side:0, low side:0
                ctrlGateOverride( 3, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L1 );   //PWM3 high side:0, low side:1
                break;
            case 2: //V2-3:+
                ctrlGateOverride( 1, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L0 );   //PWM1 high side:0, low side:0
                ctrlGateOverride( 2, GATE_OVERRIDE_DIS, GATE_OVERRIDE_DIS, GATE_OVR_H0L0 );  //PWM2 high side:PWM, low side:0
                ctrlGateOverride( 3, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L1 );   //PWM3 high side:0, low side:1
                break;
            case 3: //V1-2:-
                ctrlGateOverride( 1, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L1 );   //PWM1 high side:0, low side:1
                ctrlGateOverride( 2, GATE_OVERRIDE_DIS, GATE_OVERRIDE_DIS, GATE_OVR_H0L0 );  //PWM2 high side:PWM, low side:0
                ctrlGateOverride( 3, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L0 );   //PWM3 high side:0, low side:0
                break;
            case 4: //V3-1:+
                ctrlGateOverride( 1, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L1 );   //PWM1 high side:0, low side:1
                ctrlGateOverride( 2, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L0 );   //PWM2 high side:0, low side:0
                ctrlGateOverride( 3, GATE_OVERRIDE_DIS, GATE_OVERRIDE_DIS, GATE_OVR_H0L0 );  //PWM3 high side:PWM, low side:0
                break;
            case 5: //V2-3:-
                ctrlGateOverride( 1, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L0 );   //PWM1 high side:0, low side:0
                ctrlGateOverride( 2, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L1 );   //PWM2 high side:0, low side:1
                ctrlGateOverride( 3, GATE_OVERRIDE_DIS, GATE_OVERRIDE_DIS, GATE_OVR_H0L0 );  //PWM3 high side:PWM, low side:0
                break;
        }
    }else if(rotate_direction == ROTATE_CW){
        switch(pattern){
            case 2: //V1-2:-
                ctrlGateOverride( 1, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L1 );  //PWM1 high side:PWM, low side:0
                ctrlGateOverride( 2, GATE_OVERRIDE_DIS, GATE_OVERRIDE_DIS, GATE_OVR_H0L0 );   //PWM2 high side:0, low side:1
                ctrlGateOverride( 3, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L0 );   //PWM3 high side:0, low side:0
                break;
            case 3: //V3-1:+
                ctrlGateOverride( 1, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L1 );  //PWM1 high side:PWM, low side:0
                ctrlGateOverride( 2, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L0 );   //PWM2 high side:0, low side:0
                ctrlGateOverride( 3, GATE_OVERRIDE_DIS, GATE_OVERRIDE_DIS, GATE_OVR_H0L0 );   //PWM3 high side:0, low side:1
                break;
            case 4: //V2-3:-
                ctrlGateOverride( 1, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L0 );   //PWM1 high side:0, low side:0
                ctrlGateOverride( 2, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L1 );  //PWM2 high side:PWM, low side:0
                ctrlGateOverride( 3, GATE_OVERRIDE_DIS, GATE_OVERRIDE_DIS, GATE_OVR_H0L0 );   //PWM3 high side:0, low side:1
                break;
            case 5: //V1-2:+
                ctrlGateOverride( 1, GATE_OVERRIDE_DIS, GATE_OVERRIDE_DIS, GATE_OVR_H0L0 );   //PWM1 high side:0, low side:1
                ctrlGateOverride( 2, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L1 );  //PWM2 high side:PWM, low side:0
                ctrlGateOverride( 3, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L0 );   //PWM3 high side:0, low side:0
                break;
            case 6: //V3-1:-
                ctrlGateOverride( 1, GATE_OVERRIDE_DIS, GATE_OVERRIDE_DIS, GATE_OVR_H0L0 );   //PWM1 high side:0, low side:1
                ctrlGateOverride( 2, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L0 );   //PWM2 high side:0, low side:0
                ctrlGateOverride( 3, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L1 );  //PWM3 high side:PWM, low side:0
                break;
            case 1: //V2-3:+
                ctrlGateOverride( 1, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L0 );   //PWM1 high side:0, low side:0
                ctrlGateOverride( 2, GATE_OVERRIDE_DIS, GATE_OVERRIDE_DIS, GATE_OVR_H0L0 );   //PWM2 high side:0, low side:1
                ctrlGateOverride( 3, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L1 );  //PWM3 high side:PWM, low side:0
                break;
        }
    }

    if(duty==0){
        ctrlGateOverride( 1, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L1 );   //PWM1 high side:0, low side:0
        ctrlGateOverride( 2, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L1 );   //PWM2 high side:0, low side:1
        ctrlGateOverride( 3, GATE_OVERRIDE_EN, GATE_OVERRIDE_EN, GATE_OVR_H0L1 );  //PWM3 high side:PWM, low side:0
    }

    SetHSPWMDutyCycle1(duty);
    SetHSPWMDutyCycle2(duty);
    SetHSPWMDutyCycle3(duty);
}

static float getPID(float Kp, float Ki, float Kd, float reffernce, float measuered){
    static float error[3]={0,0,0};    //error[n]:nサンプリング前(0は現在)
    static float output[2]={0,0};

    error[0] = reffernce - measuered;

    output[1] = Ki*error[0] + Kp*error[1]+Kd*error[2];
    output[0] += output[1];

    if(output[0]>=1.0){
        output[0]=1.0;
        stateFlagFullSpeed(1);
    }else if(output[0]<-1.0){
        output[0]=-1.0;
        stateFlagFullSpeed(1);
    }else{
        stateFlagFullSpeed(0);
    }

    error[2]    = error[1];
    error[1]    = error[0];


    return output[0];
}

//static float getAngularVelocity(int cnt_encoder){
//    return (float)cnt_encoder * ENCODER_RESOLUTION * GEAR_RATIO *SMP_RATE * -1; //車輪の回転数[rad/s] = エンコーダのカウント数 * 1カウントあたりの角度 * ギヤ比 * サンプリング周波数 *-1(回転は逆になる）
//}
//**************************************************************
//各種モジュールの設定
static void  configHallSensorPort(void){
#ifdef MD_ver4
    TRISAbits.TRISA4    = 1;    //hall 1
    CNPUAbits.CNPUA4    = 1;

    TRISBbits.TRISB1    = 1;    //hall 2
    CNPUBbits.CNPUB1    = 1;

    TRISBbits.TRISB0    = 1;    //hall 3
    CNPUBbits.CNPUB0    = 1;
#endif
#ifdef MD_ver5
    TRISBbits.TRISB4    = 1;    //hall 1
    CNPUBbits.CNPUB4    = 1;

    TRISAbits.TRISA4    = 1;    //hall 2
    CNPUAbits.CNPUA4    = 1;

    TRISBbits.TRISB5    = 1;    //hall 3
    CNPUBbits.CNPUB5    = 1;
#endif
}

static void configCurrentFault(void){
    IEC5bits.PWM1IE = 1;
    TRISBbits.TRISB3    = 1;    //RB3->FLT1
    iPPSInput( IN_FN_PPS_FLT1, IN_PIN_PPS_RP35 );
}

static  void configTimer2(void){
    //Config Timer2
    OpenTimer2(T2_ON & T2_GATE_OFF & T2_PS_1_8 & T2_32BIT_MODE_OFF & T2_SOURCE_INT, 2500);  //2kHz
    ConfigIntTimer2(T2_INT_PRIOR_3 & T2_INT_ON);
}

static  void configTimer3(void){
    //Config Timer3
    OpenTimer3(T3_OFF & T3_GATE_OFF & T3_PS_1_256 & T3_SOURCE_INT, 62500);  //400 ms
    ConfigIntTimer3(T3_INT_PRIOR_3 & T3_INT_ON);
}

static void  configQEI(void){
#ifdef MD_ver4
    TRISBbits.TRISB4    = 1;    //channel A -> input
    TRISBbits.TRISB5    = 1;    //channel B -> input

    iPPSInput(IN_FN_PPS_QEA1, IN_PIN_PPS_RP36);
    iPPSInput(IN_FN_PPS_QEB1, IN_PIN_PPS_RP37);
#endif

#ifdef MD_ver5
    TRISBbits.TRISB6    = 1;    //channel A -> input
    TRISBbits.TRISB7    = 1;    //channel B -> input

    iPPSInput(IN_FN_PPS_QEA1, IN_PIN_PPS_RP38);
    iPPSInput(IN_FN_PPS_QEB1, IN_PIN_PPS_RP39);
#endif
    QEI1CONbits.QEISIDL = 1;        //アイドル時停止
    QEI1CONbits.PIMOD   = 0b110;    //速度計測モードでは無視される
    QEI1CONbits.IMV     = 0b00;     //インデックスは不使用
    QEI1CONbits.INTDIV  = 0b000;    //prescaler 1:1
    QEI1CONbits.CNTPOL  = 0;        //
    QEI1CONbits.GATEN   = 0;        //gate signal -> disable
    QEI1CONbits.CCM     = 0b00;     //quadrature mode

    QEI1IOCbits.QCAPEN  = 0;        //home  -> disable
    QEI1IOCbits.FLTREN  = 1;        //filter -> enable
    QEI1IOCbits.QFDIV   = 0b000;    //filter clock divide -> 1:1
    QEI1IOCbits.OUTFNC  = 0b00;     //output is disabled
    QEI1IOCbits.SWPAB   = 0;        //not swap
    QEI1IOCbits.HOMPOL  = 0;        //home input -> invert
    QEI1IOCbits.IDXPOL  = 0;        //
    QEI1IOCbits.QEBPOL  = 0;
    QEI1IOCbits.QEAPOL  = 0;

    QEI1CONbits.QEIEN   = 1;    //qei -> enable;
}

static void  configHSPWM(void){
    //Config HSPWM module
    ConfigHSPWM1( pwmcon_conf, iocon_conf, phase1_conf, trgcon_donf, sphase1_conf );
#ifdef MD_ver4
    ConfigHSPWMFault1( fclcon_conf_md4 );
#endif
#ifdef MD_ver5
    ConfigHSPWMFault1( fclcon_conf );
#endif
    ConfigHSPWMLeb1( lebcon_conf );
    SetHSPWMDeadTime1( dtr_conf, aldtr_conf );
#ifdef MD_ver5
    PWMCON1bits.FLTIEN  = 1;    //過電流制限をPWM1が代表で割込み
#endif

    ConfigHSPWM2( pwmcon_conf, iocon_conf, phase1_conf, trgcon_donf, sphase1_conf );
#ifdef MD_ver4
    ConfigHSPWMFault2( fclcon_conf_md4 );
#endif
#ifdef MD_ver5
    ConfigHSPWMFault2( fclcon_conf );
#endif
    ConfigHSPWMLeb2( lebcon_conf );
    SetHSPWMDeadTime2( dtr_conf, aldtr_conf );

    ConfigHSPWM3( pwmcon_conf, iocon_conf, phase1_conf, trgcon_donf, sphase1_conf );
#ifdef MD_ver4
    ConfigHSPWMFault3( fclcon_conf_md4 );
#endif
#ifdef MD_ver5
    ConfigHSPWMFault3( fclcon_conf );
#endif
    ConfigHSPWMLeb3( lebcon_conf );
    SetHSPWMDeadTime3( dtr_conf, aldtr_conf );

    OpenHSPWM( ptcon_conf, ptcon2_conf, ptper_conf, sevtcmp_conf ); //open HSPWM module
}