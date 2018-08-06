/* 
 * File:   hspwm_config.h
 * Author: Tomoaki
 *
 * Created on 2017/02/06, 19:29
 */

#ifndef HSPWM_CONFIG_H
#define	HSPWM_CONFIG_H

#include <hspwm.h>

unsigned int pwmcon_conf    =   PWM_FLT_INT_DIS |       //fault interrupt
                                PWM_CL_INT_DIS |        //Current Limit Interrupt
                                PWM_TRG_INT_DIS |       //Trigger interrupt
                                PWM_TB_MODE_PH |        //中央揃え（三角波）のときはMODE_PH(ITB=1)
                                PWM_D_CYLE_DC |         //PDCxを使うときはCYCLE_DC
                                PWM_DT_POS |            //dead time：positive dead time
                                PWM_HS_LL_DTCMP_0 |     //デッドタイムの極性設定 DTC=11のときのみ有効
                                PWM_PRI_TB |
                                PWM_CENTER_ALIGN_EN |   //三角波モード
                                PWM_EXT_RES_DIS |       //外部入力（電流制限など）を使用しない
                                PWM_DC_UPDATE_IMM;     //各種ステータスの更新をPWMの周期に合わせる

unsigned int iocon_conf     =   PWM_H_PIN_EN |          //PWMxHを使う
                                PWM_L_PIN_EN |          //PWMxLを使う
                                PWM_H_PIN_ACTHIGH |     //PWMxH is active high
                                PWM_L_PIN_ACTHIGH |     //PWMxL is active high
                                PWM_PAIR_COMP |         //PWM I/O pin mode(complementary, push pull...)
                                PWM_ORENH_PWMGEN |      //PWMxHにOVRDAT<1>をオーバーライドさせない
                                PWM_ORENL_PWMGEN |      //PWMxLにOVRDAT<0>をオーバーライドさせない
                                PWM_ORENH_OVRDAT_LL |   //OVRDAT<1:0> = 00
                                PWM_FLT_EN_FLTDAT_LL|   //FLTDAT<1:0> = 00
                                PWM_CL_EN_CLDAT_LL  |   //CLDAT<1:0>  = 00
                                PWM_PIN_SWAP_EN    |   //PWMxLとPWMxHを入れ替える
                                PWM_OR_OVRDAT_NXT_CLK;  //オーバーライドの周期

unsigned int phase1_conf    =   1000;   //PWMxHの周期

unsigned int trgcon_donf    =   PWM_TRIG_EVENT1 |       //トリガーイベントを毎回発生
                                PWM_TRIG_PS0;           //

unsigned int sphase1_conf   =   0;
///////////////////////////////////////////////////////////////////////////////////////////
unsigned int fclcon_conf    =   PWM_IND_FLT_DIS  |       //普通のfaultモード
                                PWM_CL_DIS  |           //電流制限モードオフ
                                PWM_FLT_DIS;            //フォルトしない
//                                PWM_HL_FLTDAT_LATCH |            //fault mode:latched
//                                PWM_FLT_SOURCE_LOW  |   //active low
//                                PWM_FLT_FLT1;           //source -> flt1
//MD_ver4用
unsigned int fclcon_conf_md4    =   PWM_IND_FLT_DIS  |       //普通のfaultモード
                                PWM_CL_DIS  |           //電流制限モードオフ
                                PWM_FLT_DIS;

////////////////////////////////////////////////////////////////////////////////
unsigned int lebcon_conf    =   0;                      //LEB：スイッチングノイズで電流制限がかからないようにスイッチング時は取り込まないようにする設定
////////////////////////////////////////////////////////////////////////////////
unsigned int ptcon_conf     =   PWM_MOD_EN  |           //pwm enable
                                PWM_IDLE_CONT   |       //
                                PWM_SEVT_INT_DIS    |   //special event interrupt
                                PWM_PER_UPDATE_BOUND|   //周期変更のタイミング
                                PWM_SYNCPOL_HIGH_ACT|   //
                                PWM_PTB_SYNCO_EN    |   //
                                PWM_EXT_PTB_SYNC_DIS;

unsigned int ptcon2_conf    =   PWM_INPUT_CLK_DIV1;

unsigned int ptper_conf     =   1024;
unsigned int sevtcmp_conf   =   512;

////////////////////////////////////////////////////////////////////////////////
unsigned int dtr_conf       = 0;
unsigned int aldtr_conf     = 10;   //dead time=240ns

#endif	/* HSPWM_CONFIG_H */

