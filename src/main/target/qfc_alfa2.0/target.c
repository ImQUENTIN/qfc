/************************************************************************************************
* @file    target.c
* @author  亓岳鑫
* @email   qiyuexin@yeah.net
* @Version V1.0
* @date    May 11th, 2018.
* @brief   qfc_alfa2.0
************************************************************************************************/

#include <stdint.h>

#include "platform.h"
#include "io.h"
#include "dma.h"
#include "timer.h"
#include "timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {    
   
#if 0
    // 硬件问题：把TXS0108E芯片拿掉直接短接，受到影响的有：
    // RC5(F1)~RC8(F4), SBUS_OUT, UART3_RX/TX, PPM_INPUT
    DEF_TIM(TIM4, CH1, PD12, TIM_USE_PWM,   0, 0), // S1_IN, PITCH
    DEF_TIM(TIM4, CH2, PD13, TIM_USE_PWM,   0, 0), // S2_IN, R0LL
    DEF_TIM(TIM4, CH3, PD14, TIM_USE_PWM,   0, 0), // S3_IN, THRO        
    DEF_TIM(TIM4, CH4, PD15, TIM_USE_PWM,   0, 0), // S4_IN, YAW 
#else
    // 硬件已修复
    DEF_TIM(TIM3, CH1, PB4,  TIM_USE_PWM,   0, 0), // S1_IN, PITCH
    DEF_TIM(TIM3, CH2, PB5,  TIM_USE_PWM,   0, 0), // S2_IN, R0LL    
    DEF_TIM(TIM3, CH3, PB0,  TIM_USE_PWM,   0, 0), // S4_IN, THRO 
    DEF_TIM(TIM3, CH4, PB1,  TIM_USE_PWM,   0, 0), // S3_IN, YAw
    
    DEF_TIM(TIM4, CH1, PD12, TIM_USE_PWM,   0, 0), // S5_IN
    DEF_TIM(TIM4, CH2, PD13, TIM_USE_PWM,   0, 0), // S6_IN
//    DEF_TIM(TIM4, CH3, PD14, TIM_USE_PWM,   0, 0), // S7_IN        
//    DEF_TIM(TIM4, CH4, PD15, TIM_USE_PWM,   0, 0), // S8_IN
#endif
    DEF_TIM(TIM1, CH1, PE9,  TIM_USE_MOTOR, 0, 0), // S5_OUT
    DEF_TIM(TIM1, CH2, PE11, TIM_USE_MOTOR, 0, 0), // S6_OUT
    DEF_TIM(TIM1, CH3, PE13, TIM_USE_MOTOR, 0, 0), // S7_OUT
    DEF_TIM(TIM1, CH4, PE14, TIM_USE_MOTOR, 0, 0), // S8_OUT

//    DEF_TIM(TIM2, CH1, PA15, TIM_USE_BEEPER, 0, 0), // beeper
    DEF_TIM(TIM2, CH3, PA2,  TIM_USE_MOTOR, 0, 0), // sonar echo if needed
    DEF_TIM(TIM2, CH4, PA3,  TIM_USE_ANY,   0, 0), // any

//    DEF_TIM(TIM8, CH1, PC6,  TIM_USE_PPM,   0, 0), // any
//    DEF_TIM(TIM8, CH2, PC7,  TIM_USE_PPM,   0, 0), // any
    
    
};
    
    
#ifdef USE_TARGET_CONFIG
    
#include "common/axis.h"
#include "common/maths.h"
    
#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_modes.h"
#include "fc/rc_controls.h"
    
#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"
    
#include "rx/rx.h"
    
#include "sensors/battery.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#define VBAT_SCALE 101

// alternative defaults settings for MULTIFLITEPICO targets
void targetConfiguration(void)
{

    // 加速度计 零值    

    accelerometerConfigMutable()->accZero.raw[0]  = 0xffc3;
    accelerometerConfigMutable()->accZero.raw[1]  = 0x0045;
    accelerometerConfigMutable()->accZero.raw[2]  = 0xffb0;
    
//  compassConfigMutable()->mag_hardware = MAG_NONE;            // disabled by default

    // 电压计
    voltageSensorADCConfigMutable(VOLTAGE_SENSOR_ADC_VBAT)->vbatscale = VBAT_SCALE;
    voltageSensorADCConfigMutable(VOLTAGE_SENSOR_ADC_VBAT)->vbatresdivval = 10;
    voltageSensorADCConfigMutable(VOLTAGE_SENSOR_ADC_VBAT)->vbatresdivmultiplier = 1;
    batteryConfigMutable()->vbatmaxcellvoltage = 44;
    batteryConfigMutable()->vbatmincellvoltage = 32;
    batteryConfigMutable()->vbatwarningcellvoltage = 33;

//    rxConfigMutable()->spektrum_sat_bind = 5;
//    rxConfigMutable()->spektrum_sat_bind_autoreset = 1;
//
    rcControlsConfigMutable()->yaw_deadband = 2;
    rcControlsConfigMutable()->deadband = 2;

    modeActivationConditionsMutable(0)->modeId          = BOXANGLE;
    modeActivationConditionsMutable(0)->auxChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(0)->range.endStep   = CHANNEL_VALUE_TO_STEP(1400);
    
    modeActivationConditionsMutable(1)->modeId          = BOXHORIZON;
    modeActivationConditionsMutable(1)->auxChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1425);
    modeActivationConditionsMutable(1)->range.endStep   = CHANNEL_VALUE_TO_STEP(1575);

    failsafeConfigMutable()->failsafe_delay = 2;
    failsafeConfigMutable()->failsafe_off_delay = 0;

    gyroConfigMutable()->gyro_sync_denom =  1;          //  gyro 采样周期 8K， PID /4 = 2K
    pidConfigMutable()->pid_process_denom = 4;          // 使用4次数据滤波

    for (uint8_t pidProfileIndex = 0; pidProfileIndex < MAX_PROFILE_COUNT; pidProfileIndex++) {
        pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

        pidProfile->pid[PID_ROLL].P = 70;
        pidProfile->pid[PID_ROLL].I = 62;
        pidProfile->pid[PID_ROLL].D = 19;
        pidProfile->pid[PID_PITCH].P = 70;
        pidProfile->pid[PID_PITCH].I = 62;
        pidProfile->pid[PID_PITCH].D = 19;
        pidProfile->pid[PID_LEVEL].I = 40;
    }

    for (uint8_t rateProfileIndex = 0; rateProfileIndex < CONTROL_RATE_PROFILE_COUNT; rateProfileIndex++) {
        controlRateConfig_t *controlRateConfig = controlRateProfilesMutable(rateProfileIndex);

        controlRateConfig->rcRates[FD_ROLL] = 70;
        controlRateConfig->rcRates[FD_PITCH] = 70;
    }

}

#endif

#include <stdio.h>

// stack part
char _estack;
char _Min_Stack_Size;

// fake EEPROM
static FILE *eepromFd = NULL;
uint8_t eepromData[EEPROM_SIZE];

