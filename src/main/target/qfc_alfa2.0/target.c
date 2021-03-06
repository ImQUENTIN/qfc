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
    DEF_TIM(TIM3, CH1, PB4,  TIM_USE_PWM,   0, 0), // S1_IN, PITCH
    DEF_TIM(TIM3, CH2, PB5,  TIM_USE_PWM,   0, 0), // S2_IN, R0LL
    DEF_TIM(TIM3, CH4, PB1,  TIM_USE_PWM,   0, 0), // S3_IN, THRO
    DEF_TIM(TIM3, CH3, PB0,  TIM_USE_PWM,   0, 0), // S4_IN, YAW    

    DEF_TIM(TIM4, CH4, PD15, TIM_USE_PWM,   0, 0), // S5_IN
    DEF_TIM(TIM4, CH3, PD14, TIM_USE_PWM,   0, 0), // S6_IN
    DEF_TIM(TIM4, CH2, PD13, TIM_USE_PWM,   0, 0), // S7_IN
    DEF_TIM(TIM4, CH1, PD12, TIM_USE_PWM,   0, 0), // S8_IN
    
    DEF_TIM(TIM1, CH1, PE9,  TIM_USE_MOTOR, 0, 0), // S5_OUT
    DEF_TIM(TIM1, CH2, PE11, TIM_USE_MOTOR, 0, 0), // S6_OUT
    DEF_TIM(TIM1, CH3, PE13, TIM_USE_MOTOR, 0, 0), // S7_OUT
    DEF_TIM(TIM1, CH4, PE14, TIM_USE_MOTOR, 0, 0), // S8_OUT
    
    DEF_TIM(TIM2, CH1, PA15, TIM_USE_BEEPER, 0, 0), // beeper
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


#define VBAT_SCALE 100

// alternative defaults settings for MULTIFLITEPICO targets
void targetConfiguration(void)
{

    compassConfigMutable()->mag_hardware = MAG_NONE;            // disabled by default

    voltageSensorADCConfigMutable(VOLTAGE_SENSOR_ADC_VBAT)->vbatscale = VBAT_SCALE;
    voltageSensorADCConfigMutable(VOLTAGE_SENSOR_ADC_VBAT)->vbatresdivval = 15;
    voltageSensorADCConfigMutable(VOLTAGE_SENSOR_ADC_VBAT)->vbatresdivmultiplier = 4;
    batteryConfigMutable()->vbatmaxcellvoltage = 44;
    batteryConfigMutable()->vbatmincellvoltage = 32;
    batteryConfigMutable()->vbatwarningcellvoltage = 33;

    rxConfigMutable()->spektrum_sat_bind = 5;
    rxConfigMutable()->spektrum_sat_bind_autoreset = 1;

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

    motorConfigMutable()->dev.motorPwmRate = 17000;

    gyroConfigMutable()->gyro_sync_denom = 4;
    pidConfigMutable()->pid_process_denom = 1;

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

