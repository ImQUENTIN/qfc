/************************************************************************************************
* @file    sound_beeper.c
* @author  亓岳鑫
* @email   qiyuexin@yeah.net
* @Version V2.0
* @date    May 24th, 2018.
* @brief   None
************************************************************************************************/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/io.h"
#include "drivers/pwm_output.h"

#include "pg/beeper_dev.h"

#include "sound_beeper.h"

#ifdef BEEPER
static IO_t beeperIO = DEFIO_IO(NONE);
static bool beeperInverted = false;
static uint16_t beeperFrequency = 0;
#endif

void systemBeep(bool onoff)
{
#ifdef BEEPER

    if (beeperFrequency == 0) {
        IOWrite(beeperIO, beeperInverted ? onoff : !onoff);
    } 
#ifndef QFC
    else{
        pwmWriteBeeper(onoff);
    }
#endif
#else
    UNUSED(onoff);
#endif
}

void systemBeepToggle(void)
{
#ifdef BEEPER
    if (beeperFrequency == 0) {
        IOToggle(beeperIO);
    } 
#ifndef QFC
    else    {
        pwmToggleBeeper();
    }
#endif
#endif
}

void beeperInit(const beeperDevConfig_t *config)
{
#ifdef BEEPER
    beeperFrequency = config->frequency;
    if (beeperFrequency == 0) {
        beeperIO = IOGetByTag(config->ioTag);
        beeperInverted = config->isInverted;
        if (beeperIO) {
            IOInit(beeperIO, OWNER_BEEPER, 0);
            IOConfigGPIO(beeperIO, config->isOpenDrain ? IOCFG_OUT_OD : IOCFG_OUT_PP);
        }
        systemBeep(false);
    } 
#ifndef QFC
    else {
        const ioTag_t beeperTag = config->ioTag;
        beeperPwmInit(beeperTag, beeperFrequency);
    }
#endif
#else
    UNUSED(config);
#endif
}
