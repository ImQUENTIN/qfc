/************************************************************************************************
* @file    main.c
* @author  亓岳鑫
* @email   qiyuexin@yeah.net
* @Version V2.0
* @date    May 24th, 2018.
* @brief   None
************************************************************************************************/


#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "fc/fc_init.h"

#include "scheduler/scheduler.h"

#include "bitops.h"
// 记得添加：
// mpuResetFn,  L27 @system_stm32f4.c
//     targetConfiguration();   L455,@config.c

int main(void)
{
    init();
    while (true) {
        scheduler();
        processLoopback();
#ifdef SIMULATOR_BUILD
        delayMicroseconds_real(50); // max rate 20kHz
#endif
    }
    return 0;
}
