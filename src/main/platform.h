/************************************************************************************************
* @file    platform.h
* @author  亓岳鑫
* @email   qiyuexin@yeah.net
* @Version V1.0
* @date    Apr. 11, 2018.
* @brief   None
************************************************************************************************/

#pragma once

#if defined(STM32F40_41xxx) || defined (STM32F427_437xx) 

#include "stm32f4xx.h"
#define STM32F4

#define ARM_MATH_CM4

// Chip Unique ID on F405
#define U_ID_0 (*(uint32_t*)0x1fff7a10)
#define U_ID_1 (*(uint32_t*)0x1fff7a14)
#define U_ID_2 (*(uint32_t*)0x1fff7a18)

#elif defined(STM32F10X_HD)

// 外部晶振具体值修改 HSE_VALUE，在下面头文件里。
#include "stm32f10x.h"  // 使用我修改后的头文件, 包含'target.h'

// Chip Unique ID on F103
#define U_ID_0 (*(uint32_t*)0x1FFFF7E8)
#define U_ID_1 (*(uint32_t*)0x1FFFF7EC)
#define U_ID_2 (*(uint32_t*)0x1FFFF7F0)

#define STM32F10X
#define STM32F1

#else // STM32F10X
#error "Invalid chipset specified. Update platform.h"
#endif


#ifdef USE_OSD_SLAVE
#include "target/common_osd_slave.h"
#include "target.h"
#else
#include "target/common_fc_pre.h"
#include "target.h"
#include "target/common_fc_post.h"
#endif


