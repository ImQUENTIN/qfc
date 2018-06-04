/************************************************************************************************
* @file    adis16405.h
* @author  亓岳鑫
* @email   qiyuexin@yeah.net
* @Version V2.0
* @date    May 24th, 2018.
* @brief   None
************************************************************************************************/
#pragma once

#include "drivers/bus.h"
#include "drivers/compass/compass.h"
//#define MPU6000_CONFIG              0x1A
//
//#define BITS_DLPF_CFG_256HZ         0x00
//#define BITS_DLPF_CFG_188HZ         0x01
//#define BITS_DLPF_CFG_98HZ          0x02
//#define BITS_DLPF_CFG_42HZ          0x03

#define GYRO_SCALE_FACTOR  0.00053292f  // (4/131) * pi/180   (32.75 LSB = 1 DPS)

// RF = Register Flag
#define MPU_RF_DATA_RDY_EN (1 << 0)

uint8_t adis16405SpiDetect(const busDevice_t *bus);

bool adis16405SpiAccDetect(accDev_t *acc);
bool adis16405SpiGyroDetect(gyroDev_t *gyro);
bool adis16405SPiMagDetect(magDev_t* mag);

void adis16405SpiResetGyro(void);




