/************************************************************************************************
* @file    pg\bus_spi.h
* @author  亓岳鑫
* @email   qiyuexin@yeah.net
* @Version V2.0
* @date    May 24th, 2018.
* @brief   None
************************************************************************************************/

#pragma once

#include "drivers/bus_spi.h"
#include "drivers/io_types.h"

#include "pg/pg.h"

typedef struct spiPinConfig_s {
    ioTag_t ioTagSck[SPIDEV_COUNT];
    ioTag_t ioTagMiso[SPIDEV_COUNT];
    ioTag_t ioTagMosi[SPIDEV_COUNT];
} spiPinConfig_t;


PG_DECLARE(spiPinConfig_t, spiPinConfig);
