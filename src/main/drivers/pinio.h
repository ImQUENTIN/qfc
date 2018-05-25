/************************************************************************************************
* @file    pinio.h
* @author  亓岳鑫
* @email   qiyuexin@yeah.net
* @Version V1.0
* @date    May. 23th, 2018.
* @brief   None
************************************************************************************************/


#pragma once

#include <stdbool.h>
#include "pg/pg.h"
#include "drivers/io_types.h"

#ifndef PINIO_COUNT
#define PINIO_COUNT 4
#endif

typedef struct pinioConfig_s {
    ioTag_t ioTag[PINIO_COUNT];
    uint8_t config[PINIO_COUNT];
} pinioConfig_t;

PG_DECLARE(pinioConfig_t, pinioConfig);

#define PINIO_CONFIG_OUT_INVERTED 0x80
#define PINIO_CONFIG_MODE_MASK    0x7F
#define PINIO_CONFIG_MODE_OUT_PP  0x01

struct pinioConfig_s;

void pinioInit(const struct pinioConfig_s *pinioConfig);
void pinioSet(int index, bool on);
