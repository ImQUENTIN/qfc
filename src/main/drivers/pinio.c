/************************************************************************************************
* @file    pinio.c
* @author  亓岳鑫
* @email   qiyuexin@yeah.net
* @Version V1.0
* @date    May. 23th, 2018.
* @brief   None
************************************************************************************************/



#include <stdint.h>

#include <platform.h>

#ifdef USE_PINIO

#include "build/debug.h"
#include "drivers/io.h"
#include "pg/pg_ids.h"
#include "pinio.h"
#include "drivers/io.h"

#ifndef PINIO1_PIN
#define PINIO1_PIN NONE
#endif
#ifndef PINIO2_PIN
#define PINIO2_PIN NONE
#endif
#ifndef PINIO3_PIN
#define PINIO3_PIN NONE
#endif
#ifndef PINIO4_PIN
#define PINIO4_PIN NONE
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(pinioConfig_t, pinioConfig, PG_PINIO_CONFIG, 0);
PG_RESET_TEMPLATE(pinioConfig_t, pinioConfig,
    .ioTag = {
        IO_TAG(PINIO1_PIN),
        IO_TAG(PINIO2_PIN),
        IO_TAG(PINIO3_PIN),
        IO_TAG(PINIO4_PIN),
    },
    .config = {
        PINIO_CONFIG_MODE_OUT_PP,
        PINIO_CONFIG_MODE_OUT_PP,
        PINIO_CONFIG_MODE_OUT_PP,
        PINIO_CONFIG_MODE_OUT_PP
    },
);

typedef struct pinioRuntime_s {
    IO_t io;
    bool inverted;
    bool state;
} pinioRuntime_t;
static pinioRuntime_t pinioRuntime[PINIO_COUNT];

void pinioInit(const pinioConfig_t *pinioConfig)
{
    for (int i = 0; i < PINIO_COUNT; i++) {
        IO_t io = IOGetByTag(pinioConfig->ioTag[i]);

        if (!io) {
            continue;
        }

        IOInit(io, OWNER_PINIO, RESOURCE_INDEX(i));

        switch (pinioConfig->config[i] & PINIO_CONFIG_MODE_MASK) {
        case PINIO_CONFIG_MODE_OUT_PP:
            IOConfigGPIO(io, IOCFG_OUT_PP);
            break;
        }

        if (pinioConfig->config[i] & PINIO_CONFIG_OUT_INVERTED)
        {
            pinioRuntime[i].inverted = true;
            IOHi(io);
        } else {
            pinioRuntime[i].inverted = false;
            IOLo(io);
        }
        pinioRuntime[i].io = io;
        pinioRuntime[i].state = false;
    }
}

void pinioSet(int index, bool on)
{
    bool newState = on ^ pinioRuntime[index].inverted;
    if (newState != pinioRuntime[index].state) {
        IOWrite(pinioRuntime[index].io, newState);
        pinioRuntime[index].state = newState;
    }
}
#endif
