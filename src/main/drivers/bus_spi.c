/************************************************************************************************
* @file    bus_spi.c
* @author  亓岳鑫
* @email   qiyuexin@yeah.net
* @Version V1.0
* @date    May. 23th, 2018.
* @brief   None
************************************************************************************************/



#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <platform.h>

#ifdef USE_SPI

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/rcc.h"

spiDevice_t spiDevice[SPIDEV_COUNT];

SPIDevice spiDeviceByInstance(SPI_TypeDef *instance)
{
#ifdef USE_SPI_DEVICE_1
    if (instance == SPI1)
        return SPIDEV_1;
#endif

#ifdef USE_SPI_DEVICE_2
    if (instance == SPI2)
        return SPIDEV_2;
#endif

#ifdef USE_SPI_DEVICE_3
    if (instance == SPI3)
        return SPIDEV_3;
#endif

#ifdef USE_SPI_DEVICE_4
    if (instance == SPI4)
        return SPIDEV_4;
#endif

    return SPIINVALID;
}

SPI_TypeDef *spiInstanceByDevice(SPIDevice device)
{
    if (device >= SPIDEV_COUNT) {
        return NULL;
    }

    return spiDevice[device].dev;
}

bool spiInit(SPIDevice device)
{
    switch (device) {
    case SPIINVALID:
        return false;
    case SPIDEV_1:
#ifdef USE_SPI_DEVICE_1
        spiInitDevice(device);
        return true;
#else
        break;
#endif
    case SPIDEV_2:
#ifdef USE_SPI_DEVICE_2
        spiInitDevice(device);
        return true;
#else
        break;
#endif
    case SPIDEV_3:
#if defined(USE_SPI_DEVICE_3) && !defined(STM32F1)
        spiInitDevice(device);
        return true;
#else
        break;
#endif
    case SPIDEV_4:
#if defined(USE_SPI_DEVICE_4)
        spiInitDevice(device);
        return true;
#else
        break;
#endif
    }
    return false;
}

uint32_t spiTimeoutUserCallback(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID) {
        return -1;
    }
    spiDevice[device].errorCount++;
    return spiDevice[device].errorCount;
}

uint16_t spiGetErrorCounter(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID) {
        return 0;
    }
    return spiDevice[device].errorCount;
}

void spiResetErrorCounter(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device != SPIINVALID) {
        spiDevice[device].errorCount = 0;
    }
}


bool spiBusTransfer(const busDevice_t *bus, const uint8_t *txData, uint8_t *rxData, int length)
{
    IOLo(bus->busdev_u.spi.csnPin);
    spiTransfer(bus->busdev_u.spi.instance, txData, rxData, length);
    IOHi(bus->busdev_u.spi.csnPin);
    return true;
}

bool spiBusWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data)
{
    IOLo(bus->busdev_u.spi.csnPin);
    spiTransferByte(bus->busdev_u.spi.instance, reg);
    spiTransferByte(bus->busdev_u.spi.instance, data);
    IOHi(bus->busdev_u.spi.csnPin);

    return true;
}


uint8_t spiBusReadRegister(const busDevice_t *bus, uint8_t reg)
{
    uint8_t data;
    IOLo(bus->busdev_u.spi.csnPin);
    spiTransferByte(bus->busdev_u.spi.instance, reg | 0x80); // read transaction
    spiTransfer(bus->busdev_u.spi.instance, NULL, &data, 1);
    IOHi(bus->busdev_u.spi.csnPin);

    return data;
}

bool spiBusReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length)
{
    IOLo(bus->busdev_u.spi.csnPin);
    spiTransferByte(bus->busdev_u.spi.instance, reg | 0x80); // read transaction
    spiTransfer(bus->busdev_u.spi.instance, NULL, data, length);
    IOHi(bus->busdev_u.spi.csnPin);

    return true;
}


void spiBusSetInstance(busDevice_t *bus, SPI_TypeDef *instance)
{
    bus->bustype = BUSTYPE_SPI;
    bus->busdev_u.spi.instance = instance;
}
#endif
