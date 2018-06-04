/************************************************************************************************
* @file    accgyro_spi_adis16405.c
* @author  亓岳鑫
* @email   qiyuexin@yeah.net
* @Version V2.0
* @date    May 24th, 2018.
* @brief   None
************************************************************************************************/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_GYRO_SPI_ADIS16405) || defined(USE_ACC_SPI_ADIS16405)

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"

#include "drivers/accgyro/accgyro_spi_adis16405.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/sensor.h"
#include "drivers/system.h"


// User Register Memory Map from Table 6
#define ADIS_FLASH_CNT 0x00  //Flash memory write count
#define ADIS_SUPPLY_OUT 0x02
#define ADIS_XGYRO_OUT 0x04  //X-axis gyroscope output
#define ADIS_YGYRO_OUT 0x06  //Y-axis gyroscope output
#define ADIS_ZGYRO_OUT 0x08  //Z-axis gyroscope output
#define ADIS_XACCL_OUT 0x0A  //X-axis accelerometer output
#define ADIS_YACCL_OUT 0x0C  //Y-axis accelerometer output
#define ADIS_ZACCL_OUT 0x0E  //Z-axis accelerometer output
#define ADIS_XMAGN_OUT 0X10  //X-axis magnetometer output
#define ADIS_YMAGN_OUT 0x12  //Y-axis magnetometer output
#define ADIS_ZMAGN_OUT 0x14  //Z-axis magnetometer output
#define ADIS_TEMP_OUT  0x16   //Temperature output
#define ADIS_AUX_OUT   0x18   //aux adc: Barometer pressure measurement, high word

#define ADIS_XGYRO_OFF 0x1A  //X-axis gyroscope bias offset factor
#define ADIS_YGYRO_OFF 0x1C  //Y-axis gyroscope bias offset factor
#define ADIS_ZGYRO_OFF 0x1E  //Z-axis gyroscope bias offset factor
#define ADIS_XACCL_OFF 0x20  //X-axis acceleration bias offset factor
#define ADIS_YACCL_OFF 0x22  //Y-axis acceleration bias offset factor
#define ADIS_ZACCL_OFF 0x24  //Z-axis acceleration bias offset factor
#define ADIS_XMAGN_HIC 0x26  //X-axis magnetometer, hard iron factor
#define ADIS_YMAGN_HIC 0x28  //Y-axis magnetometer, hard iron factor
#define ADIS_ZMAGN_HIC 0x2A  //Z-axis magnetometer, hard iron factor
#define ADIS_XMAGN_SIC 0x2C  //X-axis magnetometer, soft iron factor
#define ADIS_YMAGN_SIC 0x2E  //Y-axis magnetometer, soft iron factor
#define ADIS_ZMAGN_SIC 0x30  //Z-axis magnetometer, soft iron factor
#define ADIS_GPIO_CTRL 0x32  //GPIO control
#define ADIS_MSC_CTRL 0x34   //MISC control
#define ADIS_SMPL_PRD 0x36   //Sample clock/Decimation filter control
#define ADIS_SENS_AVG 0x38   //Digital filter control
#define ADIS_SEQ_CNT 0x3A    //MAGN_OUT and BARO_OUT counter
#define ADIS_DIAG_STAT 0x3C  //System status
#define ADIS_GLOB_CMD 0x3E   //System command
#define ADIS_ALM_MAG1 0x40   //Alarm 1 amplitude threshold
#define ADIS_ALM_MAG2 0x42   //Alarm 2 amplitude threshold
#define ADIS_ALM_SMPL1 0x44  //Alarm 1 sample size
#define ADIS_ALM_SMPL2 0x46  //Alarm 2 sample size
#define ADIS_ALM_CTRL 0x48   //Alarm control
#define ADIS_LOT_ID1 0x52    //Lot identification number
#define ADIS_LOT_ID2 0x54    //Lot identification number
#define ADIS_PROD_ID 0x56    //Product identifier
#define ADIS_SERIAL_NUM 0x58 //Lot-specific serial number


// ADIS16405, 16bit 传输
uint16_t adisRegRead(const busDevice_t *bus, uint8_t regAddr) 
{
    static uint8_t dat[2];
    spiBusWriteRegister(bus, regAddr, 0x00);
    delayMicroseconds(9);   // normal mode, at least wait 9us; in burst mode, Tstall = 1/1mhz = 1us;
    spiBusTransfer(bus, NULL, dat, 2); 
    return((uint16_t)(dat[0] << 8 | dat[1]));
}

void adisRegWrite(const busDevice_t *bus, uint8_t regAddr, uint8_t value) 
{
    static uint8_t dat[2];
    spiBusWriteRegister(bus, regAddr | 0x01, value);
    delayMicroseconds(9);   // normal mode, at least wait 9us; in burst mode, Tstall = 1/1mhz = 1us;
}

#define ADIS_BURST_LEN  (13 * 2 + 1)

bool adisBurstRead(const busDevice_t *bus)
{
    static uint8_t data[ADIS_BURST_LEN];
    
    IOLo(bus->busdev_u.spi.csnPin);    
    spiTransferByte(bus->busdev_u.spi.instance, 0x3e);
    spiTransferByte(bus->busdev_u.spi.instance, 0x00);
    spiTransfer(bus->busdev_u.spi.instance, NULL, data, ADIS_BURST_LEN);    
    IOHi(bus->busdev_u.spi.csnPin);

        
    return true;
}

#undef ADIS_BURST_LEN


// 设备检测
uint8_t adis16405SpiDetect(const busDevice_t *bus)
{
    static uint16_t whoAmI;
    
    IOInit(bus->busdev_u.spi.csnPin, OWNER_MPU_CS, 0);
    IOConfigGPIO(bus->busdev_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(bus->busdev_u.spi.csnPin);

    spiSetDivisor(bus->busdev_u.spi.instance, SPI_CLOCK_INITIALIZATON);

    spiBusWriteRegister(bus, ADIS_GLOB_CMD, 0x80);  // software reset
    delay(1);
    uint8_t attemptsRemaining = 5;
    do {
        delay(150);
        whoAmI = adisRegRead(bus, ADIS_PROD_ID);
        if (whoAmI == 0x4015) {
            break;
        }
        if (!attemptsRemaining) {
            return MPU_NONE;
        }
    } while (attemptsRemaining--);

//    adisBurstRead(bus);
    // SCK <= 2Mhz
    spiSetDivisor(bus->busdev_u.spi.instance, SPI_CLOCK_SLOW);
    return ADIS_16405_SPI;

}

static bool adis16405InitDone = false;


static void adis16405AccAndGyroInit(gyroDev_t *gyro)
{
    if (adis16405InitDone) {
        return;
    }

    spiSetDivisor(gyro->bus.busdev_u.spi.instance, SPI_CLOCK_INITIALIZATON);

    // software reset
    spiBusWriteRegister(&gyro->bus, ADIS_GLOB_CMD, 0x80);  
    delay(50);
    
       static uint16_t test;
    test = adisRegRead(&gyro->bus, ADIS_SENS_AVG);
    // Number of taps in each stage, N = 2^M = 4
    spiBusWriteRegister(&gyro->bus, ADIS_SENS_AVG, 0x02);
    delayMicroseconds(20);    

    // Gyro +/- 300 DPS Full Scale
    spiBusWriteRegister(&gyro->bus, ADIS_SENS_AVG + 1, 0x04);
    delayMicroseconds(20);   
    // fixed: Accel +/- 18g Full Scale
    // fixed: mag +/- 18g Full Scale

    // sample rate = 819.2 SPS
    spiBusWriteRegister(&gyro->bus, ADIS_SMPL_PRD, 0x01);
    delayMicroseconds(20);
    
#ifdef USE_MPU_DATA_READY_SIGNAL
     // data ready enbale, data ready polarity = high, use DIO1.
    spiBusWriteRegister(&gyro->bus, ADIS_MSC_CTRL, 0x06); 
    delayMicroseconds(20);
#endif

    const uint16_t test1 = adisRegRead(&gyro->bus, ADIS_SENS_AVG);
    const uint16_t test2 = adisRegRead(&gyro->bus, ADIS_SENS_AVG|1);

    spiSetDivisor(gyro->bus.busdev_u.spi.instance, SPI_CLOCK_SLOW);
    delayMicroseconds(1);

    adis16405InitDone = true;
}


#define DATA_14_BIT     0x4000
#define DATA_12_BIT     0x1000

static inline int16_t adisDataFormat(uint16_t valid, uint16_t sensorData)
{
    int signedData = 0;
    uint16_t mask = valid - 1;
    if(sensorData & (valid >> 1)){
        // negetive
        sensorData = valid - (sensorData & mask);
        signedData = -sensorData;
    }
    else
        signedData = (sensorData & mask);

    return signedData;

}



bool adisGyroReadSPI(gyroDev_t *gyro)
{
#if 1
    int16_t adc[3];
    adc[X] = adisRegRead(&gyro->bus, ADIS_XGYRO_OUT);
    adc[Y] = adisRegRead(&gyro->bus, ADIS_YGYRO_OUT);
    adc[Z] = adisRegRead(&gyro->bus, ADIS_ZGYRO_OUT);            

    gyro->gyroADCRaw[X] = adisDataFormat(DATA_14_BIT, adc[X]);
    gyro->gyroADCRaw[Y] = adisDataFormat(DATA_14_BIT, adc[Y]);
    gyro->gyroADCRaw[Z] = adisDataFormat(DATA_14_BIT, adc[Z]);

//      float finalData = signedData * 0.05f; // degrees/sec
    return true;
#else
    uint8_t data[7];

    const bool ack = busReadRegisterBuffer(&gyro->bus, MPU_RA_GYRO_XOUT_H, data, 7);
    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[X] = (int16_t)((data[1] << 8) | data[2]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[3] << 8) | data[4]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[5] << 8) | data[6]);

#endif

}


bool adisAccReadSPI(accDev_t *acc)
{
    int16_t adc[3];
    adc[X] = adisRegRead(&acc->bus, ADIS_XACCL_OUT);
    adc[Y] = adisRegRead(&acc->bus, ADIS_YACCL_OUT);
    adc[Z] = adisRegRead(&acc->bus, ADIS_ZACCL_OUT);            

    acc->ADCRaw[X] = adisDataFormat(DATA_14_BIT, adc[X]);
    acc->ADCRaw[Y] = adisDataFormat(DATA_14_BIT, adc[Y]);
    acc->ADCRaw[Z] = adisDataFormat(DATA_14_BIT, adc[Z]);

//    float finalData = signedData * 0.00333f;      // mg
    return true;
}

static bool adisMagReadSPI(magDev_t *mag, int16_t *magData)
{
    int16_t adc[3];
    adc[X] = adisRegRead(&mag->busdev, ADIS_XMAGN_OUT);
    adc[Y] = adisRegRead(&mag->busdev, ADIS_YMAGN_OUT);
    adc[Z] = adisRegRead(&mag->busdev, ADIS_ZMAGN_OUT);            

    magData[X] = adisDataFormat(DATA_14_BIT, adc[X]);
    magData[Z] = adisDataFormat(DATA_14_BIT, adc[Y]);
    magData[Y] = adisDataFormat(DATA_14_BIT, adc[Z]);
    
    //  float finalData = signedData * 0.5f;      // mgauss
    return true;
}

void adis16405SpiGyroInit(gyroDev_t *gyro)
{
    mpuGyroInit(gyro);

    adis16405AccAndGyroInit(gyro);
    
    spiResetErrorCounter(gyro->bus.busdev_u.spi.instance);

//    spiSetDivisor(gyro->bus.busdev_u.spi.instance, SPI_CLOCK_INITIALIZATON);

    // Accel and Gyro DLPF Setting, lpf is ??
//    spiBusWriteRegister(&gyro->bus, MPU6000_CONFIG, gyro->lpf);
//    delayMicroseconds(1);

//    spiSetDivisor(gyro->bus.busdev_u.spi.instance, SPI_CLOCK_FAST);  // 18 MHz SPI clock

    adisGyroReadSPI(gyro);

    if (((int8_t)gyro->gyroADCRaw[1]) == -1 && ((int8_t)gyro->gyroADCRaw[0]) == -1) {
        failureMode(FAILURE_GYRO_INIT_FAILED);
    }
}

void adis16405SpiAccInit(accDev_t *acc)
{
    acc->acc_1G = 300;  

}

void adis16405SpiMagInit(magDev_t* mag)
{
    //  float finalData = signedData * 0.5f;      // mgauss
}

// gyro 检测
bool adis16405SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != ADIS_16405_SPI) {
        return false;
    }

    gyro->initFn = adis16405SpiGyroInit;
    gyro->readFn = adisGyroReadSPI;
    
    // 0.05 dps/lsb scalefactor, dps: degress per seconds
    gyro->scale = 0.05f;

    return true;
}

// acc 检测
bool adis16405SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != ADIS_16405_SPI) {
        return false;
    }

    acc->initFn = adis16405SpiAccInit;
    acc->readFn = adisAccReadSPI;

    return true;
}


bool adis16405SPiMagDetect(magDev_t* mag)
{
    if(adis16405InitDone != true)
        return false;
    
    mag->init = adis16405SpiMagInit;
    mag->read = adisMagReadSPI;

    return true;
}

// imu reset
void adis16405SpiResetGyro(void)
{

}

#endif
