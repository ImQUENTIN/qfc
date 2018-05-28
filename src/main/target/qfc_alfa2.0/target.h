/****************************************************************************
* @file    target.h
* @author  亓岳鑫
* @email   qiyuexin@yeah.net
* @Version V1.0
* @date    Apr. 11, 2018.
* @brief   qfc_alfa2.0
****************************************************************************/

#pragma once

#define TARGET_BOARD_IDENTIFIER "QFC_F4"
#define USBD_PRODUCT_STRING "QUENTIN'S FLIGHT F4"

#define USE_TARGET_CONFIG
#define USE_QUAD_MIXER_ONLY         // 4旋翼

// file name to save config
#define EEPROM_FILENAME "eeprom.bin"
#define EEPROM_IN_RAM
#define EEPROM_SIZE     32768



#define USE_RTC_TIME
#define QFC

#define DEFAULT_FEATURES (FEATURE_MOTOR_STOP )     // default featuresf

/************************************************************************************************
* @note    adc	@2018/5/28
************************************************************************************************/

#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE	CURRENT_METER_ADC


/************************************************************************************************
* @note    GPIO
************************************************************************************************/

#define LED0_PIN                PD4
#define LED1_PIN                PD7


#define BEEPER                  PA15    // 使用beeper
#define BEEPER_INVERTED                 // 高电平发声



/************************************************************************************************
* @note    SPI
************************************************************************************************/
#define USE_SPI
#define USE_SPI_DEVICE_1        // default pins: PA5~7
#define USE_SPI_DEVICE_2        // default pins: PB13~15
#define USE_SPI_DEVICE_4        // fram?? mpu9250
#define SPI4_SCK_PIN            PE2
#define SPI4_MISO_PIN           PE5
#define SPI4_MOSI_PIN           PE6


#define USE_ACC
#define USE_FAKE_ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW270_DEG
#define USE_ACC_SPI_MPU9250


#define USE_GYRO
#define USE_FAKE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW270_DEG
#define USE_GYRO_SPI_MPU9250

#define MPU6000_CS_PIN          PD11
#define MPU6000_SPI_INSTANCE    SPI1

#define MPU9250_CS_PIN          PC13    // GPIO_1
#define MPU9250_SPI_INSTANCE    SPI2

// MPU interrupt, 没有使用Ready信号
//#define USE_EXTI
//#define MPU_INT_EXTI            PC14
//#define DEBUG_MPU_DATA_READY_INTERRUPT
//#define USE_MPU_DATA_READY_SIGNAL
//#define ENSURE_MPU_DATA_READY_IS_LOW


#define USE_MAG
#define USE_FAKE_MAG
//#define USE_MAG_AK8963
//#define USE_MAG_AK8975
//#define USE_MAG_HMC5883
//#define USE_MAG_QMC5883
//
//#define MAG_AK8963_ALIGN        CW180_DEG_FLIP

#define USE_BARO
#define USE_FAKE_BARO
//#define USE_BARO_BMP085
//#define USE_BARO_BMP280
//#define USE_BARO_MS5611

//#define USE_MAX7456
//#define MAX7456_SPI_INSTANCE    SPI2
//#define MAX7456_SPI_CS_PIN      SPI2_NSS_PIN


//#define USE_CMS         // ??? what's this


/************************************************************************************************
* @note    i2C
************************************************************************************************/


#define USE_I2C

#define USE_I2C_DEVICE_1
#define I2C2_SCL                PB8
#define I2C2_SDA                PB9

#define USE_I2C_DEVICE_2
#define I2C3_SCL                PB10
#define I2C3_SDA                PB11
#define I2C_DEVICE              (I2CDEV_2)  // EXT

/************************************************************************************************
* @note    sdcard
************************************************************************************************/


//#define USE_SDCARD
//
//#define SDCARD_SPI_INSTANCE     SPI2
//#define SDCARD_SPI_CS_PIN       PB12
//// SPI2 is on the APB1 bus whose clock runs at 36MHz. Divide to under 400kHz for init:
//#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 128
//// Divide to under 25MHz for normal operation:
//#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER 2
//
//// Note, this is the same DMA channel as UART1_RX. Luckily we don't use DMA for USART Rx.
//#define SDCARD_DMA_CHANNEL_TX               DMA1_Channel5

// Performance logging for SD card operations:
// #define AFATFS_USE_INTROSPECTIVE_LOGGING



//#define USE_SDCARD
#if 0
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN               PB11

#define SDCARD_SPI_INSTANCE             SPI2
#define SDCARD_SPI_CS_PIN               PB10

// SPI2 is on the APB1 bus whose clock runs at 84MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER 4 // 21MHz

#define SDCARD_DMA_CHANNEL_TX               DMA1_Stream4
#define SDCARD_DMA_CHANNEL                  0
#endif

/************************************************************************************************
* @note    uart
************************************************************************************************/

//#define USE_VCP

#define USE_UART1
#define UART1_TX_PIN            PB6
#define UART1_RX_PIN            PB7

#define USE_UART2
#define UART2_TX_PIN            PD5
#define UART2_RX_PIN            PD6

#define USE_UART3
#define UART3_TX_PIN            PD8
#define UART3_RX_PIN            PD9

#define USE_UART4
#define UART4_TX_PIN            PA0
#define UART4_RX_PIN            PA1

#define SERIAL_PORT_COUNT       5



#define USE_MSP_UART                            // UART1, 使用DMA
#define USE_UART1_RX_DMA
#define USE_UART1_TX_DMA

//#define SERIALRC_UART                       SERIAL_PORT_USART2
//#define SERIALRC_PROVIDER                   SERIALRC_SBUS
//#define RX_CHANNELS_TAER
//#define DEFAULT_RC_FEATURE                  FEATURE_RC_SERIAL
//#define AVOID_UART2_FOR_PWM_PPM

#define GPS_UART                            SERIAL_PORT_USART3
//#define TELEMETRY_UART                      SERIAL_PORT_UART4
//#define TELEMETRY_PROVIDER_DEFAULT          FUNCTION_TELEMETRY_MAVLINK
//#define SBUS_TELEMETRY_UART                 SERIAL_PORT_UART5
//

/************************************************************************************************
* @note    ADC
************************************************************************************************/
#define USE_ADC
#define ADC_INSTANCE            ADC1
    //#define ADC_INSTANCE            ADC2
#define VBAT_ADC_PIN            PC0
#define CURRENT_METER_ADC_PIN   PC1
#define RSSI_ADC_PIN            PC2
#define EXTERNAL1_ADC_PIN       PC3


/************************************************************************************************
* @note    SONAR
************************************************************************************************/
#define USE_SONAR
#define SONAR_TRIGGER_PIN                   PA4
#define SONAR_ECHO_PIN                      PA2

// 测距仪
#define USE_RANGEFINDER
#define USE_RANGEFINDER_HCSR04
#define RANGEFINDER_HCSR04_TRIGGER_PIN       PA4
#define RANGEFINDER_HCSR04_ECHO_PIN          PA2

/************************************************************************************************
* @note    ESC
************************************************************************************************/
    
//#define USE_ESC_SENSOR
//    



/************************************************************************************************
* @note    Timer
************************************************************************************************/



#define MAX_SUPPORTED_MOTORS    12

#define TARGET_IO_PORTA (0xffff & ~(BIT(8) | BIT(10) | BIT(13) | BIT(14)))
#define TARGET_IO_PORTB (0xffff & ~(BIT(2) | BIT(3)  | BIT(12)))
#define TARGET_IO_PORTC (0xffff )
#define TARGET_IO_PORTD (0xffff )
#define TARGET_IO_PORTE (0xffff & ~(BIT(3) | BIT(4)  | BIT(7)  | BIT(8) | \
                                    BIT(10)| BIT(12) | BIT(15)))

#define USABLE_TIMER_CHANNEL_COUNT  15
#define USED_TIMERS                 (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) )

