/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
/*stm32 standard GPIO pin system works like this:
 * P [letter of alphabet] [pin number]
 * The letter represents the GPIO
 * The number represents the pin number of that GPIO
 * So for example: If I want pin PA9 for uart tx it would use the convention
 * GPIOA
 * Pin_9
 * GPIO_AF_[NUMBER 0-15] specifies the alternate function(s) a pin will be function as, if available
 */

#define TARGET_BOARD_IDENTIFIER "STM32F303RENUCLEO"

#define LED0_GPIO   GPIOA
#define LED0_PIN    Pin_5
#define LED0_PERIPHERAL RCC_AHBPeriph_GPIOA //defines GPIO port clock

#define BEEP_GPIO   GPIOA
#define BEEP_PIN    Pin_10
#define BEEP_PERIPHERAL RCC_AHBPeriph_GPIOA
#define BEEPER_INVERTED

#define USABLE_TIMER_CHANNEL_COUNT 17

#define EXTI_CALLBACK_HANDLER_COUNT 2 // MPU data ready and MAG data ready

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_MAG_DATA_READY_SIGNAL
#define ENSURE_MAG_DATA_READY_IS_HIGH


#define GYRO
#define USE_GYRO_MPU6050
#define GYRO_MPU6050_ALIGN CW270_DEG

#define ACC
#define USE_ACC_MPU6050
#define ACC_MPU6050_ALIGN CW270_DEG

#define BARO
#define USE_BARO_BMP085
#define USE_BARO_MS5611

#define MAG
#define USE_MAG_AK8975
#define USE_MAG_HMC5883
#define MAG_HMC5883_ALIGN CW270_DEG

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define SONAR
#define BEEPER
#define LED0

#define USE_USART1
#define USE_USART2
#define USE_USART3
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2
#define SERIAL_PORT_COUNT 5

#ifndef UART1_GPIO
#define UART1_TX_PIN        GPIO_Pin_9  // PA9
#define UART1_RX_PIN        GPIO_Pin_10 // PA10
#define UART1_GPIO          GPIOA
#define UART1_GPIO_AF       GPIO_AF_7
#define UART1_TX_PINSOURCE  GPIO_PinSource9
#define UART1_RX_PINSOURCE  GPIO_PinSource10
#endif
//by default USART2 is used by the ST-LINK Embedded debugger
#define UART2_TX_PIN        GPIO_Pin_2 // PA2
#define UART2_RX_PIN        GPIO_Pin_3 // PA3
#define UART2_GPIO          GPIOA
#define UART2_GPIO_AF       GPIO_AF_7
#define UART2_TX_PINSOURCE  GPIO_PinSource2
#define UART2_RX_PINSOURCE  GPIO_PinSource3

#ifndef UART3_GPIO
#define UART3_TX_PIN        GPIO_Pin_10 // PB10 (AF7)
#define UART3_RX_PIN        GPIO_Pin_11 // PB11 (AF7)
#define UART3_GPIO_AF       GPIO_AF_7
#define UART3_GPIO          GPIOC
#define UART3_TX_PINSOURCE  GPIO_PinSource10
#define UART3_RX_PINSOURCE  GPIO_PinSource11
#endif

#define SOFTSERIAL_1_TIMER TIM3
#define SOFTSERIAL_1_TIMER_RX_HARDWARE 4 // PWM 5
#define SOFTSERIAL_1_TIMER_TX_HARDWARE 5 // PWM 6
#define SOFTSERIAL_2_TIMER TIM3
#define SOFTSERIAL_2_TIMER_RX_HARDWARE 6 // PWM 7
#define SOFTSERIAL_2_TIMER_TX_HARDWARE 7 // PWM 8

#define USE_I2C
#define I2C_DEVICE (I2CDEV_1) // PB6/SCL, PB7/SDA

//multiple spi ports may be used, be careful as some pins may overlap other devices
#define USE_SPI
//#define USE_SPI_DEVICE_1 // default PA4, 5, 6, AND 7 on pin mode AF5. WARNING: THESE PINS ARE ONLY 3.3V TOLERANT
#define USE_SPI_DEVICE_2 //default PB12, 13, 14 AND 15 on pin mode AF5. WARNING: THESE PINS ARE ONLY 3.3V TOLERANT
//In bus_spi.c file cleanflight checks if the spi pins are defined or not, if not it will use the default pin sets
//Uncomment the following lines to use 5v tolerant spi pins for the respective spi device
/*
#define SPI1_GPIO               GPIOA
#define SPI1_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOA
#define SPI1_NSS_PIN            GPIO_Pin_15 //This overlaps the I2C1_SCL pin
#define SPI1_NSS_PIN_SOURCE     GPIO_PinSource15
#define SPI1_SCK_PIN            GPIO_Pin_3
#define SPI1_SCK_PIN_SOURCE     GPIO_PinSource3
#define SPI1_MISO_PIN           GPIO_Pin_4
#define SPI1_MISO_PIN_SOURCE    GPIO_PinSource4
#define SPI1_MOSI_PIN           GPIO_Pin_5
#define SPI1_MOSI_PIN_SOURCE    GPIO_PinSource5
*/
//5V pins on spi2 are on GPIOF and GPIOA, changes to the bus_spi.c code are required to enable the spi2 5v pins
//on two seperate GPIOs

//Uncomment the following lines to enable the M25P16 spi flash memory
//define M25P16_CS_GPIO          GPIOB //chip select pin, use any compatable pin
//#define M25P16_CS_PIN           GPIO_Pin_12
//#define M25P16_SPI_INSTANCE     SPI2 //spi1 or spi2

#define USE_ADC
#define BOARD_HAS_VOLTAGE_DIVIDER


#define ADC_INSTANCE                ADC2
#define ADC_DMA_CHANNEL             DMA2_Channel1
#define ADC_AHB_PERIPHERAL          RCC_AHBPeriph_DMA2

#define VBAT_ADC_GPIO               GPIOA
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_4
#define VBAT_ADC_CHANNEL            ADC_Channel_1

#define CURRENT_METER_ADC_GPIO      GPIOA
#define CURRENT_METER_ADC_GPIO_PIN  GPIO_Pin_5
#define CURRENT_METER_ADC_CHANNEL   ADC_Channel_2

#define RSSI_ADC_GPIO               GPIOB
#define RSSI_ADC_GPIO_PIN           GPIO_Pin_2
#define RSSI_ADC_CHANNEL            ADC_Channel_12

#define LED_STRIP
#define LED_STRIP_TIMER TIM1

#define USE_LED_STRIP_ON_DMA1_CHANNEL2
#define WS2811_GPIO                     GPIOA //ws2811 is an LED driver chip
#define WS2811_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOA
#define WS2811_GPIO_AF                  GPIO_AF_6
#define WS2811_PIN                      GPIO_Pin_8
#define WS2811_PIN_SOURCE               GPIO_PinSource8
#define WS2811_TIMER                    TIM1
#define WS2811_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM1
#define WS2811_DMA_CHANNEL              DMA1_Channel2
#define WS2811_IRQ                      DMA1_Channel2_IRQn

#define BLACKBOX
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define DISPLAY
#define GPS
#define GTUNE
#define SERIAL_RX
#define TELEMETRY
#define USE_SERVOS
#define USE_CLI

#define SPEKTRUM_BIND
// USART3,
#define BIND_PORT  GPIOB
#define BIND_PIN   Pin_11

#define USE_SERIAL_1WIRE

#define S1W_TX_GPIO         GPIOA
#define S1W_TX_PIN          GPIO_Pin_9
#define S1W_RX_GPIO         GPIOA
#define S1W_RX_PIN          GPIO_Pin_10
