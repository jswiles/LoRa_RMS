/*
 *******************************************************************************
 * Copyright (c) 2018, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */
#ifndef _VARIANT_ARDUINO_STM32_
#define _VARIANT_ARDUINO_STM32_

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/
#define PA0  0   // | 0       | A0             |                          |           |                        | WKUP1     |
#define PA1  1   // | 1       | A1             |                          |           |                        |           |
#define PA2  2   // | 2       | A2             | USART2_TX                |           |                        |           |
#define PA3  3   // | 2       | A3             | USART2_RX                |           |                        |           |
#define PA4  4   // | 4       | A4, DAC_OUT1   |                          |           | SPI1_SS/SPI3_NSS       |           |
#define PA5  5   // | 5       | A5, DAC_OUT2   |                          |           | SPI1_SCK               |           |
#define PA6  6   // | 6       | A6             |                          |           | SPI1_MISO              |           |
#define PA7  7   // | 7       | A7             |                          |           | SPI1_MOSI              |           |
#define PA8  8   // | 8       |                |                          |           |                        |           |
#define PA9  9   // | 9       |                | USART1_TX                |           |                        |           |
#define PA10 10  // | 10      |                | USART1_RX                |           |                        |           |
#define PA11 11  // | 11      |                |                          |           |                        | USB_DM    |
#define PA12 12  // | 12      |                |                          |           |                        | USB_DP    |
#define PA13 13  // | 13      |                |                          |           |                        | SWD_SWDIO |
#define PA14 14  // | 14      |                |                          |           |                        | SWD_SWCLK |
#define PA15 15  // | 15      |                |                          |           | SPI1_SS*/SPI3_SS**     |           |
//                  |---------|----------------|--------------------------|-----------|------------------------|-----------|
#define PB0  A8  // | 16      | A8             |                          |           |                        |           |
#define PB1  A9  // | 17      | A9             |                          |           |                        |           |
#define PB2  18  // | 18      |                |                          |           |                        | BOOT1     |
#define PB3  19  // | 19      |                |                          |           | SPI1_SCK*/SPI3_SCK**   |           |
#define PB4  20  // | 20      |                |                          |           | SPI1_MISO*/SPI3_MISO** |           |
#define PB5  21  // | 21      |                |                          |           | SPI1_MOSI*/SPI3_MOSI** |           |
#define PB6  22  // | 22      |                | USART1_TX                | TWI1_SCL  |                        |           |
#define PB7  23  // | 23      |                | USART1_RX                | TWI1_SDA  |                        |           |
#define PB8  24  // | 24      |                |                          | TWI1_SCL  |                        |           |
#define PB9  25  // | 25      |                |                          | TWI1_SDA  |                        |           |
#define PB10 26  // | 26      |                | USART3_TX                | TWI2_SCL  |                        |           |
#define PB11 27  // | 27      |                | USART3_RX                | TWI2_SDA  |                        |           |
#define PB12 28  // | 28      | A18            |                          |           | SPI2_SS                |           |
#define PB13 29  // | 29      | A19            |                          |           | SPI2_SCK               |           |
#define PB14 30  // | 30      | A20            |                          |           | SPI2_MISO              |           |
#define PB15 31  // | 31      | A21            |                          |           | SPI2_MOSI              |           |
//                  |---------|----------------|--------------------------|-----------|------------------------|-----------|
#define PC0  32  // | 32      |                |                          |           |                        |           |
#define PC1  33  // | 33      |                |                          |           |                        |           |
#define PC2  34  // | 34      |                |                          |           |                        |           |
#define PC3  35  // | 35      |                |                          |           |                        |           |
#define PC4  36  // | 36      |                |                          |           |                        |           |
#define PC5  37  // | 37      |                |                          |           |                        |           |
#define PC6  38  // | 38      |                |                          |           |                        |           |
#define PC7  39  // | 39      |                |                          |           |                        |           |
#define PC8  40  // | 40      |                |                          |           |                        |           |
#define PC9  41  // | 41      |                |                          |           |                        |           |
#define PC10 42  // | 42      |                |                          |           |                        |           |
#define PC11 43  // | 43      |                |                          |           |                        |           |
#define PC12 44  // | 44      |                |                          |           |                        |           |
#define PC13 45  // | 45      |                |                          |           |                        | WKUP2     |
#define PC14 46  // | 46      |                |                          |           |                        | OSC32_IN  |
#define PC15 47  // | 47      |                |                          |           |                        | OSC32_OUT |
//                  |---------|----------------|--------------------------|-----------|------------------------|-----------|


// This must be a literal
#define NUM_DIGITAL_PINS        48
// This must be a literal with a value less than or equal to to MAX_ANALOG_INPUTS
//#define NUM_ANALOG_INPUTS       14
#define NUM_ANALOG_INPUTS       4
// First Analog I/O pin number must start after digitial I/O
#define NUM_ANALOG_FIRST        6

// On-board LED pin number
#define LED_BUILTIN             PC13
#define LED_GREEN               LED_BUILTIN

// SPI Definitions
#define PIN_SPI_SS              PA15
#define PIN_SPI_MOSI            PB5
#define PIN_SPI_MISO            PB4
#define PIN_SPI_SCK             PB3

// I2C Definitions
#define PIN_WIRE_SDA            PB9
#define PIN_WIRE_SCL            PB8

// UART Definitions
#define SERIAL_UART_INSTANCE    1
// Default pin used for 'Serial' instance
// Mandatory for Firmata
#define PIN_SERIAL_RX           PB7
#define PIN_SERIAL_TX           PB6

#ifdef __cplusplus
} // extern "C"
#endif
/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
  // These serial port names are intended to allow libraries and architecture-neutral
  // sketches to automatically default to the correct port name for a particular type
  // of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
  // the first hardware serial port whose RX/TX pins are not dedicated to another use.
  //
  // SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
  //
  // SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
  //
  // SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
  //
  // SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
  //
  // SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
  //                            pins are NOT connected to anything by default.
  #define SERIAL_PORT_MONITOR     Serial
  #define SERIAL_PORT_HARDWARE    Serial

#endif

#endif /* _VARIANT_ARDUINO_STM32_ */
