/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2022 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */
 
 #define snprintf_P(s, f, ...) snprintf((s), (f), __VA_ARGS__)
#define vsnprintf_P(s, n, f, ...) vsnprintf((s), (n), (f), __VA_ARGS__)

// redefine 8 bit types of inttypes.h (as of SAMD board defs 1.8.1)
#undef PRId8
#undef PRIi8
#undef PRIo8
#undef PRIu8
#undef PRIx8
#undef PRIX8
#undef PRIdLEAST8
#undef PRIiLEAST8
#undef PRIoLEAST8
#undef PRIuLEAST8
#undef PRIxLEAST8
#undef PRIXLEAST8
#undef PRIdFAST8
#undef PRIiFAST8
#undef PRIoFAST8
#undef PRIuFAST8
#undef PRIxFAST8
#undef PRIXFAST8
#define PRId8		"d"
#define PRIi8		"i"
#define PRIo8		"o"
#define PRIu8		"u"
#define PRIx8		"x"
#define PRIX8		"X"
#define PRIdLEAST8	"d"
#define PRIiLEAST8	"i"
#define PRIoLEAST8	"o"
#define PRIuLEAST8	"u"
#define PRIxLEAST8	"x"
#define PRIXLEAST8	"X"
#define PRIdFAST8	"d"
#define PRIiFAST8	"i"
#define PRIoFAST8	"o"
#define PRIuFAST8	"u"
#define PRIxFAST8	"x"
#define PRIXFAST8	"X"


 
 #define MCU_STM32F103C8

#ifndef MyHwSTM32F1_h
#define MyHwSTM32F1_h
#include <itoa.h>
#include <EEPROM.h>
#include <SPI.h>

#ifdef __cplusplus
#include <Arduino.h>
#endif

#define CRYPTO_LITTLE_ENDIAN

#ifndef MY_SERIALDEVICE
#define MY_SERIALDEVICE Serial
#endif

#ifndef MY_DEBUGDEVICE
#define MY_DEBUGDEVICE MY_SERIALDEVICE
#endif

#ifndef MY_STM32F1_TEMPERATURE_OFFSET
#define MY_STM32F1_TEMPERATURE_OFFSET (0.0f)
#endif

#ifndef MY_STM32F1_TEMPERATURE_GAIN
#define MY_STM32F1_TEMPERATURE_GAIN (1.0f)
#endif

// SS default
#ifndef SS
#define SS PA4
#endif

// mapping
#define vsnprintf_P vsnprintf
//#define strncpy_P strncpy
#define printf_P printf
#define yield()				  // not defined

#ifndef digitalPinToInterrupt
#define digitalPinToInterrupt(__pin) (__pin)
#endif

void iwdg_feed(void) {
 
}

#define hwDigitalWrite(__pin, __value) digitalWrite(__pin, __value)
#define hwDigitalRead(__pin) digitalRead(__pin)
#define hwPinMode(__pin, __value) pinMode(__pin, __value)
#define hwWatchdogReset() iwdg_feed()
#define hwReboot() iwdg_feed()
#define hwMillis() millis()
#define hwGetSleepRemaining() (0ul)

extern void serialEventRun(void) __attribute__((weak));
bool hwInit(void);
void hwRandomNumberInit(void);
void hwReadConfigBlock(void *buf, void *addr, size_t length);
void hwWriteConfigBlock(void *buf, void *addr, size_t length);
void hwWriteConfig(const int addr, uint8_t value);
uint8_t hwReadConfig(const int addr);

// SOFTSPI
#ifdef MY_SOFTSPI
#error Soft SPI is not available on this architecture!
#endif
#define hwSPI SPI //!< hwSPI


#ifndef DOXYGEN
#define MY_CRITICAL_SECTION
#endif  /* DOXYGEN */

#endif
