/*
  ccs811.h - Library for the CCS811 digital gas sensor for monitoring indoor air quality from ams.
*/
#ifndef _CCS8111_H_
#define _CCS8111_H_

#include <stdint.h>
#include <stdbool.h>

// I2C slave address for ADDR 0 respectively 1
#define CCS811_ADDR 0x5A

// The values for mode in ccs811_start()
#define CCS811_MODE_IDLE 				0x00
#define CCS811_MODE_1SEC 				0x10
#define CCS811_MODE_10SEC 			0x20
#define CCS811_MODE_60SEC 			0x30
#define CCS811_MODE_CONTINOUS 	0x40

bool ccs811_begin(void);                                                             // Reset the CCS811, switch to app mode and check HW_ID. Returns false on problems.
void ccs811_start(uint8_t mode);                                                         // Switched CCS811 to `mode`, use constants CCS811_MODE_XXX. Returns false on I2C problems.
void ccs811_getRawData(uint8_t *current, uint16_t *adcVal);
bool ccs811_getData(uint16_t *eco2, uint16_t *tvoc);
uint16_t ccs811_getBaseline(void);
void ccs811_setBaseline(uint16_t baseline);
void ccs811_setEnvironment(float humidity, float temperature);

#endif
