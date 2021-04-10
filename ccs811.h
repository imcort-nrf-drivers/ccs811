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


//void CCS811_read(uint16_t *eco2, uint16_t *etvoc, uint16_t *errstat, uint16_t *raw); // Get measurement results from the CCS811 (all args may be NULL), check status via errstat, e.g. ccs811_errstat(errstat)

//const char *CCS811_errstat_str(uint16_t errstat);                                    // Returns a string version of an errstat. Note, each call, this string is updated.

//int CCS811_hardware_version(void);                  // Gets version of the CCS811 hardware (returns -1 on I2C failure)
//int CCS811_bootloader_version(void);                // Gets version of the CCS811 bootloader (returns -1 on I2C failure)
//int CCS811_application_version(void);               // Gets version of the CCS811 application (returns -1 on I2C failure)
//int CCS811_get_errorid(void);                       // Gets the ERROR_ID [same as 'err' part of 'errstat' in 'read'] (returns -1 on I2C failure)
//bool CCS811_set_envdata(uint16_t t, uint16_t h);    // Writes t and h to ENV_DATA (see datasheet for format). Returns false on I2C problems.
//bool CCS811_set_envdata210(uint16_t t, uint16_t h); // Writes t and h (in ENS210 format) to ENV_DATA. Returns false on I2C problems.
//bool CCS811_get_baseline(uint16_t *baseline);       // Reads (encoded) baseline from BASELINE. Returns false on I2C problems. Get it, just before power down (but only when sensor was on at least 20min) - see CCS811_AN000370
//bool CCS811_set_baseline(uint16_t baseline);        // Writes (encoded) baseline to BASELINE. Returns false on I2C problems. Set it, after power up (and after 20min)
//bool CCS811_flash(const uint8_t *image, int size);  // Flashes the firmware of the CCS811 with size bytes from image - image _must_ be in PROGMEM

#endif
