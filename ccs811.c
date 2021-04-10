/*
  ccs811.cpp - Library for the CCS811 digital gas sensor for monitoring indoor air quality from ams.
*/

#include "ccs811.h"
#include <string.h>
#include "transfer_handler.h"

uint16_t ccs811_appversion;  // Version of the app firmware inside the CCS811 (for workarounds)

// Timings
#define CCS811_WAIT_AFTER_RESET_US     2000 // The CCS811 needs a wait after reset
#define CCS811_WAIT_AFTER_APPSTART_US  1000 // The CCS811 needs a wait after app start
#define CCS811_WAIT_AFTER_WAKE_US        50 // The CCS811 needs a wait after WAKE signal
#define CCS811_WAIT_AFTER_APPERASE_MS   500 // The CCS811 needs a wait after app erase (300ms from spec not enough)
#define CCS811_WAIT_AFTER_APPVERIFY_MS   70 // The CCS811 needs a wait after app verify
#define CCS811_WAIT_AFTER_APPDATA_MS     50 // The CCS811 needs a wait after writing app data

// Main interface =====================================================================================================

// CCS811 registers/mailboxes, all 1 byte except when stated otherwise
#define CCS811_STATUS           0x00
#define CCS811_MEAS_MODE        0x01
#define CCS811_ALG_RESULT_DATA  0x02 // up to 8 bytes
#define CCS811_RAW_DATA         0x03 // 2 bytes
#define CCS811_ENV_DATA         0x05 // 4 bytes
#define CCS811_THRESHOLDS       0x10 // 5 bytes
#define CCS811_BASELINE         0x11 // 2 bytes
#define CCS811_HW_ID            0x20
#define CCS811_HW_VERSION       0x21
#define CCS811_FW_BOOT_VERSION  0x23 // 2 bytes
#define CCS811_FW_APP_VERSION   0x24 // 2 bytes
#define CCS811_ERROR_ID         0xE0
#define CCS811_APP_ERASE        0xF1 // 4 bytes
#define CCS811_APP_DATA         0xF2 // 9 bytes
#define CCS811_APP_VERIFY       0xF3 // 0 bytes
#define CCS811_APP_START        0xF4 // 0 bytes
#define CCS811_SW_RESET         0xFF // 4 bytes

#define MAX_RW_LENGTH           8

// The flags for errstat in ccs811_read()
// ERRSTAT is a merge of two hardware registers: ERROR_ID (bits 15-8) and STATUS (bits 7-0)
// Also bit 1 (which is always 0 in hardware) is set to 1 when an I2C read error occurs
#define CCS811_ERRSTAT_ERROR 0x0001             // There is an error, the ERROR_ID register (0xE0) contains the error source
#define CCS811_ERRSTAT_I2CFAIL 0x0002           // Bit flag added by software: I2C transaction error
#define CCS811_ERRSTAT_DATA_READY 0x0008        // A new data sample is ready in ALG_RESULT_DATA
#define CCS811_ERRSTAT_APP_VALID 0x0010         // Valid application firmware loaded
#define CCS811_ERRSTAT_FW_MODE 0x0080           // Firmware is in application mode (not boot mode)
#define CCS811_ERRSTAT_WRITE_REG_INVALID 0x0100 // The CCS811 received an I²C write request addressed to this station but with invalid register address ID
#define CCS811_ERRSTAT_READ_REG_INVALID 0x0200  // The CCS811 received an I²C read request to a mailbox ID that is invalid
#define CCS811_ERRSTAT_MEASMODE_INVALID 0x0400  // The CCS811 received an I²C request to write an unsupported mode to MEAS_MODE
#define CCS811_ERRSTAT_MAX_RESISTANCE 0x0800    // The sensor resistance measurement has reached or exceeded the maximum range
#define CCS811_ERRSTAT_HEATER_FAULT 0x1000      // The heater current in the CCS811 is not in range
#define CCS811_ERRSTAT_HEATER_SUPPLY 0x2000     // The heater voltage is not being applied correctly

// These flags should not be set. They flag errors.
#define CCS811_ERRSTAT_HWERRORS (CCS811_ERRSTAT_ERROR | CCS811_ERRSTAT_WRITE_REG_INVALID | CCS811_ERRSTAT_READ_REG_INVALID | CCS811_ERRSTAT_MEASMODE_INVALID | CCS811_ERRSTAT_MAX_RESISTANCE | CCS811_ERRSTAT_HEATER_FAULT | CCS811_ERRSTAT_HEATER_SUPPLY)
#define CCS811_ERRSTAT_ERRORS (CCS811_ERRSTAT_I2CFAIL | CCS811_ERRSTAT_HWERRORS)
// These flags should normally be set - after a measurement. They flag data available (and valid app running).
#define CCS811_ERRSTAT_OK (CCS811_ERRSTAT_DATA_READY | CCS811_ERRSTAT_APP_VALID | CCS811_ERRSTAT_FW_MODE)
// These flags could be set after a measurement. They flag data is not yet available (and valid app running).
#define CCS811_ERRSTAT_OK_NODATA (CCS811_ERRSTAT_APP_VALID | CCS811_ERRSTAT_FW_MODE)


static void ccs811_write(uint8_t reg, int count, const uint8_t * buf) {

		uint8_t sendbuf[MAX_RW_LENGTH + 1] = {reg};
		memcpy(sendbuf + 1, buf, count > MAX_RW_LENGTH ? 0 : count);
		iic_send(CCS811_ADDR, sendbuf, count + 1, false);
	
}

static void ccs811_read(uint8_t reg, int count, uint8_t * buf) {
	
		iic_send(CCS811_ADDR, &reg, 1, true);
		iic_read(CCS811_ADDR, buf, count);
		
}

static void ccs811_trig(uint8_t reg){

		iic_send(CCS811_ADDR, &reg, 1, false);

}

void ccs811_reset(void){
	
		uint8_t sw_reset_pattern[]= {0x11, 0xE5, 0x72, 0x8A};
		ccs811_write(CCS811_SW_RESET, 4, sw_reset_pattern);
		delayMicroseconds(CCS811_WAIT_AFTER_RESET_US);

}

uint8_t ccs811_readHWID(void){
		
		uint8_t ret;
		ccs811_read(CCS811_HW_ID, 1, &ret);
		return ret;

}

uint8_t ccs811_readHWVersion(void){
		
		uint8_t ret;
		ccs811_read(CCS811_HW_VERSION, 1, &ret);
		return ret;

}

uint16_t ccs811_readFWBootVersion(void){
		
		uint8_t ret[2];
		ccs811_read(CCS811_FW_BOOT_VERSION, 2, ret);
		return (ret[0] << 8) | ret[1];

}

uint16_t ccs811_readFWAppVersion(void){
		
		uint8_t ret[2];
		ccs811_read(CCS811_FW_APP_VERSION, 2, ret);
		return (ret[0] << 8) | ret[1];

}

uint8_t ccs811_readStatus(void){
		
		uint8_t ret;
		ccs811_read(CCS811_STATUS, 1, &ret);
		return ret;

}


bool ccs811_begin( void ) {
  
    // Invoke a SW reset (bring CCS811 in a know state)
    ccs811_reset();
	
		Debug("CCS811 HW ID(0x81): 0x%2x", ccs811_readHWID());
		Debug("CCS811 HW_VERSION(0x1X): 0x%2x", ccs811_readHWVersion());
		Debug("CCS811 FW_BOOT_VERSION: 0x%4x", ccs811_readFWBootVersion());
	
		ccs811_appversion = ccs811_readFWAppVersion();
		Debug("CCS811 FW_APP_VERSION: 0x%4x", ccs811_appversion);

		// Switch CCS811 from boot mode into app mode
		ccs811_trig(CCS811_APP_START);
		delayMicroseconds(CCS811_WAIT_AFTER_APPSTART_US);
	
		if( ccs811_readStatus() != 0x90 ) {
			
				Debug("CCS811 Not in app mode, or no valid app");
				return false;
			
    }

		return true;

}


// Switch CCS811 to `mode`, use constants CCS811_MODE_XXX. Returns false on problems.
void ccs811_start( uint8_t mode ) {
	
		ccs811_write(CCS811_MEAS_MODE, 1, &mode);
	
}

void ccs811_getRawData(uint8_t *current, uint16_t *adcVal){
		
		uint8_t ret[2];
		ccs811_read(CCS811_RAW_DATA, 2, ret);
		
		*current = ret[0] >> 2;
		*adcVal = ((ret[0] << 8) & 0x30) | ret[1];
	
}

// Returns a string version of an errstat. Note, each call, this string is updated.
static const char * CCS811_errstat_str(uint16_t errstat) {
  static char s[17]; // 16 bits plus terminating zero
  // First the ERROR_ID flags
                                                  s[ 0]='-';
                                                  s[ 1]='-';
  if( errstat & CCS811_ERRSTAT_HEATER_SUPPLY    ) s[ 2]='V'; else s[2]='v';
  if( errstat & CCS811_ERRSTAT_HEATER_FAULT     ) s[ 3]='H'; else s[3]='h';
  if( errstat & CCS811_ERRSTAT_MAX_RESISTANCE   ) s[ 4]='X'; else s[4]='x';
  if( errstat & CCS811_ERRSTAT_MEASMODE_INVALID ) s[ 5]='M'; else s[5]='m';
  if( errstat & CCS811_ERRSTAT_READ_REG_INVALID ) s[ 6]='R'; else s[6]='r';
  if( errstat & CCS811_ERRSTAT_WRITE_REG_INVALID) s[ 7]='W'; else s[7]='w';
  // Then the STATUS flags
  if( errstat & CCS811_ERRSTAT_FW_MODE          ) s[ 8]='F'; else s[8]='f';
                                                  s[ 9]='-';
                                                  s[10]='-';
  if( errstat & CCS811_ERRSTAT_APP_VALID        ) s[11]='A'; else s[11]='a';
  if( errstat & CCS811_ERRSTAT_DATA_READY       ) s[12]='D'; else s[12]='d';
                                                  s[13]='-';
  // Next bit is used by SW to signal I2C transfer error
  if( errstat & CCS811_ERRSTAT_I2CFAIL          ) s[14]='I'; else s[14]='i';
  if( errstat & CCS811_ERRSTAT_ERROR            ) s[15]='E'; else s[15]='e';
                                                  s[16]='\0';
  return s;
}

bool ccs811_getData(uint16_t *eco2, uint16_t *tvoc){
		
		uint8_t ret[6];
		ccs811_read(CCS811_ALG_RESULT_DATA, 5, ret);
		
		*eco2 = (ret[0] << 8) | ret[1];
		*tvoc = (ret[2] << 8) | ret[3];
	
		if(ret[4] & 0x01) {  		//ERROR
				
				ccs811_read(CCS811_ERROR_ID, 1, &ret[5]);
				Debug("CCS811 Error :%s", CCS811_errstat_str((ret[5] << 8) | ret[4]));
		
		}
	
		return ret[4] & 0x08;  //DATA_READY
	
}

uint16_t ccs811_getBaseline(){
	
		uint8_t ret[2];
		ccs811_read(CCS811_BASELINE, 2, ret);
		return (ret[0] << 8) | ret[1];

}

void ccs811_setBaseline(uint16_t baseline){
	
		uint8_t set[2] = { baseline >> 8, baseline};
		ccs811_write(CCS811_BASELINE, 2, set);

}

void ccs811_setEnvironment(float humidity, float temperature){
	
		uint16_t humidity_f512 = humidity * 512.0f;
		uint16_t temperature_f512 = (temperature + 25.0f) * 512.0f;
		uint8_t set[4] = { humidity_f512 >> 8, humidity_f512, temperature_f512 >> 8, temperature_f512};
		ccs811_write(CCS811_ENV_DATA, 4, set);

}
