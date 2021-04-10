/*
  ccs811.cpp - Library for the CCS811 digital gas sensor for monitoring indoor air quality from ams.
  2019 jan 22  v10  Maarten Pennings  Added F() on all strings, added get/set_baseline()
  2019 jan 15   v9  Maarten Pennings  Function set_envdata did not write T; flash now uses PROGMEM array; flash now works without app, better PRINT macros, removed warnings
  2018 dec 06   v8  Maarten Pennings  Added firmware flash routine
  2018 Dec 04   v7  Maarten Pennings  Added support for older CCS811's (fw 1100)
  2018 Nov 11   v6  Maarten Pennings  uint16 -> uint16_t, added cast
  2018 Nov 02   v5  Maarten Pennings  Added clearing of ERROR_ID
  2018 Oct 23   v4  Maarten Pennings  Added envdata/i2cdelay
  2018 Oct 21   v3  Maarten Pennings  Fixed bug in begin(), added hw-version
  2018 Oct 21   v2  Maarten Pennings  Simplified I2C, added version mngt
  2017 Dec 11   v1  Maarten Pennings  Created
*/


#include "ccs811.h"
#include <string.h>
#include "transfer_handler.h"


int CCS811_appversion;  // Version of the app firmware inside the CCS811 (for workarounds)

// begin() and flash() prints errors to help diagnose startup problems.
// Change these macro's to empty to suppress those prints.
//#define PRINTLN(s)    Serial.println(s)
//#define PRINT(s)      Serial.print(s)
//#define PRINTLN2(s,m) Serial.println(s,m)
//#define PRINT2(s,m)   Serial.print(s,m)

//#define F(s)   				(s)
//#define PRINTLN(s)
//#define PRINT(s)
//#define PRINTLN2(s,m)
//#define PRINT2(s,m)


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

static void ccs811_write(uint8_t reg, int count, const uint8_t * buf) {

		uint8_t sendbuf[MAX_RW_LENGTH + 1] = {reg};
		memcpy(sendbuf + 1, buf, count > MAX_RW_LENGTH ? 0 : count);
		iic_send(CCS811_ADDR, sendbuf, count + 1, false);
	
}

static void ccs811_read(uint8_t reg, int count, uint8_t * buf) {
////  Wire.beginTransmission(_slaveaddr);              // START, SLAVEADDR
////  Wire.write(regaddr);                             // Register address
////  int wres= Wire.endTransmission(false);           // Repeated START
////  delayMicroseconds(_i2cdelay_us);                 // Wait
////  int rres=Wire.requestFrom(_slaveaddr,count);     // From CCS811, read bytes, STOP
////  for( int i=0; i<count; i++ ) buf[i]=Wire.read();
////  return (wres==0) && (rres==count);

//	ret_code_t err_code;

//	err_code = nrf_drv_twi_tx(&m_twi, CCS811_ADDR, &regaddr, 1, true);
//	APP_ERROR_CHECK(err_code);
//	
//	nrf_delay_ms(20);
//	
//	err_code = nrf_drv_twi_rx(&m_twi, CCS811_ADDR, buf, count);
//	APP_ERROR_CHECK(err_code);
//	nrf_delay_ms(20);
//	return true;
	
		iic_send(CCS811_ADDR, &reg, 1, true);
		iic_read(CCS811_ADDR, buf, count);
		
}

void ccs811_reset(void){
	
		uint8_t sw_reset_pattern[]= {0x11, 0xE5, 0x72, 0x8A};
		ccs811_write(CCS811_SW_RESET, 4, sw_reset_pattern);
		delayMicroseconds(CCS811_WAIT_AFTER_RESET_US);

}

uint8_t ccs811_readID(void){
		
		uint8_t ret;
		ccs811_read(CCS811_HW_ID, 1, &ret);
		return ret;

}

uint8_t ccs811_readVersion(void){
		
		uint8_t ret;
		ccs811_read(CCS811_HW_VERSION, 1, &ret);
		return ret;

}

uint8_t ccs811_readStatus(void){
		
		uint8_t ret;
		ccs811_read(CCS811_STATUS, 1, &ret);
		return ret;

}

uint16_t ccs811_readAppversion(void){
		
		uint8_t ret[2];
		ccs811_read(CCS811_STATUS, 2, ret);
		return (ret[0] << 8) | ret[1];

}


// Reset the CCS811, switch to app mode and check HW_ID. Returns false on problems.
bool CCS811_begin( void ) {
  
//  uint8_t app_start[]= {0x00};
//  
//  uint8_t hw_version;
//  uint8_t app_version[2];
//  uint8_t status;
//  bool ok;

//  CCS811_wake_init();
//  // Wakeup CCS811
//  CCS811_wake_up();

//    // Try to ping CCS811 (can we reach CCS811 via I2C?)
//    ok= i2cwrite(0,0,0);
//    if( !ok ) ok= i2cwrite(0,0,0); // retry
//    if( !ok ) {
//      // Try the other slave address
//      _slaveaddr= CCS811_SLAVEADDR_0 + CCS811_SLAVEADDR_1 - _slaveaddr; // swap address
//      ok= i2cwrite(0,0,0);
//      _slaveaddr= CCS811_SLAVEADDR_0 + CCS811_SLAVEADDR_1 - _slaveaddr; // swap back
//      if( ok ) {
//        PRINTLN(F("ccs811: wrong slave address, ping successful on other address"));
//      } else {
//        PRINTLN(F("ccs811: ping failed (VDD/GND connected? SDA/SCL connected?)"));
//      }
//      goto abort_begin;
//    }

    // Invoke a SW reset (bring CCS811 in a know state)
    ccs811_reset();

//    // Check that HW_ID is 0x81
//    ok= CCS811_i2cread(CCS811_HW_ID,1,&hw_id);
////    if( !ok ) {
////      PRINTLN(F("ccs811: HW_ID read failed"));
////      goto abort_begin;
////    }
//    if( hw_id!=0x81 ) {
////      PRINT(F("ccs811: Wrong HW_ID: "));
////      PRINTLN2(hw_id,HEX);
//			NRF_LOG_INFO("ccs811: Wrong HW_ID: %x", hw_id);
//      goto abort_begin;
//    }

//    // Check that HW_VERSION is 0x1X
//    ok= CCS811_i2cread(CCS811_HW_VERSION,1,&hw_version);
////    if( !ok ) {
////      PRINTLN(F("ccs811: HW_VERSION read failed"));
////      goto abort_begin;
////    }
//    if( (hw_version&0xF0)!=0x10 ) {
////      PRINT(F("ccs811: Wrong HW_VERSION: "));
////      PRINTLN2(hw_version,HEX);
//			NRF_LOG_INFO("ccs811: Wrong HW_VERSION: %x", hw_version);
//      goto abort_begin;
//    }

    // Check status (after reset, CCS811 should be in boot mode with valid app)
//    ok= CCS811_i2cread(CCS811_STATUS,1,&status);
//    if( !ok ) {
//      PRINTLN(F("ccs811: STATUS read (boot mode) failed"));
//      goto abort_begin;
//    }
		uint8_t status = ccs811_readStatus();
    if( status != 0x10 ) {
			Debug("ccs811: Not in boot mode, or no valid app: %x", status);
      return false;
    }

    // Read the application version
		uint8_t app_version[2];
    ok= CCS811_i2cread(CCS811_FW_APP_VERSION,2,app_version);
//    if( !ok ) {
//      PRINTLN(F("ccs811: APP_VERSION read failed"));
//      goto abort_begin;
//    }
    CCS811_appversion= app_version[0]*256+app_version[1];

    // Switch CCS811 from boot mode into app mode
    ok= CCS811_i2cwrite(CCS811_APP_START,0,app_start);
//    if( !ok ) {
//      PRINTLN(F("ccs811: Goto app mode failed"));
//      goto abort_begin;
//    }
    nrf_delay_us(CCS811_WAIT_AFTER_APPSTART_US);

    // Check if the switch was successful
    ok= CCS811_i2cread(CCS811_STATUS,1,&status);
//    if( !ok ) {
//      PRINTLN(F("ccs811: STATUS read (app mode) failed"));
//      goto abort_begin;
//    }
    if( status!=0x90 ) {
//      PRINT(F("ccs811: Not in app mode, or no valid app: "));
//      PRINTLN2(status,HEX);
			NRF_LOG_INFO("ccs811: Not in app mode, or no valid app: %x", status);
      goto abort_begin;
    }

  // CCS811 back to sleep
  CCS811_wake_down();
  // Return success
  return true;

abort_begin:
  // CCS811 back to sleep
  CCS811_wake_down();
  // Return failure
  return false;
}


// Switch CCS811 to `mode`, use constants CCS811_MODE_XXX. Returns false on problems.
bool CCS811_start( int mode ) {
  uint8_t meas_mode[]= {(uint8_t)(mode<<4)};
  CCS811_wake_up();
  bool ok = CCS811_i2cwrite(CCS811_MEAS_MODE,1,meas_mode);
  CCS811_wake_down();
  return ok;
}


// Get measurement results from the CCS811, check status via errstat, e.g. ccs811_errstat(errstat)
void CCS811_read( uint16_t*eco2, uint16_t*etvoc, uint16_t*errstat,uint16_t*raw) {
  bool    ok;
  uint8_t buf[8];
  uint8_t stat;
  CCS811_wake_up();
    if( CCS811_appversion<0x2000 ) {
      ok= CCS811_i2cread(CCS811_STATUS,1,&stat); // CCS811 with pre 2.0.0 firmware has wrong STATUS in CCS811_ALG_RESULT_DATA
      if( ok && stat==CCS811_ERRSTAT_OK ) ok= CCS811_i2cread(CCS811_ALG_RESULT_DATA,8,buf); else buf[5]=0;
      buf[4]= stat; // Update STATUS field with correct STATUS
    } else {
      ok = CCS811_i2cread(CCS811_ALG_RESULT_DATA,8,buf);
    }
  CCS811_wake_down();
  // Status and error management
  uint16_t combined = buf[5]*256+buf[4];
  if( combined & ~(CCS811_ERRSTAT_HWERRORS|CCS811_ERRSTAT_OK) ) ok= false; // Unused bits are 1: I2C transfer error
  combined &= CCS811_ERRSTAT_HWERRORS|CCS811_ERRSTAT_OK; // Clear all unused bits
  if( !ok ) combined |= CCS811_ERRSTAT_I2CFAIL;
  // Clear ERROR_ID if flags are set
  if( combined & CCS811_ERRSTAT_HWERRORS ) {
      int err = CCS811_get_errorid();
      if( err==-1 ) combined |= CCS811_ERRSTAT_I2CFAIL; // Propagate I2C error
  }
  // Outputs
  if( eco2   ) *eco2   = buf[0]*256+buf[1];
  if( etvoc  ) *etvoc  = buf[2]*256+buf[3];
  if( errstat) *errstat= combined;
  if( raw    ) *raw    = buf[6]*256+buf[7];;
}


// Returns a string version of an errstat. Note, each call, this string is updated.
const char * CCS811_errstat_str(uint16_t errstat) {
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


// Extra interface ========================================================================================


// Gets version of the CCS811 hardware (returns 0 on I2C failure)
int CCS811_hardware_version(void) {
  uint8_t buf[1];
  CCS811_wake_up();
  bool ok = CCS811_i2cread(CCS811_HW_VERSION,1,buf);
  CCS811_wake_down();
  int version= -1;
  if( ok ) version= buf[0];
  return version;
}


// Gets version of the CCS811 boot loader (returns 0 on I2C failure)
int CCS811_bootloader_version(void) {
  uint8_t buf[2];
  CCS811_wake_up();
  bool ok = CCS811_i2cread(CCS811_FW_BOOT_VERSION,2,buf);
  CCS811_wake_down();
  int version= -1;
  if( ok ) version= buf[0]*256+buf[1];
  return version;
}


// Gets version of the CCS811 application (returns 0 on I2C failure)
int CCS811_application_version(void) {
  uint8_t buf[2];
  CCS811_wake_up();
  bool ok = CCS811_i2cread(CCS811_FW_APP_VERSION,2,buf);
  CCS811_wake_down();
  int version= -1;
  if( ok ) version= buf[0]*256+buf[1];
  return version;
}


// Gets the ERROR_ID [same as 'err' part of 'errstat' in 'read'] (returns -1 on I2C failure)
// Note, this actually clears CCS811_ERROR_ID (hardware feature)
int CCS811_get_errorid(void) {
  uint8_t buf[1];
  CCS811_wake_up();
  bool ok = CCS811_i2cread(CCS811_ERROR_ID,1,buf);
  CCS811_wake_down();
  int version= -1;
  if( ok ) version= buf[0];
  return version;
}


#define HI(u16) ( (uint8_t)( ((u16)>>8)&0xFF ) )
#define LO(u16) ( (uint8_t)( ((u16)>>0)&0xFF ) )


// Writes t and h to ENV_DATA (see datasheet for format). Returns false on I2C problems.
bool CCS811_set_envdata(uint16_t t, uint16_t h) {
  uint8_t envdata[]= { HI(h), LO(h), HI(t), LO(t) };
  CCS811_wake_up();
  // Serial.print(" [T="); Serial.print(t); Serial.print(" H="); Serial.print(h); Serial.println("] ");
  bool ok = CCS811_i2cwrite(CCS811_ENV_DATA,4,envdata);
  CCS811_wake_down();
  return ok;
}


// Writes t and h (in ENS210 format) to ENV_DATA. Returns false on I2C problems.
bool CCS811_set_envdata210(uint16_t t, uint16_t h) {
  // Humidity formats of ENS210 and CCS811 are equal, we only need to map temperature.
  // The lowest and highest (raw) ENS210 temperature values the CCS811 can handle
  uint16_t lo= 15882; // (273.15-25)*64 = 15881.6 (float to int error is 0.4)
  uint16_t hi= 24073; // 65535/8+lo = 24073.875 (24074 would map to 65539, so overflow)
  // Check if ENS210 temperature is within CCS811 range, if not clip, if so map
  bool ok;
  if( t<lo )      ok= CCS811_set_envdata(0,h);
  else if( t>hi ) ok= CCS811_set_envdata(65535,h);
  else            ok= CCS811_set_envdata( (t-lo)*8+3 , h); // error in 'lo' is 0.4; times 8 is 3.2; so we correct 3
  // Returns I2C transaction status
  return ok;
}


// Reads (encoded) baseline from BASELINE (see datasheet). Returns false on I2C problems.
bool CCS811_get_baseline(uint16_t *baseline) {
  uint8_t buf[2];
  CCS811_wake_up();
  bool ok = CCS811_i2cread(CCS811_BASELINE,2,buf);
  CCS811_wake_down();
  *baseline= (buf[0]<<8) + buf[1];
  return ok;
}


// Writes (encoded) baseline to BASELINE (see datasheet). Returns false on I2C problems.
bool CCS811_set_baseline(uint16_t baseline) {
  uint8_t buf[]= { HI(baseline), LO(baseline) };
  CCS811_wake_up();
  bool ok = CCS811_i2cwrite(CCS811_BASELINE,2,buf);
  CCS811_wake_down();
  return ok;
}


// Flashes the firmware of the CCS811 with size bytes from image - image _must_ be in PROGMEM
//bool CCS811_flash(const uint8_t * image, int size) {
//  uint8_t sw_reset[]=   {0x11,0xE5,0x72,0x8A};
//  uint8_t app_erase[]=  {0xE7,0xA7,0xE6,0x09};
//  uint8_t app_verify[]= {};
//  uint8_t status;
//  int count;
//  bool ok;
//  wake_up();

//    // Try to ping CCS811 (can we reach CCS811 via I2C?)
//    PRINT(F("ccs811: ping "));
//    ok= i2cwrite(0,0,0);
//    if( !ok ) {
//      PRINTLN(F("FAILED"));
//      goto abort_begin;
//    }
//    PRINTLN(F("ok"));

//    // Invoke a SW reset (bring CCS811 in a know state)
//    PRINT(F("ccs811: reset "));
//    ok= i2cwrite(CCS811_SW_RESET,4,sw_reset);
//    if( !ok ) {
//      PRINTLN(F("FAILED"));
//      goto abort_begin;
//    }
//    delayMicroseconds(CCS811_WAIT_AFTER_RESET_US);
//    PRINTLN(F("ok"));

//    // Check status (after reset, CCS811 should be in boot mode with or without valid app)
//    PRINT(F("ccs811: status (reset1) "));
//    ok= i2cread(CCS811_STATUS,1,&status);
//    if( !ok ) {
//      PRINTLN(F("FAILED"));
//      goto abort_begin;
//    }
//    PRINT2(status,HEX);
//    PRINT(F(" "));
//    if( status!=0x00 && status!=0x10 ) {
//      PRINTLN(F("ERROR - ignoring")); // Seems to happens when there is no valid app
//    } else {
//      PRINTLN(F("ok"));
//    }

//    // Invoke app erase
//    PRINT(F("ccs811: app-erase "));
//    ok= i2cwrite(CCS811_APP_ERASE,4,app_erase);
//    if( !ok ) {
//      PRINTLN(F("FAILED"));
//      goto abort_begin;
//    }
//    delay(CCS811_WAIT_AFTER_APPERASE_MS);
//    PRINTLN(F("ok"));

//    // Check status (CCS811 should be in boot mode without valid app, with erase completed)
//    PRINT(F("ccs811: status (app-erase) "));
//    ok= i2cread(CCS811_STATUS,1,&status);
//    if( !ok ) {
//      PRINTLN(F("FAILED"));
//      goto abort_begin;
//    }
//    PRINT2(status,HEX);
//    PRINT(F(" "));
//    if( status!=0x40 ) {
//      PRINTLN(F("ERROR"));
//      goto abort_begin;
//    }
//    PRINTLN(F("ok"));

//    // Write all blocks
//    count= 0;
//    while( size>0 ) {
//        if( count%64==0 ) { PRINT(F("ccs811: writing ")); PRINT(size); PRINT(F(" ")); }
//        int len= size<8 ? size : 8;
//        // Copy PROGMEM to RAM
//        uint8_t ram[8];
//        memcpy_P(ram, image, len);
//        // Send 8 bytes from RAM to CCS811
//        ok= i2cwrite(CCS811_APP_DATA,len, ram);
//        if( !ok ) {
//          PRINTLN(F("ccs811: app data failed"));
//          goto abort_begin;
//        }
//        PRINT(F("."));
//        delay(CCS811_WAIT_AFTER_APPDATA_MS);
//        image+= len;
//        size-= len;
//        count++;
//        if( count%64==0 ) { PRINT(F(" ")); PRINTLN(size); }
//    }
//    if( count%64!=0 ) { PRINT(F(" ")); PRINTLN(size); }

//    // Invoke app verify
//    PRINT(F("ccs811: app-verify "));
//    ok= i2cwrite(CCS811_APP_VERIFY,0,app_verify);
//    if( !ok ) {
//      PRINTLN(F("FAILED"));
//      goto abort_begin;
//    }
//    delay(CCS811_WAIT_AFTER_APPVERIFY_MS);
//    PRINTLN(F("ok"));

//    // Check status (CCS811 should be in boot mode with valid app, and erased and verified)
//    PRINT(F("ccs811: status (app-verify) "));
//    ok= i2cread(CCS811_STATUS,1,&status);
//    if( !ok ) {
//      PRINTLN(F("FAILED"));
//      goto abort_begin;
//    }
//    PRINT2(status,HEX);
//    PRINT(F(" "));
//    if( status!=0x30 ) {
//      PRINTLN(F("ERROR"));
//      goto abort_begin;
//    }
//    PRINTLN(F("ok"));

//    // Invoke a second SW reset (clear flashing flags)
//    PRINT(F("ccs811: reset2 "));
//    ok= i2cwrite(CCS811_SW_RESET,4,sw_reset);
//    if( !ok ) {
//      PRINTLN(F("FAILED"));
//      goto abort_begin;
//    }
//    delayMicroseconds(CCS811_WAIT_AFTER_RESET_US);
//    PRINTLN(F("ok"));

//    // Check status (after reset, CCS811 should be in boot mode with valid app)
//    PRINT(F("ccs811: status (reset2) "));
//    ok= i2cread(CCS811_STATUS,1,&status);
//    if( !ok ) {
//      PRINTLN(F("FAILED"));
//      goto abort_begin;
//    }
//    PRINT2(status,HEX);
//    PRINT(F(" "));
//    if( status!=0x10 ) {
//      PRINTLN(F("ERROR"));
//      goto abort_begin;
//    }
//    PRINTLN(F("ok"));

//  // CCS811 back to sleep
//  wake_down();
//  // Return success
//  return true;

//abort_begin:
//  // CCS811 back to sleep
//  wake_down();
//  // Return failure
//  return false;
//}


// Advanced interface: i2cdelay ========================================================================================


// Delay before a repeated start - needed for e.g. ESP8266 because it doesn't handle I2C clock stretch correctly
//void CCS811_set_i2cdelay(int us) {
//  if( us<0 ) us= 0;
//  _i2cdelay_us= us;
//}


//// Get current delay
//int  CCS811_get_i2cdelay(void) {
//  return _i2cdelay_us;
//}


// Helper interface: nwake pin ========================================================================================


// _nwake<0 means nWAKE is not connected to a pin of the host, so no action needed
void CCS811_wake_init( void ) {
  nrf_gpio_cfg_output(CCS811_WAKEPIN);
}


void CCS811_wake_up( void) {
	
	nrf_gpio_pin_clear(CCS811_WAKEPIN);
	nrf_delay_us(CCS811_WAIT_AFTER_WAKE_US);
  //if( _nwake>=0 ) { digitalWrite(_nwake, LOW); delayMicroseconds(CCS811_WAIT_AFTER_WAKE_US);  }
	
}


void CCS811_wake_down( void) {
	
	nrf_gpio_pin_set(CCS811_WAKEPIN);
  //if( _nwake>=0 ) digitalWrite(_nwake, HIGH);
}


// Helper interface: i2c wrapper ======================================================================================





