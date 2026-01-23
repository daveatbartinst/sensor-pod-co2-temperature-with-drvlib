/*
 * main.h
 *
 *  Created on: Jan 5, 2026
 *      Author: Dave
 */

// testing notes -
// 1-21-26 dpb
// first iteration of co2 readings is a 10s state machine that reset the scd30 and sets for 2 sec reads, reads for 6 seconds,
// after 3reads ( 6 sec ) stop readings, read/parse/store min/max over an hour.
// doing the reset every 10 second is so the altitude switch could be read and sent to the scd30
//Every hour store in I2C_TRANSACTION_HOLDER[]
// the spread.  keeps data for 24 hours
// the results have been around 180ppm swing in an hour.  Have also seen that much swing in only ten 10-sec intervals.
// added a 470uF cap to power pins - no change in swing.

// next go back to Nate's continuous reading version and if more steady, do a check to see if the switch changes and only reset the
// scd30 if there is a change.
// not doing the reset keep reading within 5 over a 90 second sampling !!!!
















#ifndef MAIN_H_
#define MAIN_H_


// variables added by dpb
extern uint16_t SCD30_ALTITUDE ;
extern uint8_t SCD_STATUS ;
extern uint8_t SCD_SECOND ;


// From Nate's code, these were local variables inside main() and moved here as global variables
// clears space from stack and ensures if main() goes out of focus the variables are not lost

extern struct I2C_RX_Meas_t READING;
     // Keep these variables at this size. Minimal excess space but enough for maximal values.
extern uint32_t CO2_AVG ;
extern uint16_t T_AVG ;
extern uint16_t RH_AVG ;
extern uint8_t NUM_READS ;
extern uint16_t data_ready ;

     // Note: The below can be used for logging.
extern uint16_t peak_tx;
extern uint16_t peak_rx;
extern uint16_t peak_read;
extern uint16_t peak_UART_TX ;
extern uint16_t peak_UART_RX ;
extern uint16_t  peak_CO2 ;
extern uint16_t trough_CO2 ;
extern uint8_t  peak_RH ;
extern uint8_t trough_RH ;
extern uint8_t  peak_T ;
extern uint8_t trough_T ;







//SCD STATUS MACROs

//    //BIT 7 SET IS SCD_INIT IN PROGRESS.  EITHER FROM POWER UP OR ALTITUDE SWITCH CHANGED
//      BIT 1 INDICATES IT IS TIME TO READ AND PARSE SCD DATA
//      bit 0 set is use SCD_defaults.  used in SCD_init
#define Scd_Init_In_Progress (SCD_STATUS & BIT7)
#define Set_Scd_Init_In_Progress (SCD_STATUS |= BIT7)
#define Clr_Scd_Init_In_Progress (SCD_STATUS &= (~BIT7))

#define I2C_Success_Flag (SCD_STATUS & BIT6)
#define Set_I2C_Success_Flag (SCD_STATUS |= BIT6)
#define Clr_I2C_Success_Flag (SCD_STATUS &= (~BIT6))

#define Scd_Read_Parse (SCD_STATUS & BIT1)
#define Set_Scd_Read_Parse (SCD_STATUS |= BIT1)
#define Clr_Scd_Read_Parse (SCD_STATUS &= (~BIT1))

#define Scd_Use_defaults (SCD_STATUS & BIT0)
#define Set_Scd_Use_defaults (SCD_STATUS |= BIT0)
#define Clr_Scd_Use_defaults (SCD_STATUS &= (~BIT0))



extern char VERSION_STR[12];            //controller firmware version string
extern char EXT_VERSION_STR[16];        //controller firmware version string with extended info for factory & mfg
extern char SERIAL_NUMBER[12];          //controller serial number
//extern char WIFI_MAC_ID[13];          //wifi module MAC id
//extern spi_device_handle_t TOUCH_SPI; //touch screen spi handle
//extern spi_device_handle_t EEPROM_SPI;    //EEPROM spi handle
extern uint16_t MFG_ID;                 //manufacturer ID, set at check back
extern uint8_t FW_VERSION_MAJOR;        //major version, read from running app at startup
extern uint8_t FW_VERSION_MINOR;        //minor version, read from running app at startup
extern uint8_t FW_VERSION_BUILD;        //build version, read from running app at startup
//extern wifi_time_64bit_t SYSTEM_TIME;   //system clock time

//extern uint8_t data_ready_timing_byte;

extern uint8_t data_ready_timing_byte;

extern uint16_t DELAY_MILLISECOND ;
extern uint16_t U16_TRANSACTION_HOLDER ;
//macros


void SCD_init(uint8_t ambient_pressure,
                      uint8_t asc_status,
                      uint8_t temp_offset,
                      uint8_t altitude_comp,
                      uint8_t frc_value);
void handle_UART_CMD();

#endif /* MAIN_H_ */
