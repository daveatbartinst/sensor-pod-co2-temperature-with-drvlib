/*
 * main.h
 *
 *  Created on: Jan 5, 2026
 *      Author: Dave
 */

#ifndef MAIN_H_
#define MAIN_H_


// variables added by dpb
extern uint16_t SCD30_ALTITUDE ;
extern uint8_t SCD_STATUS ;
extern uint8_t SCD_SECOND ;
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
//macros


void SCD_init(uint8_t ambient_pressure,
                      uint8_t asc_status,
                      uint8_t temp_offset,
                      uint8_t altitude_comp,
                      uint8_t frc_value);
void handle_UART_CMD();

#endif /* MAIN_H_ */
