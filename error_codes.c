/*
 * error_codes.c
 *
 *  Created on: Nov 16, 2016
 *      Author: steve
 */

#include "error_codes.h"

//standard libs

#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "string.h"
//#include "driverlib.h"
#include <stdint.h>
// +-------------------------+
// | Global variable        |
// +-------------------------+


uint8_t ERROR_VALUE = 0xff;     //ff is no error. last_error_values hold value on eeprom

// +-------------------------+
// | Global constants        |
// +-------------------------+

//// Error code description array
////  There is room for about 26 characters per line
//const error_desc ERROR_STR[] = {
////   "|---max length for short desc --|"
//	{"E-0 Calibration Error","E-0 Calibration Error"}, //error 0
//	{"E-1 Temp ramping up too slowly","E-1 Kiln temperature increasing too slowly when ramping up"}, //error 1
//	{"E-2 Above allowed temp in hold","E-2 Kiln temperature 50F(28C) above hold temperature"}, //error 2
//	{"E-3 Below allowed temp in hold","E-3 Kiln temperature 50F(28C) below hold temperature"}, //error 3
//	{"E-4 Above allowed temp in ramp","E-4 Kiln temperature 50F(28C) above hold when ramping down"}, //error 4
//	{"E-5 Below set point in down ramp","E-5 Kiln temperature 50F below set pt when ramping down"},	//error 5
//	{"E-6 Thermocouple backwards","E-6 Thermocouple backwards"}, //error 6
//	{"E7",""}, //error 7 *** NOT USED ***
//	{"E-8 Unexpected falling temp","E-8 Unexpected falling temperature near end of firing"}, //error 8
//	{"E-- Bad EE Write","E-- Bad EE Write"}, //error 9 '-'
//	{"E-D Temp above allowable range","E-D Kiln temperature 50F(28C) above set point"}, //error 10 'D'
//	{"Write to EE Error","Error 11 writing to EEPROM"}, //error 11 EE Write/Verify error
//	{"E12",""}, //error 12 'A'  *** NOT USED DIRECTLY, REPLACED WITH ALTERNATE 'A' ERRORS***
//	{"PF Long term Power loss","PF Long term Power loss or restart below 140F"}, //error 13 'P'
//	{"E-R Memory Error","E-R Memory Error - RAM does not match EE"}, //error 14 'R'
//	{"E-bd Board temperature too high","E-bd board temperature too high"}, //error 15 BOARD TEMP
//	{"E-16","External oscillator error"},	//error 16 external oscillator
//	{"E17",""},	//error 17 A2D CHIP  *** NOT USED ***
//	{"E18",""},	//error 18 A2D BB  *** NOT USED ***
//	{"E19",""},	//error 19 SPI ERROR *** NOT USED ***
//	{"Write to EE Error","Error 20 writing to EEPROM"}, //error 20 Write_Busy_Error
//	{"E21",""},	//error 21 A2D_Error_TIMEOUT *** NOT USED ***
//	{"E-9 Wrong TC Type","E-9 Wrong TC Type"},//error 22 Thermocouple_Error
//	{"E23","Internal oscillator error"}, //error 23 Internal oscillator failed
//	{"E24",""},	//error 24 Error_Division_Zero *** NOT USED ***
//	{"E25",""},	//error 25 Error_Stuck *** NOT USED ***
//	{"E26",""},	//error 26 'N' *** NOT USED ***
//	{"E27",""},	//error 27 Error_A2D_Data *** NOT USED ***
//	{"E28",""}, //error 28 *** NOT USED ***
//	{"E29",""},	//error 29 *** NOT USED ***
//	{"E30",""},	//error 30 *** NOT USED ***
//    {"E-D1 Stuck relay","zone 1"},    //error 31 'D1'
//    {"E-D2 Stuck relay","zone 2"},    //error 32 'D2'
//    {"E-D3 Stuck relay","zone 3"},    //error 33 'D3'
//    {"E-DX Stuck relay",""},          //error 34 'DX'
//	{"",""}, //35 dpb getting cplt to *** NOT USED ***
//	{"Thermocouple failed","Thermocouple failed, check wiring"}, //36 dpb fail and ended firing
//	{"Illegal safety operation","Illegal safety operation"}, //37 Cap_Coup_Not_firing
//	{"Illegal_Cone_Value","Illegal_Cone_Value, check settings"}, //38 Illegal_Cone_Corr
//	{"Watch Dog Error","Cycle power to reset"}, //39 Watch Dog Error
//    {"E40",""}, //40 Amps_Not_Firing *** NOT USED ***
//	{"Illegal Ceramic Speed","Illegal ceramic speed, check settings"}, //41 Illegal_Ceramic_Spd
//	{"E42 All TCs backwards","E-42 All thermocouple backwards"}, //42 Set_Error_6all
//	{"E43 TC backwards Z1,Z2","E-43 Thermocouples Z1,Z2 backwards"}, //43 Set_Error_6Z12
//	{"E44 TC backwards Z2","E-44 Thermocouple Z2 backwards"}, //44 Set_Error_6Z2
//	{"E45 TC backwards Z3","E-45 Thermocouple Z3 backwards"}, //45 Set_Error_6Z3
//	{"E46 TC backwards Z1","E-46 Thermocouple Z1 backwards"}, //46 Set_Error_6Z1
//	{"E47 TC backwards Z2,Z3","E-47 Thermocouples Z2,Z3 backwards"}, //47 Error_6Z32
//	{"E48 TC backwards Z1,Z3","E-48 Thermocouples Z1,Z3 backwards"}, //48 Error_6Z13
//	{"Internal error, mode = Idle","Internal error, mode = Idle"}, //49 mode is idle but op_mode firing
//	{"Internal error, mode = Firing","Internal error, mode = Firing"}, //50 mode is firing but op_mode is idle
//	{"E51",""}, //51 mode is firing but op_mode is idle *** NOT USED ***
//	{"E52",""} , //52 No load V cal time out error *** NOT USED ***
//	{"E53",""}, //53 Full load V cal time out error *** NOT USED ***
//	{"E-1 with lid open","E-1 Lid open, kiln temperature increasing too slowly when ramping up"}, //54 - error 1 with lid open
//	{"E-3 with lid open","E-3 Lid Open, kiln temperature 50F(28C) below hold temperature"}, //55 - error 3 with lid open
//	{"E-5 with lid open","E-5 Lid open, kiln temperature 50F below set pt when ramping down"}, //56 - error 5 with lid open
//	{"E-8 with lid open","E-8 Lid open, unexpected falling temperature near end of firing"}, //57 - error 8 with lid open
//	{"E-AT Setting is out of range","E-AT One or more temperature settings is out of range"}, //58 - temperature above max, bad cone correlate or ramp rate is 0
//	{"E-AU Program no. is out of range","E-AU Custom program number is out of range"}, //59 - user program number out of range
//	{"E-AO Offset is out of range","E-AO One or more offset is out of range"}, //60 - offset out of range
//	{"E-AP Max program temp error","E-AP Max program temperature is out of range"}, //61 - max program temp too high
//	{"E-AG Setting is out of range","E-AG One or more glass setting is out of range"}, //62 - glass setting out of range
//	{"Soft Watch Dog Error","Cycle power to reset"}, //63 - soft watch dog error
//	{"Temperature deviation error","Temperature deviation error. Check thermocouple wiring"}, //64 - temperature deviation error
//	{"Burn out ignition error","Burn out ignition error. Ignition temperature not reached"}, //65 - burn out ignition error
//};

// +-------------------------+
// | Functions               |
// +-------------------------+

//// load a string with appropriate error code
//void get_error_string(char *str, uint8_t str_len, uint8_t error){
//    if (error <= Highest_Error_Value){
//        snprintf(str, str_len, "%s", (char*)ERROR_STR[error].brief);
//    } else if (No_Error){
//        snprintf(str, str_len, "no error");
//    } else {
//        snprintf(str, str_len, "unknown error");
//    }
//}// end get_error_string

