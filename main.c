
// sensor only board

/// SCD30 I2C ON UCB1
//  RADIO UART ON UCA1
//  EE, FLASH, SPI ON UCB0
//  SHT45 I2C ON UCB2
//  WIFI UART ON UCA0










#include <stdint.h>
#include "main.h"
#include "init_var.h"
#include <msp430.h>
#include "grlib.h"
#include "LCD_driver/Template_Driver.h"
#include "images/images.h"
//#include "driverlib.h"
#include "uart.h"
#include "I2C.h"
#include "timer.h"
#include "hw_config.h"

//#include "hw_config.h"
Graphics_Context g_sContext;


// +-------------------------+
// | Global variables        |
// +-------------------------+


//from gen3 kiln_control.c
uint16_t U16_TRANSACTION_HOLDER = 0;    //@ 0x00e3;
uint32_t U32HOLDER = 0;                 //scratch pad variable
uint64_t U64HOLDER = 0;                 //scratch pad variable
uint64_t U64HOLDER1 = 0;                //scratch pad variable
uint16_t HALF_MILLISECOND = 0;          //@ 0x0070;  timing variables
uint16_t DELAY_MILLISECOND = 0;     // used to count down in ms_delay function
uint16_t ONE_MILLISECOND = 0;           //@ 0x0071;
uint16_t HUNDRED_MILLISECOND = 0;       //@ 0x0072;
uint16_t SECOND = 0;                    //@ 0x0073;
uint16_t MINUTE = 0;                    //@ 0x008f;
uint16_t EE_MILLISECOND_TIMER = 0;          //@ 0x0074;
uint16_t SPI_SEND_MILLISECOND = 0;      //@ 0x0076;
uint16_t TEST_MILLISECOND = 0;          // millisecond timer used for testing process timing
uint8_t EE_WRITE_TIMER = 0;             //timer for monitoring EEPROM write time in mS

//from gen3 main.c
char VERSION_STR[12];           //controller firmware version string
char EXT_VERSION_STR[16];       //controller firmware version string with extended info for factory & mfg
char SERIAL_NUMBER[12];         //controller serial number
//char WIFI_MAC_ID[13];         //wifi module MAC id
uint16_t MFG_ID=0;              //manufacturer ID, set at check back
//uint16_t HALF_MIL_COUNTER=0; //incs every half mil, wraps at 2000 (1 second)
uint8_t HUNDRED_MS_COUNTER=0; //100ms counter, incs every ms, wraps at 100 (0.1 second)

uint8_t FW_VERSION_MAJOR = 0;       //major version, read from running app at startup
uint8_t FW_VERSION_MINOR = 0;       //minor version, read from running app at startup
uint8_t FW_VERSION_BUILD = 0;       //build version, read from running app at startup
//wifi_time_64bit_t SYSTEM_TIME;      //system clock time
uint8_t WIFI_RESET_COUNT = 0;       //number of times the wifi has been reset due to no internet since last online


// from Nate's code
// Holds Flag Values for half_mil:
//uint8_t TIMING_FLAG_BYTE = 0;  // system timing flags to track new sec, minute ....
//// Half_mil Counters Used for Timing:
//
//// Iterator for number of milliseconds left during a wait.
//uint16_t MS_COUNTER = 0;
//uint16_t FIFTY_MS_COUNTER = 0;
//uint16_t SECOND_COUNTER = 0;
//uint16_t TEN_SECOND_COUNTER = 0;
//uint16_t MINUTE_COUNTER = 0;
//uint16_t HOUR_COUNTER = 0;

//added by dpb
uint8_t SCD_SECOND = 0;
uint16_t SCD30_ALTITUDE =0;

// added by dpb
uint8_t SCD_STATUS =0;


// From Nate's code, these were local variables inside main() and moved here as global variables
// clears space from stack and ensures if main() goes out of focus the variables are not lost

struct I2C_RX_Meas_t READING;
     // Keep these variables at this size. Minimal excess space but enough for maximal values.
uint32_t CO2_avg = 0;
uint16_t T_avg = 0;
uint16_t RH_avg = 0;
uint8_t NUM_READS = 0;
uint16_t data_ready = 0;

     // Note: The below can be used for logging.
uint16_t peak_tx = 0, peak_rx = 0, peak_read = 0, peak_UART_TX = 0, peak_UART_RX = 0;
uint16_t peak_CO2 = 0, trough_CO2 = 0;
uint8_t peak_RH = 0, trough_RH = 0;
uint8_t peak_T = 0, trough_T = 0;












////    //BIT 7 SET IS SCD_INIT IN PROGRESS.  EITHER FROM POWER UP OR ALTITUDE SWITCH CHANGED
////      BIT 1 INDICATES IT IS TIME TO READ AND PARSE SCD DATA
////      bit 0 set is use SCD_defaults.  used in SCD_init
//#define Scd_Init_In_Progress (SCD_STATUS & BIT7)
//#define Set_Scd_Init_In_Progress (SCD_STATUS |= BIT7)
//#define Clr_Scd_Init_In_Progress (SCD_STATUS &= (~BIT7))
//
//#define I2C_Success_Flag (SCD_STATUS & BIT6)
//#define Set_I2C_Success_Flag (SCD_STATUS |= BIT6)
//#define Clr_I2C_Success_Flag (SCD_STATUS &= (~BIT6))
//
//#define Scd_Read_Parse (SCD_STATUS & BIT1)
//#define Set_Scd_Read_Parse (SCD_STATUS |= BIT1)
//#define Clr_Scd_Read_Parse (SCD_STATUS &= (~BIT1))
//
//#define Scd_Use_defaults (SCD_STATUS & BIT0)
//#define Set_Scd_Use_defaults (SCD_STATUS |= BIT0)
//#define Clr_Scd_Use_defaults (SCD_STATUS &= (~BIT0))
//

///**
// * @brief Sets the sensor settings for main(). USE_DEFAULTS allows for default values to be loaded (true) or manually entered values (false).
// *
// * @param ambient_pressure ambient pressure in mBar .
// * @param asc_status automatic self-calibration status (true/false = on/off).
// * @param temp_offset temperature offset 0.01 degrees C.
// * @param altitude_comp altitude in meters above sea level.
// * @param frc_value forced recalibration value in ppm CO2.
// */

void SCD_init(uint8_t ambient_pressure,
                       uint8_t asc_status,
                       uint8_t temp_offset,
                       uint8_t altitude_comp,
                       uint8_t frc_value ) {
     // Ensure required 2 second power up wait time is met, split up over TX time below.
     // TODO: set error if any of the settings failed I2C_Success. Reinit.
    // if (Scd_Use_defaults){
       SCD30_ALTITUDE = ((~P3IN & 0x0f)*305); // ALTITUDE IN METERS. switch reading is 1-10 indicating thousands of feet.
         ambient_pressure = PRODUCTION_AMBIENT_PRESSURE_MBAR_DEFAULT;
         asc_status = PRODUCTION_AUTOMATIC_SELF_CALIBRATION_DEFAULT;
         temp_offset = PRODUCTION_TEMPERATURE_OFFSET_DEFAULT;
         altitude_comp = SCD30_ALTITUDE;
         // No FRC for defaults.
//     }
//     else{
//         altitude_comp = SCD30_ALTITUDE;
//     }

     //I2C_SETUP_LED_on();
     Sensirion_trigger_continuous_measurement(ambient_pressure); // meas. interval is 2 sec by default.
     wait_ms(400);

    // I2C_SETUP_LED_on();
     Sensirion_toggle_automatic_self_calibration(asc_status);
     wait_ms(400);

    // I2C_SETUP_LED_on();
     Sensirion_set_temperature_offset(temp_offset);
     wait_ms(400);

   //  I2C_SETUP_LED_on();
     Sensirion_set_altitude_compensation(altitude_comp);
     wait_ms(400);

//     if (!Scd_Use_defaults)
//     {
//         // No settings overwritten if ! defaults. FRC used.
//         Sensirion_set_altitude_compensation(altitude_comp);
//             wait_ms(400);
//     //    I2C_SETUP_LED_on();
////         Sensirion_set_forced_recalibration_value(frc_value);
////         wait_ms(400);
//     }

   //  ledRGB_off();
   //  UART_TRANSMITTING_LED_on();
     //Initialize UART after sensor is correctly set or I2C timing may be affected, causing errors.
    // UART_init();  THIS IS TAKEN CARE OF IN hw_config
     // Initialize all values with 0s while Sensirion reading buffer fills. Then send average of values.
   //  UART_populate_TX_READ_buffer();
  //   UART_RECEIVE_SETUP;
     wait_ms(400);
    // UART_TRANSMITTING_LED_off();
     Clr_Scd_Init_In_Progress;   //if altitude switch is changed, SCD_INIT is triggered.  this clear the flag when init is done dpb
     SCD_SECOND = 0; // start timer to read after 8 seconds
 }




void handle_UART_CMD()
 {
     // DEBUG: This hasn't been thoroughly tested.
     switch (CMD)
     {
     case UART_READ_DATA_COMMAND:
         // Prepare for TX.
         UART_SEND_SETUP;
         SET_TX_READ;
         RESET_CMD;
         break;
     case UART_SET_MEASUREMENT_INTERVAL_COMMAND:
         UART_CMD_SET_PRESETS;
         Sensirion_set_measurement_interval(ARG);
         break;
     case UART_GET_MEASUREMENT_INTERVAL_COMMAND:
         if (!UART_FILL_CMD_BUFFER_FLG)
         {
             if (UART_TX_RX_FLIP_FLOP)
             {
                 UART_CMD_TX_PRESETS;
                 Sensirion_get_measurement_interval_TX();
             }
             else if (!UART_TX_RX_FLIP_FLOP)
             {
                 UART_CMD_RX_PRESETS;
                 Sensirion_get_measurement_interval_RX(&ARG);
             }
         } // End if (!UART_FILL_CMD_BUFFER_FLG)
         // Fill CMD_BUFFER
         else
         {
             UART_CMD_FILL_BUFFER_PRESETS;
             UART_populate_TX_COMMAND_buffer();
         } // End fill CMD_BUFFER
         break;
     case UART_SET_FORCED_RECALIBRATION_COMMAND:
         UART_CMD_SET_PRESETS;
         Sensirion_set_forced_recalibration_value(ARG);
         break;
     case UART_GET_FORCED_RECALIBRATION_COMMAND:
         if (!UART_FILL_CMD_BUFFER_FLG)
         {
             if (UART_TX_RX_FLIP_FLOP)
             {
                 UART_CMD_TX_PRESETS;
                 Sensirion_get_forced_recalibration_value_TX();
             }
             else if (!UART_TX_RX_FLIP_FLOP)
             {
                 UART_CMD_RX_PRESETS;
                 Sensirion_get_forced_recalibration_value_RX(&ARG);
             }
         } // End if (!UART_FILL_CMD_BUFFER_FLG)
         // Fill CMD_BUFFER
         else
         {
             UART_CMD_FILL_BUFFER_PRESETS;
             UART_populate_TX_COMMAND_buffer();
         } // End fill CMD_BUFFER
         break;
     case UART_TOGGLE_AUTOMATIC_SELF_CALIBRATION_COMMAND:
         UART_CMD_SET_PRESETS;
         Sensirion_toggle_automatic_self_calibration(ARG);
         break;
     case UART_GET_AUTOMATIC_SELF_CALIBRATION_STATUS_COMMAND:
         if (!UART_FILL_CMD_BUFFER_FLG)
         {
             if (UART_TX_RX_FLIP_FLOP)
             {
                 UART_CMD_TX_PRESETS;
                 Sensirion_get_automatic_self_calibration_status_TX();
             }
             else if (!UART_TX_RX_FLIP_FLOP)
             {
                 UART_CMD_RX_PRESETS;
                 Sensirion_get_automatic_self_calibration_status_RX(&ARG);
             }
         } // End if (!UART_FILL_CMD_BUFFER_FLG)
         // Fill CMD_BUFFER
         else
         {
             UART_CMD_FILL_BUFFER_PRESETS;
             UART_populate_TX_COMMAND_buffer();
         } // End fill CMD_BUFFER
         break;
     case UART_SET_TEMPERATURE_OFFSET_COMMAND:
         UART_CMD_SET_PRESETS;
         Sensirion_set_temperature_offset(ARG);
         break;
     case UART_GET_TEMPERATURE_OFFSET_COMMAND:
         if (!UART_FILL_CMD_BUFFER_FLG)
         {
             if (UART_TX_RX_FLIP_FLOP)
             {
                 UART_CMD_TX_PRESETS;
                 Sensirion_get_temperature_offset_TX();
             }
             else if (!UART_TX_RX_FLIP_FLOP)
             {
                 UART_CMD_RX_PRESETS;
                 Sensirion_get_temperature_offset_RX(&ARG);
             }
         } // End if (!UART_FILL_CMD_BUFFER_FLG)
         // Fill CMD_BUFFER
         else
         {
             UART_CMD_FILL_BUFFER_PRESETS;
             UART_populate_TX_COMMAND_buffer();
         } // End fill CMD_BUFFER
         break;
     case UART_SET_ALTITUDE_COMPENSATION_COMMAND:
         UART_CMD_SET_PRESETS;
         Sensirion_set_altitude_compensation(ARG);
         break;
     case UART_GET_ALTITUDE_COMPENSATION_COMMAND:
         if (!UART_FILL_CMD_BUFFER_FLG)
         {
             if (UART_TX_RX_FLIP_FLOP)
             {
                 UART_CMD_TX_PRESETS;
                 Sensirion_get_altitude_compensation_TX();
             }
             else if (!UART_TX_RX_FLIP_FLOP)
             {
                 UART_CMD_RX_PRESETS;
                 Sensirion_get_altitude_compensation_RX(&ARG);
             }
         } // End if (!UART_FILL_CMD_BUFFER_FLG)
         // Fill CMD_BUFFER
         else
         {
             UART_CMD_FILL_BUFFER_PRESETS;
             UART_populate_TX_COMMAND_buffer();
         } // End fill CMD_BUFFER
         break;
     case UART_READ_FIRMWARE_VERSION_COMMAND:
         if (!UART_FILL_CMD_BUFFER_FLG)
         {
             if (UART_TX_RX_FLIP_FLOP)
             {
                 UART_CMD_TX_PRESETS;
                 Sensirion_read_firmware_version_TX();
             }
             else if (!UART_TX_RX_FLIP_FLOP)
             {
                 UART_CMD_RX_PRESETS;
                 Sensirion_read_firmware_version_RX(&ARG);
             }
         } // End if (!UART_FILL_CMD_BUFFER_FLG)
         // Fill CMD_BUFFER
         else
         {
             UART_CMD_FILL_BUFFER_PRESETS;
             UART_populate_TX_COMMAND_buffer();
         } // End fill CMD_BUFFER
         break;
     case UART_SOFT_RESET_COMMAND:
         UART_CMD_SET_PRESETS;
         Sensirion_soft_reset();
         break;
     case UART_SET_AMBIENT_PRESSURE_COMMAND:
         UART_CMD_SET_PRESETS;
         Sensirion_trigger_continuous_measurement(ARG);
         break;
     case 0x0E:
 // TODO: Set min/max if desired
         break;
     case 0x0F:
         // TODO: Clear min/max if desired
         break;
     default:
         // Shouldn't normally take this path
         break;
     }
 }


//
//void main_init(uint8_t ambient_pressure,
//                      uint8_t asc_status,
//                      uint8_t temp_offset,
//                      uint8_t altitude_comp,
//                      uint8_t frc_value,
//                      bool USE_DEFAULTS);

///**
// * @brief Handles the process of filling the UART_COMMAND_BUFFER with the requested information.
// * TODO: Confirm that a setting was successfully set based upon I2C_SUCCESS. Respond to PULSE.
// *

//void handle_UART_CMD();




void main(void)
{

uint8_t quarter_second_count = 0;

//struct I2C_RX_Meas_t reading;
//     // Keep these variables at this size. Minimal excess space but enough for maximal values.
//uint32_t CO2_avg = 0;
//uint16_t T_avg = 0;
//uint16_t RH_avg = 0;
//uint8_t num_reads = 0;
//uint16_t data_ready = 0;
//
//     // Note: The below can be used for logging.
//volatile uint16_t peak_tx = 0, peak_rx = 0, peak_read = 0, peak_UART_TX = 0, peak_UART_RX = 0;
//volatile uint16_t peak_CO2 = 0, trough_CO2 = 0;
//volatile uint8_t peak_RH = 0, trough_RH = 0;
//volatile uint8_t peak_T = 0, trough_T = 0;

      WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
      init_hardware();  // INIT ALL - CLOCK SOURCE GPIO, SERIAL PORTS, A2D
     //General_GPIO_init();
    // I2C_GPIO_init();
   //  UART_GPIO_init();
   //  ledRGB_GPIO_init();
     UCB1_I2C_init_master(Sensirion_Baud_Rate, Sensirion_Address); // SET UP FOR SCD30
     timer_A_init();

     Set_Scd_Init_In_Progress;  //if power up or altitude switch is changed, SCD_INIT is triggered.  this clear the flag when init is done dpb
     Set_Scd_Use_defaults;
     SCD_init(PRODUCTION_AMBIENT_PRESSURE_MBAR_DEFAULT,
                                PRODUCTION_AUTOMATIC_SELF_CALIBRATION_DEFAULT,
                                (uint8_t)200,
                                PRODUCTION_ALTITUDE_COMPENSATION_DEFAULT,
                                (uint8_t)1272);  // this takes several seconds so other functions are stopped
     while( SCD_SECOND < 2) {
         //Wait until scd first read is done
     }
     SCD_SECOND = 0; // THIS WILL SYNC SO WE CAN USE DATA ON ODD SCD_SECOND



 while (1){
      //  if (!Scd_Init_In_Progress){

             // Every 50 ms
          if (New_50_ms){
              Clr_New_50_ms;


             if(!Scd_Read_Parse){
                 quarter_second_count =   0;
//                 if (UART_TIME > UART_TIMEOUT)
//                 {
//                     handle_UART_TIMEOUT();
//                 }
//                 if (I2C_TIME > I2C_TIMEOUT)
//                 {
//                     handle_I2C_TIMEOUT();
//                 }
//                 if (I2C_REDUX_COUNT > 6)
//                 {
//                     I2C_REDUX_COUNT = 1;
//                     Sensirion_soft_reset();
//                     // Caught by the default path of I2C switch, essentially a 250 ms wait time.
//                     SKIP_READ;
//                 }

             } // end if not Scd_Read_Parse
             else{ // is time to Scd_Read_Parse so inc quarter sec
                 quarter_second_count++;
             }
                 // TODO: Handle UART_REDUX_COUNT
          } // end new 50 ms

             // Every 250 ms (derived from 50_ms)
             if (quarter_second_count > 4)
             {
                 quarter_second_count = 0;

                 // Parse RX if done receiving.
    //             if (UART_PARSE_RX_FLG)
    //             {
    //                 UART_parse_RX_buffer();
    //             }
    //
    //             // Start handle UART CMD. 0xF0 is CMD default reset state.
    //             if (!CMD_IS_RESET)
    //             {
    //                 handle_UART_CMD();
    //             } // End handle UART CMD
    //

                 // Start get sensor READ
                 // This switch() should take 1.5 seconds to loop back to case 0 plus polling time for data_ready.
                 switch (data_ready_timing_byte)
                 {
//                 case 0: // Send data_ready_TX
//                     Sensirion_get_data_ready_status_TX();
//                     data_ready_timing_byte++;
//                     // Wait 250 ms
//                     break;
//                 case 1: // Send data_ready_RX
//                     Sensirion_get_data_ready_status_RX(&data_ready);
//                     data_ready_timing_byte++;
//                     // Wait 250 ms
//                     break;
                 case 0: // If data_ready, send read_TX
                  //   I2C_parse_RX_DATA();
                  //   if (data_ready)
                  //   {
                         Sensirion_read_measurement_TX();  // send cmd to read data
                         data_ready_timing_byte++;
                  //   }
                  //   else
                  //   {
                  //       // Back to polling for data ready
                  //       data_ready_timing_byte = 0;
                  //   }
                     // Wait 250 ms
                     break;
                 case 1: // If sent read_TX, send read_RX
                     Sensirion_read_measurement_RX(&READING); // read data
                     data_ready_timing_byte++;
                     // Wait 250 ms
                     break;
                 case 2: // Allow for 500 ms to have passed.
                     data_ready_timing_byte++;
                     // Wait another 250 ms
                     break;
                 case 3: //Analyze reading, update state.
                     I2C_parse_RX_DATA();
                     NUM_READS++;
                     // Prevent from num_reads overflow.
                     if (NUM_READS > 200)
                     {
                         NUM_READS = 10;
                     }
                     // Reset back to start.
                     data_ready_timing_byte = 0;
                     Clr_Scd_Read_Parse;
                     break;
                 default:
                     // Shouldn't normally take this path. Clearing flags to prevent program halt.
                     // This path can act as a 250 ms wait timer for I2C TX/RX/Parse function calls.
                     data_ready_timing_byte = 0;
                     Clr_Scd_Read_Parse;
                     break;
                 } // End get sensor READ
             }
             // Every second
             if (New_Second){
                 Clr_New_Second;
               //  SCD30_ALTITUDE = ((~P3IN & 0x0f)*305); // ALTITUDE IN METERS
// 1 second state machine.  0 = scd_init, 7 is stop readings, 8 is read scd data, 9 parse
                  if(SCD_SECOND & 0x01){ // when odd use data.  readings are taken on even seconds
                      Set_Scd_Read_Parse;        // allow 2 sec to parse
                      LED_1_On;
                  }
                  else{
                  LED_1_Off;
                  }
//                  else if(SCD_SECOND == 7 ){// stop scd readings
//                      Sensirion_stop_continuous_measurement();
//                  }
//                  else if(SCD_SECOND == 8 ){
//                        Set_Scd_Read_Parse;        // allow 2 sec to parse
//                  }
//                  else if(SCD_SECOND == 10 ){ // scd_second gets clr in hafmil and at end of scd_init
//                    // Set_Scd_Init_In_Progress;    // allows scd_init to run
//                     Clr_Scd_Read_Parse;   //gets set after 6 seconds of scd readings
//                  }

             }  // end new second

                 // Every 10 seconds
                 if (New_Ten_Second)
                 {
                     Clr_New_Ten_Second;
                     // Allow read buffer to populate fully, else send 0s.
                     if (NUM_READS > I2C_Readings_Buffer_Size)
                     {
                         // Update averages before sending.
                         I2C_avg_readings(&CO2_avg, &T_avg, &RH_avg);

                         CO2_MSB = (uint8_t)(CO2_avg >> One_Byte);
                         CO2_LSB = (uint8_t)CO2_avg;
                         TEMPERATURE = (uint8_t)T_avg;
                         HUMIDITY = (uint8_t)RH_avg;
                        // UART_populate_TX_READ_buffer();
                     }

        //             //TODO: Peak values are for debugging only.
        //             peak_tx = I2C_PEAK_TX_TIME;
        //             peak_rx = I2C_PEAK_RX_TIME;
        //             peak_read = I2C_PEAK_READ_TIME;
        //             peak_UART_TX = UART_PEAK_TX_TIME;
        //             peak_UART_RX = UART_PEAK_RX_TIME;
        //
        //             peak_CO2 = max_CO2;
        //             peak_RH = max_RH;
        //             peak_T = max_T;
        //
        //             trough_CO2 = min_CO2;
        //             trough_RH = min_RH;
        //             trough_T = min_T;
                 } // End 10 seconds

                 // TODO: The below can be used for logging. E.g., peak TX, RX, READ times for I2C and UART.

                 // Every minute
                 if (New_Minute)
                 {
                     Clr_New_Minute;



                 } // End new minute

                 // Every Hour
                 if (New_Hour){
                     Clr_New_Hour;

                     I2C_TRANSACTION_HOLDER [HOUR_COUNTER] = (max_CO2-min_CO2);
                     max_CO2 =0;  // clear min/max every hour
                     min_CO2 = 0xffff;



             } // End new hour
        //   } //end if (!Scd_Init_In_Progress)
//      else{// do scd30 init every 10 seconds
//          SCD_init(PRODUCTION_AMBIENT_PRESSURE_MBAR_DEFAULT,
//                             PRODUCTION_AUTOMATIC_SELF_CALIBRATION_DEFAULT,
//                             (uint8_t)200,
//                             PRODUCTION_ALTITUDE_COMPENSATION_DEFAULT,
//                             (uint8_t)1272);  // this takes several seconds so other functions are stopped
//     }

 }  // end while(1)
}
 /********************************************************************************************
  *                                  MAIN()  FUNCTIONS:                                *
  ********************************************************************************************/




//void main_init(uint8_t ambient_pressure,
//                       uint8_t asc_status,
//                       uint8_t temp_offset,
//                       uint8_t altitude_comp,
//                       uint8_t frc_value,
//                       bool USE_DEFAULTS) {
//     // Ensure required 2 second power up wait time is met, split up over TX time below.
//     // TODO: set error if any of the settings failed I2C_Success. Reinit.
//     if (USE_DEFAULTS)
//     {
//         ambient_pressure = PRODUCTION_AMBIENT_PRESSURE_MBAR_DEFAULT;
//         asc_status = PRODUCTION_AUTOMATIC_SELF_CALIBRATION_DEFAULT;
//         temp_offset = PRODUCTION_TEMPERATURE_OFFSET_DEFAULT;
//         altitude_comp = PRODUCTION_ALTITUDE_COMPENSATION_DEFAULT;
//         // No FRC for defaults.
//     }
//
//
//     //I2C_SETUP_LED_on();
//     Sensirion_trigger_continuous_measurement(ambient_pressure);
//     wait_ms(400);
//
//    // I2C_SETUP_LED_on();
//     Sensirion_toggle_automatic_self_calibration(asc_status);
//     wait_ms(400);
//
//    // I2C_SETUP_LED_on();
//     Sensirion_set_temperature_offset(temp_offset);
//     wait_ms(400);
//
//   //  I2C_SETUP_LED_on();
//     Sensirion_set_altitude_compensation(altitude_comp);
//     wait_ms(400);
//
//     if (!USE_DEFAULTS)
//     {
//         // No settings overwritten if ! defaults. FRC used.
//
//     //    I2C_SETUP_LED_on();
//         Sensirion_set_forced_recalibration_value(frc_value);
//         wait_ms(400);
//     }
//
//   //  ledRGB_off();
//   //  UART_TRANSMITTING_LED_on();
//     //Initialize UART after sensor is correctly set or I2C timing may be affected, causing errors.
//     UART_init();
//     // Initialize all values with 0s while Sensirion reading buffer fills. Then send average of values.
//     UART_populate_TX_READ_buffer();
//     UART_RECEIVE_SETUP;
//     wait_ms(400);
//    // UART_TRANSMITTING_LED_off();
//    // Clr_Scd_Init_In_Progress;   //if altitude switch is changed, SCD_INIT is triggered.  this clear the flag when init is done dpb
// }
//

//void handle_UART_CMD()
// {
//     // DEBUG: This hasn't been thoroughly tested.
//     switch (CMD)
//     {
//     case UART_READ_DATA_COMMAND:
//         // Prepare for TX.
//         UART_SEND_SETUP;
//         SET_TX_READ;
//         RESET_CMD;
//         break;
//     case UART_SET_MEASUREMENT_INTERVAL_COMMAND:
//         UART_CMD_SET_PRESETS;
//         Sensirion_set_measurement_interval(ARG);
//         break;
//     case UART_GET_MEASUREMENT_INTERVAL_COMMAND:
//         if (!UART_FILL_CMD_BUFFER_FLG)
//         {
//             if (UART_TX_RX_FLIP_FLOP)
//             {
//                 UART_CMD_TX_PRESETS;
//                 Sensirion_get_measurement_interval_TX();
//             }
//             else if (!UART_TX_RX_FLIP_FLOP)
//             {
//                 UART_CMD_RX_PRESETS;
//                 Sensirion_get_measurement_interval_RX(&ARG);
//             }
//         } // End if (!UART_FILL_CMD_BUFFER_FLG)
//         // Fill CMD_BUFFER
//         else
//         {
//             UART_CMD_FILL_BUFFER_PRESETS;
//             UART_populate_TX_COMMAND_buffer();
//         } // End fill CMD_BUFFER
//         break;
//     case UART_SET_FORCED_RECALIBRATION_COMMAND:
//         UART_CMD_SET_PRESETS;
//         Sensirion_set_forced_recalibration_value(ARG);
//         break;
//     case UART_GET_FORCED_RECALIBRATION_COMMAND:
//         if (!UART_FILL_CMD_BUFFER_FLG)
//         {
//             if (UART_TX_RX_FLIP_FLOP)
//             {
//                 UART_CMD_TX_PRESETS;
//                 Sensirion_get_forced_recalibration_value_TX();
//             }
//             else if (!UART_TX_RX_FLIP_FLOP)
//             {
//                 UART_CMD_RX_PRESETS;
//                 Sensirion_get_forced_recalibration_value_RX(&ARG);
//             }
//         } // End if (!UART_FILL_CMD_BUFFER_FLG)
//         // Fill CMD_BUFFER
//         else
//         {
//             UART_CMD_FILL_BUFFER_PRESETS;
//             UART_populate_TX_COMMAND_buffer();
//         } // End fill CMD_BUFFER
//         break;
//     case UART_TOGGLE_AUTOMATIC_SELF_CALIBRATION_COMMAND:
//         UART_CMD_SET_PRESETS;
//         Sensirion_toggle_automatic_self_calibration(ARG);
//         break;
//     case UART_GET_AUTOMATIC_SELF_CALIBRATION_STATUS_COMMAND:
//         if (!UART_FILL_CMD_BUFFER_FLG)
//         {
//             if (UART_TX_RX_FLIP_FLOP)
//             {
//                 UART_CMD_TX_PRESETS;
//                 Sensirion_get_automatic_self_calibration_status_TX();
//             }
//             else if (!UART_TX_RX_FLIP_FLOP)
//             {
//                 UART_CMD_RX_PRESETS;
//                 Sensirion_get_automatic_self_calibration_status_RX(&ARG);
//             }
//         } // End if (!UART_FILL_CMD_BUFFER_FLG)
//         // Fill CMD_BUFFER
//         else
//         {
//             UART_CMD_FILL_BUFFER_PRESETS;
//             UART_populate_TX_COMMAND_buffer();
//         } // End fill CMD_BUFFER
//         break;
//     case UART_SET_TEMPERATURE_OFFSET_COMMAND:
//         UART_CMD_SET_PRESETS;
//         Sensirion_set_temperature_offset(ARG);
//         break;
//     case UART_GET_TEMPERATURE_OFFSET_COMMAND:
//         if (!UART_FILL_CMD_BUFFER_FLG)
//         {
//             if (UART_TX_RX_FLIP_FLOP)
//             {
//                 UART_CMD_TX_PRESETS;
//                 Sensirion_get_temperature_offset_TX();
//             }
//             else if (!UART_TX_RX_FLIP_FLOP)
//             {
//                 UART_CMD_RX_PRESETS;
//                 Sensirion_get_temperature_offset_RX(&ARG);
//             }
//         } // End if (!UART_FILL_CMD_BUFFER_FLG)
//         // Fill CMD_BUFFER
//         else
//         {
//             UART_CMD_FILL_BUFFER_PRESETS;
//             UART_populate_TX_COMMAND_buffer();
//         } // End fill CMD_BUFFER
//         break;
//     case UART_SET_ALTITUDE_COMPENSATION_COMMAND:
//         UART_CMD_SET_PRESETS;
//         Sensirion_set_altitude_compensation(ARG);
//         break;
//     case UART_GET_ALTITUDE_COMPENSATION_COMMAND:
//         if (!UART_FILL_CMD_BUFFER_FLG)
//         {
//             if (UART_TX_RX_FLIP_FLOP)
//             {
//                 UART_CMD_TX_PRESETS;
//                 Sensirion_get_altitude_compensation_TX();
//             }
//             else if (!UART_TX_RX_FLIP_FLOP)
//             {
//                 UART_CMD_RX_PRESETS;
//                 Sensirion_get_altitude_compensation_RX(&ARG);
//             }
//         } // End if (!UART_FILL_CMD_BUFFER_FLG)
//         // Fill CMD_BUFFER
//         else
//         {
//             UART_CMD_FILL_BUFFER_PRESETS;
//             UART_populate_TX_COMMAND_buffer();
//         } // End fill CMD_BUFFER
//         break;
//     case UART_READ_FIRMWARE_VERSION_COMMAND:
//         if (!UART_FILL_CMD_BUFFER_FLG)
//         {
//             if (UART_TX_RX_FLIP_FLOP)
//             {
//                 UART_CMD_TX_PRESETS;
//                 Sensirion_read_firmware_version_TX();
//             }
//             else if (!UART_TX_RX_FLIP_FLOP)
//             {
//                 UART_CMD_RX_PRESETS;
//                 Sensirion_read_firmware_version_RX(&ARG);
//             }
//         } // End if (!UART_FILL_CMD_BUFFER_FLG)
//         // Fill CMD_BUFFER
//         else
//         {
//             UART_CMD_FILL_BUFFER_PRESETS;
//             UART_populate_TX_COMMAND_buffer();
//         } // End fill CMD_BUFFER
//         break;
//     case UART_SOFT_RESET_COMMAND:
//         UART_CMD_SET_PRESETS;
//         Sensirion_soft_reset();
//         break;
//     case UART_SET_AMBIENT_PRESSURE_COMMAND:
//         UART_CMD_SET_PRESETS;
//         Sensirion_trigger_continuous_measurement(ARG);
//         break;
//     case 0x0E:
// // TODO: Set min/max if desired
//         break;
//     case 0x0F:
//         // TODO: Clear min/max if desired
//         break;
//     default:
//         // Shouldn't normally take this path
//         break;
//     }
// }
