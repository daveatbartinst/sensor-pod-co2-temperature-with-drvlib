/**
 * @file init_var.h
 * @author Nathaniel Suddarth
 * @brief Holds variables and macros used in configuring the CO2 module. Allows for quick changing of hardware components.
 * @version 0.1
 * @date 2025-06-30
 *
 *
 */

#ifndef INIT_VAR_H_
#define INIT_VAR_H_

//#include "I2C.h"
//#include "Timer.h"
//#include "UART.h"
////#include "driverlib.h"
//#include <msp430.h>
#include <stdint.h>
////#include "driverlib.h"

// Used in main() to track the state during READ operations.

extern uint8_t data_ready_timing_byte;


/****************************************************************************************************
*                                       SCD30 PRODUCTION SETTINGS:                                  *
*****************************************************************************************************/


/* Default ambient pressure is 1013.25 mbar. Deactivated to use altitude instead since most customers
won't have accurate barometric pressure readings to input. They are expected to enter their altitude
in feet above sea level into the ClimateBoss/Pulse, which will be sent to the CO2 Module as a
command to set altitude in meters above sea level. */
#define PRODUCTION_AMBIENT_PRESSURE_MBAR_DEFAULT (0) // mbar
#define PRODUCTION_MEASUREMENT_INTERVAL_DEFULT 2   // seconds
// Over ~7 days of fresh air, calibrates the lowest frequently read CO2 value to 400 ppm.
#define PRODUCTION_AUTOMATIC_SELF_CALIBRATION_DEFAULT (true)
// Calibrates the current CO2 value to the given value.
#define PRODUCTION_FORCED_RECALIBRATION_VALUE_DEFAULT (400) // ppm
// Calibrates the temperature value to the given value.
#define PRODUCTION_TEMPERATURE_OFFSET_DEFAULT (0) // 0.01 degrees C
// Alternative to using ambient pressure compensation. Set to Fort Madison, Iowa.
#define PRODUCTION_ALTITUDE_COMPENSATION_DEFAULT (159) // m above sea level

/****************************************************************************************************
*                                 SCD30 FACTORY DEFAULT SETTINGS:                                   *
*****************************************************************************************************/

// Altitude Compensation is not used, ambient pressure compensation is used instead.
#define SENSIRION_AMBIENT_PRESSURE_MBAR_DEFAULT (1013.25) // mbar
#define SENSIRION_MEASUREMENT_INTERVAL_DEFULT (2)         // seconds
// Over ~7 days of fresh air, calibrates the lowest frequently read CO2 value to 400 ppm.
#define SENSIRION_AUTOMATIC_SELF_CALIBRATION_DEFAULT (false)
// Calibrates the current CO2 value to the given value.
#define SENSIRION_FORCED_RECALIBRATION_VALUE_DEFAULT (400) // ppm
// Calibrates the temperature value to the given value.
#define SENSIRION_TEMPERATURE_OFFSET_DEFAULT (0) // 0.01 degrees C


/****************************************************************************************************
*                                       CRC CHECKSUM DEFAULTS:                                      *
*****************************************************************************************************/

// Matches specifications for Sensirion SCD30
#define crc_polynomial (0x31) // 0x31 (x^8 + x^5 + x^4 + 1).
// Matches specifications for Sensirion SCD30
#define initial_remainder (0xFF)


/********************************************************************************************
 *                          SENSIRION SCD30 I2C Macros:                                     *
 ********************************************************************************************/

// Sensirion Commands:
// "+ argument" indicates that the measurement command is sent along with
// additional data as an "argument."
#define Sensirion_Address (0x0061)
// baud rate = SMCLK / 0x140; 1MHz / 20 = 50 kHz, maximum for Sensiron sensor
#define Sensirion_Baud_Rate (0x0014)
#define Start_Continuous_Measurement_CMD (0x0010)      //+ argument
#define Stop_Continuous_Measurement_CMD (0x0104)       // No argument
#define Set_Measurement_Interval_CMD (0x4600)          // No argument
#define Get_Data_Ready_Status_CMD (0x0202)             // No argument
#define Read_Measurement_CMD (0x0300)                  // No argument
#define Toggle_Automatic_Self_Calibration_CMD (0x5306) //+ argument
#define Set_Forced_Recalibration_Value_CMD (0x5204)    //+ argument
#define Set_Temperature_Offset_CMD (0x5403)            //+ argument
#define Set_Altitude_Compensation_CMD (0x5102)         //+ argument
#define Read_Firmware_Version_CMD (0xD100)             // No argument
#define Soft_Reset_CMD (0xD304)                        // No argument

// Buffer Sizes:
// Command (CMD) + Argument (ARG) + CRC
#define I2C_Transmit_Buffer_Size (5)
// Size of a READ
#define I2C_Receive_Buffer_Size (18)
// NOTE: Set readings buffer size to 10 for faster data response. Set readings buffer to 15+ for smoother fluctuations and
// higher accuracy averages. This can be changed as needed.
#define I2C_Readings_Buffer_Size (10)

// Hardware Components for Quick Changing:
#define UCBxCTLW0 (UCB1CTLW0)
#define UCBxCTLW1 (UCB1CTLW1)
#define UCBxIE (UCB1IE)
#define UCBxBRW (UCB1BRW)
#define UCBxI2CSA (UCB1I2CSA)
#define UCBxI2COA0 (UCB1I2COA0)
#define UCBxIV (UCB1IV)
#define UCBxRXBUF (UCB1RXBUF)
#define UCBxTXBUF (UCB1TXBUF)
#define USCI_Bx_VECTOR (USCI_B1_VECTOR)

// Peak Times:
#define IS_TX (I2C_MODE_BYTE == 1)
#define IS_RX (I2C_MODE_BYTE == 2)
#define IS_READ (I2C_MODE_BYTE == 3)
#define CLR_I2C_MODE_BYTE (I2C_MODE_BYTE = 0) //0 is not transmitting.
#define SET_TX (I2C_MODE_BYTE = 1)
#define SET_RX (I2C_MODE_BYTE = 2)
#define SET_READ (I2C_MODE_BYTE = 3)
#define START_I2C_STOPWATCH (TRACK_I2C_TIME = true)
#define STOP_I2C_STOPWATCH (TRACK_I2C_TIME = false)
#define CLR_I2C_STOPWATCH (I2C_TIME = 0)

// Communication timeout value (in half-mils)
#define I2C_TIMEOUT (1000) // Timeout communication after 0.5 seconds of communication time.

/**
 * @brief Clarifies the LED state and simplifies handling.
 *
 */
//enum LED_STATE
//{
//    VALID_SET_FUNCT,
//    INVALID_SET_FUNCT,
//    GET_FUNCT,
//    VALID_READ,
//    INVALID_READ,
//    IN_TRANSMISSION,
//    NOT_IN_TRANSMISSION,
//    IN_SETUP
//};

// Bitwise Shifting:
#define One_Byte (8)
#define Two_Bytes (16)
#define Three_Bytes (24)
#define Four_Bytes (32)


///************************************************************
//* STANDARD BITS
//************************************************************/
//
//#define BIT0                   (0x0001)
//#define BIT1                   (0x0002)
//#define BIT2                   (0x0004)
//#define BIT3                   (0x0008)
//#define BIT4                   (0x0010)
//#define BIT5                   (0x0020)
//#define BIT6                   (0x0040)
//#define BIT7                   (0x0080)
//#define BIT8                   (0x0100)
//#define BIT9                   (0x0200)
//#define BITA                   (0x0400)
//#define BITB                   (0x0800)
//#define BITC                   (0x1000)
//#define BITD                   (0x2000)
//#define BITE                   (0x4000)
//#define BITF                   (0x8000)


#define USCIBxRMP (USCIB1RMP)


/********************************************************************************************
 *                                  Timer Macros:                                           *
 ********************************************************************************************/


// Hardware components for easy switching
#define TBxCTL (TB0CTL)
#define TBxCCR0 (TB0CCR0)
#define TBxCCTL0 (TB0CCTL0)
#define TIMERx_B0_VECTOR (TIMER0_B0_VECTOR)

// Timing flags
#define New_50_ms (TIMING_FLAG_BYTE & BIT0)
#define Set_New_50_ms (TIMING_FLAG_BYTE |= BIT0)
#define Clr_New_50_ms (TIMING_FLAG_BYTE &= (~BIT0))
#define New_Second (TIMING_FLAG_BYTE & BIT1)
#define Set_New_Second (TIMING_FLAG_BYTE |= BIT1)
#define Clr_New_Second (TIMING_FLAG_BYTE &= (~BIT1))

// The below timing flags can be used for logging purposes, if desired.
#define New_Ten_Second (TIMING_FLAG_BYTE & BIT2)
#define Set_New_Ten_Second (TIMING_FLAG_BYTE |= BIT2)
#define Clr_New_Ten_Second (TIMING_FLAG_BYTE &= (~BIT2))
#define New_Minute (TIMING_FLAG_BYTE & BIT3)
#define Set_New_Minute (TIMING_FLAG_BYTE |= BIT3)
#define Clr_New_Minute (TIMING_FLAG_BYTE &= (~BIT3))
#define New_Hour (TIMING_FLAG_BYTE & BIT4)
#define Set_New_Hour (TIMING_FLAG_BYTE |= BIT4)
#define Clr_New_Hour (TIMING_FLAG_BYTE &= (~BIT4))

// 1 MHz = 1000000 cycles / 1 second --> 1000 cycles/ 1 millisecond --> 500 cycles = 0.5 ms
#define HALF_MIL_PERIOD (500)


/********************************************************************************************
 *                                   UART Macros:                                           *
 ********************************************************************************************/

//Pulse "Start TX" code
// #define UART_PULSE_CODE_ELEMENT0 0x27
// #define UART_PULSE_CODE_ELEMENT1 0x00
// #define UART_PULSE_CODE_ELEMENT2 0x02


//UART commands sent from the Pulse
#define UART_READ_DATA_COMMAND (0x00)                             //No argument
#define UART_SET_MEASUREMENT_INTERVAL_COMMAND (0x01)              //+ argument
#define UART_GET_MEASUREMENT_INTERVAL_COMMAND (0x02)              //No argument
#define UART_SET_FORCED_RECALIBRATION_COMMAND (0x03)              //+ argument
#define UART_GET_FORCED_RECALIBRATION_COMMAND (0x04)              //No argument
#define UART_TOGGLE_AUTOMATIC_SELF_CALIBRATION_COMMAND (0x05)     //+ argument
#define UART_GET_AUTOMATIC_SELF_CALIBRATION_STATUS_COMMAND (0x06) //No argument
#define UART_SET_TEMPERATURE_OFFSET_COMMAND (0x07)                //+ argument
#define UART_GET_TEMPERATURE_OFFSET_COMMAND (0x08)                //No argument
#define UART_SET_ALTITUDE_COMPENSATION_COMMAND (0x09)             //+ argument
#define UART_GET_ALTITUDE_COMPENSATION_COMMAND (0x0A)             //No argument
#define UART_READ_FIRMWARE_VERSION_COMMAND (0x0B)                 //No argument
#define UART_SOFT_RESET_COMMAND (0x0C)                            //No argument
#define UART_SET_AMBIENT_PRESSURE_COMMAND (0x0D)                  //+ argument
#define UART_GET_DATA_MIN_MAX_COMMAND (0x0E)                      //No argument
#define UART_CLR_DATA_MIN_MAX_COMMAND (0x0F)                      //No argument

// Buffer Sizes
// '2' + 'O' + 'C' + ':' + MESSAGE_LENGTH + CMD + CRC0 + ARG_MSB + ARG_LSB + CRC1 + '\r'
#define UART_RX_BUFFER_SIZE (17)
//'C' + 'O' + '2' + ':' + MESSAGE_LENGTH + CMD + CRC0 + CO2_MSB + CO2_LSB + CRC1 + T + RH + CRC2 + '\r'
#define UART_TX_READ_BUFFER_SIZE (23)
// 'C' + 'O' + '2' + ':' + MESSAGE_LENGTH + CMD + CRC0 + ARG_MSB + ARG_LSB + CRC1 + '\r'
#define UART_TX_COMMAND_BUFFER_SIZE (17)


// UART Flag Macros
#define UART_TX_EMPTY (UART_FLAG_REG & UCTXIFG)
#define UART_RX_FULL (UART_FLAG_REG & UCRXIFG)
#define CLR_UART_TX_EMPTY (UART_FLAG_REG &= ~UCTXIFG)
#define CLR_UART_RX_FULL (UART_FLAG_REG &= ~UCRXIFG)
#define CHIP_TRANSMIT (P1OUT |= BIT6)
#define CHIP_LISTEN (P1OUT &= ~BIT6)
#define CHIP_TRANSMITING (P1OUT & BIT6)
#define SET_UART_PARSE_RX_FLG (UART_PARSE_RX_FLG = true)
#define CLR_UART_PARSE_RX_FLG (UART_PARSE_RX_FLG = false)
#define START_UART_STOPWATCH (TRACK_UART_TIME = true)
#define STOP_UART_STOPWATCH (TRACK_UART_TIME = false)
#define CLR_UART_STOPWATCH (UART_TIME = 0)
#define CMD_RESET_STATE (0xFF)
#define RESET_CMD (CMD = CMD_RESET_STATE) // 0xFF is an unused command, keep this as a reset state.
#define CMD_IS_RESET (CMD == CMD_RESET_STATE)
#define SET_TX_READ (TX_READ = true)
#define CLR_TX_READ (TX_READ = false)


// RX Enable Macros
#define ENABLE_UART_RX (UART_IE_REG |= UCRXIE)
#define DISABLE_UART_RX (UART_IE_REG &= ~UCRXIE)

// TX Enable Macros
#define ENABLE_UART_TX (UART_IE_REG |= UCTXIE)
#define DISABLE_UART_TX (UART_IE_REG &= ~UCTXIE)


// TXBUF should be automatically cleared.
#define UART_SEND_SETUP (CHIP_TRANSMIT, DISABLE_UART_RX, UART_TX_BUFFER_PTR = 0, ENABLE_UART_TX) // UART_TX_BUF = 0,
#define UART_RECEIVE_SETUP (CHIP_LISTEN, DISABLE_UART_TX, UART_RX_BUFFER_PTR = 0, ENABLE_UART_RX) // UART_RX_BUF = 0,

// Communication timeout value (in half-mils)
#define UART_TIMEOUT (2000) // Timeout communication after 1 second(s) of communication time.

// Delays
#define Lazy_Wait(X) for (lazy_timer = 0; lazy_timer < X; lazy_timer++)

// Main() UART CMD Macros:
// Used to flip flop between TX and RX for main() UART CMDs
#define TOGGLE_UART_TX_RX_FLIP_FLOP (UART_TX_RX_FLIP_FLOP = ~UART_TX_RX_FLIP_FLOP)
// Used to skip READs in that 250 ms window in MAIN()
#define SKIP_READ (data_ready_timing_byte = 6)
// Parses UART ARG MSB
#define GET_UART_ARG_MSB (ARG_MSB = ((uint8_t) (ARG >> One_Byte)))
// Parses UART ARG LSB
#define GET_UART_ARG_LSB (ARG_LSB = ((uint8_t) ARG))
// Indicates that UART_CMD_BUFFER is ready to be filled.
#define SET_UART_FILL_CMD_BUFFER_FLG (UART_FILL_CMD_BUFFER_FLG = true)
// Indicates that UART_CMD_BUFFER is not ready to be filled.
#define CLR_UART_FILL_CMD_BUFFER_FLG (UART_FILL_CMD_BUFFER_FLG = false)
// Automates UART CMD TX settings (250 ms wait time)
#define UART_CMD_TX_PRESETS (CLR_TX_READ, TOGGLE_UART_TX_RX_FLIP_FLOP, SKIP_READ)
// Automates UART CMD RX settings (250 ms wait time)
#define UART_CMD_RX_PRESETS (SET_UART_FILL_CMD_BUFFER_FLG, TOGGLE_UART_TX_RX_FLIP_FLOP, SKIP_READ)
// Automates UART CMD BUFFER settings (250 ms wait time)
#define UART_CMD_FILL_BUFFER_PRESETS (RESET_CMD, CLR_UART_FILL_CMD_BUFFER_FLG, UART_SEND_SETUP, GET_UART_ARG_MSB, GET_UART_ARG_LSB)
// Automates UART CMD SET function settings
#define UART_CMD_SET_PRESETS (CLR_TX_READ, RESET_CMD, SKIP_READ)

// Pre-processor build settings
#ifdef USE_UART_0
// UCA0
#define UART_TX_BUF UCA0TXBUF
#define UART_RX_BUF UCA0RXBUF
#define UART_FLAG_REG UCA0IFG
#define UART_IE_REG UCA0IE
#define UART_CTLW0 UCA0CTLW0
#define UCAxIV UCA0IV

#else
// UCA1
#define UART_TX_BUF UCA1TXBUF
#define UART_RX_BUF UCA1RXBUF
#define UART_FLAG_REG UCA1IFG
#define UART_IE_REG UCA1IE
#define UART_CTLW0 UCA1CTLW0
#define UCAxIV UCA1IV


#endif /* Pre-processor build settings*/


#endif /* INIT_VAR_H_ */
