/*
 * config.h
 *
 *  Created on: Dec 23, 2021
 *      Author: HP
 */

#ifndef MAIN_CONFIG_H_
#define MAIN_CONFIG_H_

//MSP libs
//#include <msp430.h>
//#include <stdint.h>
//#include "driverlib.h"
//#include "error_codes.h"
//#include "hw_config.h"
//#include "main.h"

//#include "sdkconfig.h"
//#include "driver/spi_master.h"
//#include "driver/gpio.h"
//#include "adc1.h"
//#include "bit_defs.h"

// +-------------------------+
// | Macros                  |
// +-------------------------+

// Choose which config, comment out others
//#define AI_THINKER_S3		//enable config for AI Thinker ESP32S3 board
//#define ESP32_S3_DEVKITC	//enable config for Espressif DevkitC-1 demo board, modified to match AIT where possible (all but IO33-34)
//#define SMT_1120MICRO_REVB	//enable config for Bartlett SMT_1120MICRO Rev B board - uses ESP32-S3, internal A2D
//#define SMT_1120MICRO_REVC	//enable config for Bartlett SMT_1120MICRO Rev C board - uses ESP32-S2, external A2D for TC and board temp

// other configuration options
//#define ENABLE_MULTI_ZONE
//#define ENABLE_OUTPUT_4
//#define ENABLE_LOG_GRAPH
//#define SKUTT_VERSION
//#define BIG_GENESIS		//standard Genesis
//#define LITTLE_GENESIS		//Genesis Mini or Micro
//#define MICRO_GENESIS		//Genesis Micro
//#define SINGLE_SELECT_MENU	//menu list has a single item selection

//macros for various special functions and options
#define ENABLE_WDT			// enables watch dog timer

//macros for various special functions and options
//#define SMALL_DISPLAY		// SMALL_DISPLAY for 2.8 or 3.5 in, LARGE_DISPLAY for 5.0 in
//#define PORTRAIT_MODE		// PORTRAIT_MODE rotates display 90 degrees


// ----------------------------------------------------------
// Standard Genesis 2.0+ hardware
// ----------------------------------------------------------

//spi port registers
#define SPI_RX_L     	UCB0RXBUF_L //rx reg using spi 1
#define SPI_TX_L     UCB0TXBUF_L //tx reg using spi 1
//#define SPI_RX_L     UCB0RXBUF_L //rx reg using spi 1
#define SPI_IFG_L        UCB0IFG //ifg reg using spi 1
#define SPI_STAT_L       UCB0STATW   //stat reg using spi 1
#define Read_Null	0

 // UPDATED FOR SENSOR BD

//50   P2.7   EE /CS
#define EE_Enable_CSlo      P3OUT &= ~BIT6 // EEPROM CS clear  UPDATED FOR SENSOR BD
#define EE_Disable_CShi     P3OUT |= BIT6       // EEPROM CS set  UPDATED FOR SENSOR BD
//pin 59 p4.0
#define Flash_Enable_CSlo      P3OUT &= ~BIT7 // FLASH CS clear  UPDATED FOR SENSOR BD
#define Flash_Disable_CShi     P3OUT |= BIT7       // FLASH CS set  UPDATED FOR SENSOR BD

//pin 60 p4.1
#define Wifi_Reset_lo        P3OUT &= ~BIT5 // FLASH CS clear           UPDATED FOR SENSOR BD
#define Wifi_Reset_hi        P3OUT |= BIT5      // FLASH CS set         UPDATED FOR SENSOR BD

// 1 MHz = 1000000 cycles / 1 second --> 1000 cycles/ 1 millisecond --> 500 cycles = 0.5 ms
#define HALF_MIL_PERIOD (500)


//needs macro   54  P3.3    /(X/R) DISP
#define SP483_Transmit		P7OUT |= BIT2  //KISS X/R P HIGH TO XMIT
#define SP483_Listen		P7OUT &= ~BIT2  //KISS X/R P LOW TO RCV
#define SCI_Reciever_Full	UCA1IFG & BIT0  //0,UCA1IFG

                                                //46  p2.5    NC
                                                //47  P2.6    NC
                                                //48  P2.7    NC
                                                //49  P3.0/BSL_TX NC
                                                //50  P3.1/BSL_RX NC
//                                                //
////42  P2.0   buzzer
//#define	Buzzer_Output_Toggle	P2OUT ^= BIT0                          // Toggle P3.3
//#define	Buzzer_Output_Off		P2OUT |= BIT0	//buzzer is active low
//#define	Buzzer_Output_On		P2OUT &= ~BIT0	//buzzer is active low
//
////43  P2.1   output3
//#define Output_3_On     P2OUT |= BIT1    // turns on output 3
//#define Output_3_Off    P2OUT &= ~BIT1    // turns off output 3
//#define Output_3_Set    P2OUT &  BIT1           // determines if output 3 is on
//
//
////44   P2.2   output2
//#define Output_2_On     P2OUT |= BIT2    // turns on output 2
//#define Output_2_Off    P2OUT &= ~BIT2
//#define Output_2_Set    P2OUT &  BIT2           // determines if output 2 is on
//
////45  P2.3   output1
//#define Output_1_On     P2OUT |= BIT3    // turns on output 1
//#define Output_1_Off    P2OUT &= ~BIT3    // turns off output 1
//#define Output_1_Set    P2OUT &  BIT3           // determines if output 1 is on
//
////47   P2.4   output4
//#define Output_4_On     P2OUT |= BIT4    // turns on output 4
//#define Output_4_Off    P2OUT &= ~BIT4    // turns off output 4
//#define Output_4_Set    P2OUT &  BIT4             // determines if output 4 is on

//48   P2.5    LED for monitor only
#define LED_1_On     P4OUT |= BIT4    // turns on cap coupled (safety)
#define LED_1_Off    P4OUT &= ~BIT4    // turns off cap coupled (safety)
#define LED_1_Toggle   P4OUT ^= BIT4      // Toggle P7.3
//#define Cap_Couple_Disable  P4OUT &= ~BIT6
//48   P2.5    LED for monitor only
#define LED_2_On     P4OUT |= BIT5    // turns on cap coupled (safety)
#define LED_2_Off    P4OUT &= ~BIT5    // turns off cap coupled (safety)
#define LED_2_Toggle   P4OUT ^= BIT5      // Toggle P7.3
//#define Cap_Couple_Disable  P4OUT &= ~BIT6
//48   P2.5    LED for monitor only
#define LED_3_On     P4OUT |= BIT6    // turns on cap coupled (safety)
#define LED_3_Off    P4OUT &= ~BIT6    // turns off cap coupled (safety)
#define LED_3_Toggle   P4OUT ^= BIT6      // Toggle P7.3
//#define Cap_Couple_Disable  P4OUT &= ~BIT6

                                                //57   P4.0   out4 input
    //49  P2.6    type S
//#define Type_S_Input   !( P2IN &  BIT6)             // pulled low WHEN set to S or R, must match status bit Type_S_Thermocouple STATUS_BYTE_5 & Bit7
    //59   P4.2   type R
//#define Type_R   !( P6IN &  Bit2)             // pulled low WHEN set to R
    //60   P4.3   type K
//#define Type_K   !( P6IN &  Bit1)             // pulled low WHEN set to K


//
//    //93  P8.0    LCD/RST;ODD5
//#define LCD_Reset           P8OUT &= ~BIT0     //P5.0
//#define LCD_Release_Reset   P8OUT |= BIT0
//    //94  P8.1    LCD_D/C;EVEN5
//#define LCD_Command     P8OUT &= ~BIT1      //P7.3 PTED &= Bit3_Clear
//#define LCD_Data        P8OUT |= BIT1           // PTED |= Bit3_Set
//
//   //71  P5.4    LCD_/CS;ODD1
//#define LCD_Enable_CSlo     P5OUT &= ~BIT5     // TI P7.4 PTCD &= Bit4_Clear
//#define LCD_Disable_CShi    P5OUT |= BIT5           //   PTCD |= Bit4_Set
//     //36  P1.1    TP_/CS;EVEN1
//#define TS_Enable_CSlo      P1OUT &= ~BIT1 // touch screen CS clear
//#define TS_Disable_CShi     P1OUT |= BIT1       // touch screen CS set

//
////needs mac????                                                //89  P7.6    SD_/CS;ODD2
//      //90  P7.7    LCD /WR;EVEN2
////#define LCD_Command     P7OUT &= Bit7_Clear      //P7.3 PTED &= Bit7_Clear
////#define LCD_Data            P7OUT |= BIT7           // PTED |= Bit7_Set
////40 P1.6
//#define LCD_WR_low           P1OUT &= ~BIT6      //P7.3 PTED &= Bit7_Clear
//#define LCD_WR_hi            P1OUT |= BIT6           // PTED |= Bit7_Set
//
////72  P5.5    LCD /RD
//#define LCD_RD_low           P5OUT &= ~BIT4      //P8.0 PTED &= Bit0_Clear
//#define LCD_RD_hi            P5OUT |= BIT4           // PTED |= Bit0_Set
////for 67461 sd card /cs is port 5 bit 3
////needs mac                                               //91  P8.0    LCD /RD;ODD3
////needs mac                                                //92  P8.1    BackLite on/off;EVEN3
//needs mac????                                               //93  P8.2    BLE /CS;ODD4
//41  P1.7    STOP switch input
#define Stop_Button_Pressed    P1IN &  BIT7             // pulled low if stop key pressed, check in half_mil
                                                //95  SBWTCK
                                                //96  TDO
                                                //97  TDI
                                                //98  TMS
                                                //99  TCK
                                                //100 /RST

#define output_1_logic_level    (P4IN & BIT6)  //PIN 65
#define output_2_logic_level    (P4IN & BIT5)  //PIN 64
#define output_3_logic_level    (P4IN & BIT4)  //PIN 63
#define output_4_logic_level    (P4IN & BIT7)  //PIN 66


#define Connected_to_cal_is_0        !(P5IN &= BIT7)
#define Calibration_Offset_Voltage  P5OUT &= ~BIT6
#define Calibration_Scale_Voltage   P5OUT |= BIT6


// On-chip analog input channels  ***** needs to be configured *****
    //channel =0 is output3 volts
    //         1 is output2 volts
    //         2 is output1 volts
    //         3 is safety volts
    //         4 is amps
    //         5 is cjc
#define SAFETY_IN_ADC_CHANNEL	ADC_CHANNEL_3
#define OUT1_IN_ADC_CHANNEL		ADC_CHANNEL_2


////////////////////  error value macros

#define Set_WDT                 ERROR_VALUE = 39

//#define WDTPW  0X5A00  // PASSWORD THAT HAS TO BE PART OF ANY WRITE TO WATCHDOG
//#define WDTHOLD  0X0080  // SET BIT 7 TO HOLD WATCHDOG

// ----------------------------------------------------------



//#endif //end if CONFIG_IDF_TARGET_ESP32S3


// +-------------------------+
// | Global variables        |
// +-------------------------+

//extern spi_device_handle_t TOUCH_SPI;	//touch screen spi handle
//extern spi_device_handle_t EEPROM_SPI;	//EEPROM spi handle
//extern spi_device_handle_t FLASH_SPI;	//flash spi handle


// +-------------------------+
// | Functions               |
// +-------------------------+

void ms_delay(uint16_t delays);
void set_wifi_reset_low (void);
void set_wifi_reset_high (void);
void init_hardware (void);
void soft_reset(void);


#endif /* MAIN_CONFIG_H_ */
