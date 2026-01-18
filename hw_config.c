/*
 * config.c
 *
 *  Created on: Feb 28, 2022
 *      Author: HP
 */
#include "error_codes.h"
#include "hw_config.h"
#include "main.h"
//MSP libs
#include <msp430.h>
//#include <driverlib.h>
#include <stdint.h>

//project libs
//#include "bi_lcd_spi_par.h"
//#include "adc_esp_23.h"
//#include "error_codes.h"

// +-------------------------+
// | Global variables        |
// +-------------------------+


// +-------------------------+
// | Functions               |
// +-------------------------+

// Utility delay function
//  delay: time to delay in mS
void ms_delay(uint16_t delay){// requires halfmil int enables so can't be used until config is complete

    DELAY_MILLISECOND = delay;

    while (DELAY_MILLISECOND){
        //DELAY_MILLISECOND is decremented in hafmil
    }

}  //end haf_mil_delay

// Set the wifi reset line low (hold in reset)
//  Needs to be a function to use it in the wifi library.
void set_wifi_reset_low (void){
	Wifi_Reset_lo;
}//end set_wifi_reset_low

// Set the wifi reset line high (release from reset)
//  Needs to be a function to use it in the wifi library.
void set_wifi_reset_high (void){
	Wifi_Reset_hi;
}//end set_wifi_reset_high

// Initialize the hardware according to the defined configuration
void init_hardware(void){
  //  uint16_t loops;

    WDTCTL = WDTPW | WDTHOLD; //watchdog powers up active so Stop watchdog timer

//  code for fr5964

//    Pmmctl0   = a540
//    15-8 = a5  password
//    7=0
//    6= 1, high side always checked
//    5 = 0 don’t care
//    4= 0 ldo reg on, in lpm3,4
//    3 = 0 normal op, 1 software reset
//    2 = 0 normal op, no BOR
//    0,1 =  0,0 reserved
//
  //2.3 PMM Registers
    PMMCTL0 = 0xA540;
   // PMMCTL1 is reserved so no settings dpb
    PM5CTL0 = 0x00;  // bit 0 set locks I/O in high imped mode
//
//            Locks I/O pin and other LPMx.5 relevant (for example, RTC) configurations upon
//            exit from LPMx.5.
//            This bit is set by hardware and must be cleared by software. It cannot be set by
//            software.
//            After a power cycle I/O pins are locked in high-impedance state with input
//            Schmitt triggers disabled until LOCKLPM5 is cleared by the user software.
//            After a wake-up from LPMx.5 I/O pins and other LPMx.5 relevant (for example,
//            RTC) configurations are locked in their states configured before LPMx.5 entry
//            until LOCKLPM5 is cleared by the user software.
//            0b = I/O pin and LPMx.5 configurations unlocked.
//            1b = I/O pin and LPMx.5 configuration remains locked.










/////////&&&&&&&&&&&&&  below is gen 3 code


  //  0,SFRIFG1   SFR is special function register
	if (SYSRSTIV == 0x0016) {  //if watchdog,    -- same code for msp430fr5964 to indicate wd time out
//		STATUS_BYTE_18 = 123;  // SET A BREAKPT
		SYSRSTIV = 0;  // CLR IN CASE WE GET ANOTHER WDT TIMEOUT
		//  set_new_mode (Op_Mode_Idle) ;// MODE=1; no interrupts yet so don't use, has EE prom routines

//		OP_MODE = 1;
		Set_WDT;  // sets error value to 39
	}

	/* Initialize DCO to 8.00MHz */
	// smclk and mclk to 1 mhz  reset condition

	CSCTL0 = 0xA500;     // Set PASS WORD TO enable all CS CTL registers
	CSCTL1 = 0x000C;    // same as reset value DCO FREQUENCY = 8M Hz
	CSCTL2 = 0x0033 ;  // ACLK is lfxt,  SMCLK SOURCE IS DCOCLK,  MCLK SOURCE IS DCO RESET CONDITION

	CSCTL3 = 0x0033 ;    //  33 is reset cond., aclk /1, smclk is source /8, mclk /8. so smclk and mclk = 1MHZ
	CSCTL4 = 0xCDC8;   // NOT RESET COND (cdc9 is reset) enables LFXT, highest drive, external crystal
	//CSCTL5  USE RESET COND.  hF AND LF COUNT DOWN ENABLED

	//CSCTL6  RESET COND MCLK, SMCLK AND ACLK CONDITIONAL REQUESTS ENABLED


    PJOUT = 0XFF;
    PJDIR = 0xdf;
    PJSEL1 = 0x00;
    PJSEL0 = 0x10;  // enables low frequency osc on msp430fr5964


	__delay_cycles(100000);                      // wait for clocks to stabilize





//set gpio ports  UNUSED GPIO SET TO OUTPUT


    P1OUT = 0X00;  // PULL UPS IF RESIS ENABLED AND SET TO INPUT
    P1IN = 0;
    P1DIR = 0xFF;  //all OUTPUTS or set by peripheral
    P1SEL0 = 0x07;  //1.7 UCA0 clock pin, 0,1,2 a2d inputs
    P1SEL1 = 0xC7;  //1.7 UCA0 clock pin, 0,1,2 a2d inputs
    //P1REN = 0x84;  ///resistor enable FOR P1.7, STOP SWITCH, P1.2 lid switch
   // P1DS = 0XFF;  //HIGH DRIVE STRENGTH
  //  P1IE = 0X00;  //IF P1.3 0X08; IS INTERRUPT
  //  P1IES = 0X00; //0X08 IS HIGH TO LOW TRANSITION TRIGGERS INT ON P1.3
    //P1IV            INTERRUPT VECTOR WORD
    //P1IFG         INTERRUPT FLAG BYTE

    //port 1
//
//    i.  P1.0  use as A0 analog input for thermistor, pin 1
//    ii. P1.1  use as A1 analog input for battery voltage divided by 4, pin 2
//    iii.    P1.2  use as A2 analog input spare for now, pin 3
//    iv. P1.3  input not used. Pin 12
//    v.  P1.4   input not used. Pin 13
//    vi. P1. 5 input not used. Pin 14
//    vii.    P1.6  ucb0 spi master out, slave in , pin 39
//    viii.   P1.7   ucb0 spi master in, slave out, pin 40
//
//



 //*********************************************************************************************************

    P2OUT = 0Xff;  //pull up on 2.6
    P2DIR = 0XfF;  //

    P2SEL0 = 0x00;  //
    P2SEL1 = 0x66;

   // P2DS = 0XFF;  //DRIVE STRENGTH HIGH
  //  P2IE = 0X00;  //NO INTERRUPT

//    i.  P2.0  uca0 txd, uP output to wifi module, pin 32
//    ii. P2.1   uca0 rxd, input to uP from wifi.  Pin 33
//    iii.    P2.2   UCB0 SCK                                            pin 34
//    iv. P2.3  NOT USED, INPUT,                             pin 51
//    v.  P2.4  NOT USED, INPUT,                             pin 52
//    vi. P2.5  UCA1 TXD, uP output to rs485     pin 28
//    vii.    P2.6  UCA1 RXD, uP input from rs485  pin 29
//    viii.   P2.7   INPUT, NOT USED                             pin 50
//




    P3OUT = 0X0F;   // PULL UP RESISTORS ON P3.3 - P3.0
    P3DIR = 0XF0;  //P3.3 OUTPUT FOR X/R KISS
   // P3DS = 0XFF;  //DRIVE STRENGTH HIGH
    P3SEL0 = 0X00;  //SELECT
    P3SEL1 = 0X00;  //SELECT
    P3REN = 0X00F; //ENABLE P3.0-P3.3 AS BCD INPUT

 //P3.0 TO P3.7 DATA BUS LOW
    //*********************************************************************************************************


    P4OUT = 0X00;

    P4DIR = 0XF3; //bit 0 is flash cs, bit 1 is wifi reset
	//P4DS = 0XFF;  //DRIVE STRENGTH HIGH
	P4SEL0 = 0X00;  //
	P4SEL1 = 0X00;  //
	//P4REN = 0X00;
       //100 /RST


//  P4.7 TO P4.0 DATA BUS HIGH
    //*********************************************************************************************************



    P5OUT = 0X00;            //PULLUP RESISTORS
    P5DIR = 0XFF;           //P5+P6 IS DATA BUS TO DISPLAY
     P5SEL0 =0X03;    //
     P5SEL1 = 0X00;  //
   //  P5REN = 0X80;              ///ENABLE RESISTORS
   //  P5DS = 0XFF;  //DRIVE STRENGTH HIGH
     //    72  P5.7   CONNECTED TO CAL INPUT
     //    71  P5.6    0-50mV output
     //    70  P5.5    lcd /rd
     //    69  P5.4    lcd  /cs
     //    68  P5.3    sd /cs
     //    67  P5.2    odd 3
     //    66  P5.1    odd 2
     //    65  P5.0    odd  1
 //************************************************************************************************

    P6OUT = 0X00;
    P6DIR = 0XFF;
    P6SEL0 = 0;
    P6SEL1 = 0;
  //  P6REN = 0X00;
   // P6DS = 0XFF;  //DRIVE STRENGTH HIGH
         //       P6.7 6.0 data bus low
         //
     //*********************************************************************************************************

    P7OUT = 0X00;
    P7DIR = 0xFF;  //P7.7 7.0 data bus HIGH
    //  P7SEL = P7SEL_INIT;
    P7REN = 0X00;
  //  P7DS = 0XFF;  //DRIVE STRENGTH HIGH

//*********************************************************************************************************

    P8OUT = 0X00;
    P8DIR = 0XFE;
      P8SEL0 = 0;
      P8SEL1 = 0;
   // P8REN = 0X00;  //
   // P8DS = 0XFF;  //DRIVE STRENGTH HIGH

    //       P8.1    lcd  d/c
    //       P8.0    lcd  /rst   1

    //*********************************************************************************************************
//
//    P9OUT = 0XFF;
//    P9DIR = 0xF1;  //P9.0 is eeprom /cs
//    P9SEL = 0x0e;  //p9.1,.2,.3 are a2d inputs
//    P9REN = 0X00;
//    P9DS = 0XFF;  //DRIVE STRENGTH HIGH
    //    13  P9.3, A3    Analog inputs   0   3,P9SEL
    //    12  P9.2, A4    Analog inputs   0   2,P9SEL
    //    11 P9.1, A5    Analog inputs   0   1,P9SEL
    //    42   P9.0   EE /CS  1

    //*********************************************************************************************************
//  moved to bottom of Clock System control registers to get LFXT enabled
//    PJOUT = 0XFF;
//    PJDIR = 0;
//    PJREN = 0XFF;
    //*********************************************************************************************************


      /*
          * 1 MHz SMCLK clock frequency on the MS430FR2476 development kit
          * Want to count to 500 microseconds = 500 clock ticks.
          * Each clock tick is 1 microsecond.
          */

         // Clear timer
         TA0CTL |= TACLR;
         // Enable interrupt, output mode 7, set to compare mode
         TA0CCTL0 = (TA0CCTL0 | CCIE | OUTMOD_7) & (~CAP);
         // Output mode 7, set to compare mode
         TA0CCTL1 = (TA0CCTL1 | OUTMOD_7) & (~CAP);
         // HALF_MIL_PERIOD==500, 0-499 is 500 microseconds
         TA0CCR0 = HALF_MIL_PERIOD - 1;
         // Interrupt enable, Upmode, SMCLK,
         TA0CTL = TA0CTL | TAIE | MC_1 | TASSEL_2;
  //       __enable_interrupt();







//    // Setup TA0
//    TA0CCTL0 = 0x00f0; // CCR0 interrupt enabled and compare 0 output mode = 7 toggle
//    TA0CCTL1 = 0x00e0;                      //  compare 1 output mode = 7 toggle
////    TA0CCR0 = 6100;                     //6100 for.5mS interrupt, smclk = 24Mhz, /2; 3050 for .25ms
//    TA0CCR0 = 6296;                     //6100 for.5mS interrupt, smclk = 24Mhz, /2; 3050 for .25ms
//    TA0CCR1 = 3000; //.5mS interrupt, smclk = 24Mhz, /2 needed for 50% duty cycle
//    TA0CTL = 0x0250;       // smCLK, /2, upmode, no TAIFG int

    //timera0 output defaults to p1.0 output

//*****************************************************************************************************
//set up spi on UCB0 to write to EE, FLASH for sensor only board

//*****************************************************************************************************
    UCB0CTL1 = 0xc1;   //select smclk (1 mhz FOR SENSOR ONLY) as br source AND IN RESET
    UCB0CTL0 = 0x69;
    //bit 7=0 phase 0 - TI phase 0 is same as freescale phase 1
    //    6=1 pol 1 , clock idles high
    //    5=1   msb out first
    //    4=0 8 bits,
    //    3=1 master
    //    2,1 00 3wire spi
    //    0=1 sync mode for spi

               UCB0BRW_L = 0x02;  // FOR SENSOR ONLY SMCLK = 1MHZ FOR DIVIDE BY 2 FOR 500K SPI CLK
               UCB0BRW_H = 0x00;
//
//                UCB0BRW_L = 0x30;   //DIVIDE SMCLK BY b= /96  5 is /6 , 1 is divide by 2 CLOCK SIGNAL VIEWED WITH SCOPE - Vlo IS ABOVE .4V UNTIL DIVIDE BY 6
//                UCB0BRW_H = 0x00;  //BAUD RATE IS .5M
////  UCA0BRW_L = 0x0b;//DIVIDE SMCLK BY b= /12  5 is /6 , 1 is divide by 2 CLOCK SIGNAL VIEWED WITH SCOPE - Vlo IS ABOVE .4V UNTIL DIVIDE BY 6
//  UCA0BRW_H = 0x00;  //BAUD RATE IS 2M
//              UCA0BRW_L = 0x06;   //DIVIDE SMCLK BY b= /6  5 is /6 , 1 is divide by 2 CLOCK SIGNAL VIEWED WITH SCOPE - Vlo IS ABOVE .4V UNTIL DIVIDE BY 6
//              UCA0BRW_H = 0x00;  //BAUD RATE IS 4M
//  UCA0BRW_L = 0x03;   //DIVIDE SMCLK BY b= /3  5 is /6 , 1 is divide by 2 CLOCK SIGNAL VIEWED WITH SCOPE - Vlo IS ABOVE .4V UNTIL DIVIDE BY 6
//  UCA0BRW_H = 0x00;  //BAUD RATE IS 8M

    UCB0CTL1 = UCB0CTL1 & 0xFE;   // RESET RELEASED

//*****************************************************************************************************
//    I2C FOR SENSIRION  SCD30  CO2, HUM, TEMP ON UCB1 for sensor only board
//*****************************************************************************************************

     // UCB1CTLW0_L
    UCB1CTLW0 = 0X0FC1;
 //   UCB1CTL1 = 0xc1;   //select smclk (1mhz) as br source AND IN RESET
 //   UCB1CTL0 = 0x0F;    //3 master, 2,1 I2C, 0 sync mode
   // UCB1CTLW1  STAYS AS RESET VALUES

    UCB1BRW_L = 0x14;//desired is 50K  Divide 1M by 20 decimal  = 0x14
    UCB1BRW_H = 0x00;  //BAUD RATE


    UCB1CTLW0 &= 0xFFFE;
    //UCB1CTL1 = UCA0CTL1 & 0xFE;   // RESET RELEASED

//*****************************************************************************************************

// set up UART for wifi ON UCA0 for sensor only board

//***********************************************************************************************

//                  UCA0CTL1 =0xC1;    //SMCLK, KEEP IN RESET
//                  P2SEL = 0x2f;  //MAP PINS TO SPI BUS and I2C INSTEAD OF GPIO
//                  UCA0CTL0 = 0x0F;   //MASTER, I2C, SYNC
//                  UCA0CTLW1 = 0X0008;  //AUTOMATIC STOP AFTER XMIT # OF BYTES

//                  UCA0BRW = 0x00FA;  // DIVIDE BY 250 96000 BAUD.  24MHZ / 250
//                  UCA0BRW = 240;  // DIVIDE BY 240 100k BAUD.  24MHZ / 240
//
//                  UCA0CTL1 =  UCA0CTL1 & 0xFE;   // RESET RELEASED

//
//    //*****************************************************************************************************
//	//set up radio UART on UCA1, 9600 BAUD  for sensor only board
//
//	//   ****************************************************************************************************
//	UCA1CTLW0 = UCA1CTLW0 | 0x0001;   // RESET set so setting can be changed
//	UCA1CTLW0 = 0x00c1; //select smclk (24mhz) as br source AND IN RESET 20c1 is msb first, 00c1 is lsb first
//	//  UCA1CTL0 = 0x69;
//	UCA1CTLW1 = 0x03;  // 03 SETS DEGLITCH TO 200nS
//
//	UCA1BRW = 161; //DIVIDE SMCLK N= 24MHZ / 9600 BAUD = 2500.  N>16 SO SET UCA1BRW TO INT (N/16) = 156
//	//HAD TO USE TRIAL AND ERROR TO GET BAUD RATE. USED LINK BD INTO 485 TO USB TO XCTL TO VIEW DATA
//	//STARTED UCA1BRW = 156, GETTING ERROR FREE DATA AT 172, 185, 190 OK, ERROR AGAIN AT 191
//
//	UCA1MCTLW = 0X0041; //SEE PAGE 990 IN USER GUIDE
//
//	UCA1CTLW0 = UCA1CTLW0 & 0xFFFE;   // RESET RELEASED
//
//	SP483_Listen;                // RS485 TO LISTEN
//
//	//    0,UCA1IFG SET INDICATES NEW DATA IN UCA1RXBUF
//

    //*****************************************************************************************************
    //  //set up I2C on UCB2, for SHT45 humidity sensor on sensor only board
    //
    //  //   ****************************************************************************************************
    //








//    //*****************************************************************************************************
//	//set up UART on UCA2, 9600 BAUD  Wifi
//
//	//   ****************************************************************************************************
//	UCA2CTLW0 = UCA2CTLW0 | 0x0001;   // RESET set so settings can be changed
//	UCA2CTLW0 = 0x00c1; //select smclk (24mhz) as br source AND IN RESET 20c1 is msb first, 00c1 is lsb first
//	//  UCA1CTL0 = 0x69;
//	UCA2CTLW1 = 0x03;  // 03 SETS DEGLITCH TO 200nS
//
////	UCA2BRW = 161; //DIVIDE SMCLK N= 24MHZ / 9600 BAUD = 2500.  N>16 SO SET UCA1BRW TO INT (N/16) = 156
////	UCA2BRW = 40; //38400 scaled the same as the 9600 setting
//	UCA2BRW = 13; //115200 scaled the same as the 9600 setting
//	UCA2MCTLW = 0X0041; //SEE PAGE 990 IN USER GUIDE
////	UCA2BRW = 106; //230400
////	UCA2BRW = 53; //460800 scaled the same as the 230400 setting
////	UCA2MCTLW = 0X0040; //SEE PAGE 990 IN USER GUIDE
//	//HAD TO USE TRIAL AND ERROR TO GET BAUD RATE. USED LINK BD INTO 485 TO USB TO XCTL TO VIEW DATA
//	//STARTED UCA1BRW = 156, GETTING ERROR FREE DATA AT 172, 185, 190 OK, ERROR AGAIN AT 191
//
//
//	UCA2CTLW0 = UCA2CTLW0 & 0xFFFE;   // RESET RELEASED
//
//	UCA2IE = UCRXIE;	//enable receive interrupt
//
////***********************************************************************************************

//set up 24 bit converter - clock source smclk/12 for 1mhz

//*****************************************************************************************************


//*****************************************************************************************************



//**********************  set 10 bit a2d  ************************************
    //PICK SMCLK/6 FOR 4MHZ CLOCK, DO SINGLE CONVERSION ON 1 INPUT,
//
//    ADC10CTL0_H = 0X0f;  //sample time
//    ADC10CTL0_L = 0X10;  //
//
//    ADC10CTL1_H = 0X02;   //
//    ADC10CTL1_L = 0Xd8;  // /by, clock source
//
//    ADC10CTL2_H = 0X00;
//    ADC10CTL2_L = 0X10;
//
//    ADC10MCTL0 =0X10; //SELECT ONBOARD VREFBITS 4-7 =1 ,BITS 4-7 =0  VREF =VCC
//
//    REFCTL0_L =0X81;  //1.5V INTERNAL REFERENCE

}//end init_hardware

void soft_reset(void){

    //trigger a software brownout reset
    PMMCTL0 |= PMMSWBOR;

} // end process_ts_install

