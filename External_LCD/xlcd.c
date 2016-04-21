/**********************************************************************
* 2009 Microchip Technology Inc.
*
* FileName:        	xlcd.c
* Dependencies:    Header (.h) files if applicable, see below
* Processor:       	PIC18F
* Compiler:        	MCC18 v3.30 or higher
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Incorporated ("Microchip") retains all 
* ownership and intellectual property rights in the code accompanying
* this message and in all derivatives hereto.  You may use this code,
* and any derivatives created by any person or entity by or on your 
* behalf, exclusively with Microchip's proprietary products.  Your 
* acceptance and/or use of this code constitutes agreement to the 
* terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS". NO 
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT 
* NOT LIMITED TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, 
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS 
* CODE, ITS INTERACTION WITH MICROCHIP'S PRODUCTS, COMBINATION WITH 
* ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE 
* LIABLE, WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR
* BREACH OF STATUTORY DUTY), STRICT LIABILITY, INDEMNITY, 
* CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
* EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR 
* EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE CODE, HOWSOEVER 
* CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE
* DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE BY LAW, 
* MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS
* CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP 
* SPECIFICALLY TO HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and 
* determining its suitability.  Microchip has no obligation to modify,
* test, certify, or support the code.
*
* REVISION HISTORY:
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Author        	Date      	Comments on this revision
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Harsha.J.M	04/04/09	First release of source file
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*
* ADDITIONAL NOTES:
* Code uses the Peripheral library support available with MCC18 Compiler
* Code Tested on:
* PicDem2+ demo board with PIC18F4685 controller
* The Processor starts with the External Crystal (8 Mhz).
*
**********************************************************************/
#include <p18cxxx.h>
#include "xlcd.h"

//-------------------------------Configuration setting ----------------------------------------------
/**
	* Oscillator is configured as HS
	* Fail safe monitor is enabled
	* watch dog timer is disabled
	* Extended instruction mode is disabled
	* oscillator switch over is enabled	
*/
#if defined(__18F4685)		//If the selected device if PIC18F4685, then apply below settings else user will have to set
#pragma config OSC=HS, FCMEN=ON, WDT=OFF, IESO=ON, XINST=OFF, LVP=OFF 
#endif

char XLCD_Disp1[] = " MICROCHIP      ";
char XLCD_Disp2[] = " PIC_UC       ";


void main(void)
{
unsigned char config=0xFF,i=0,addr=0,data=0;
ADCON1 = 0xFF;

	config = FOUR_BIT  & LINES_5X7;
//********  Configure LCD for four line communication and 5X7 line display *********************************
	OpenXLCD(config);		
		while( BusyXLCD() );		//wait untill LCD controller is busy

//*********** Set the starting address in the LCD RAM for display. This determines the location of display ********	
	SetDDRamAddr(0x80);
		while( BusyXLCD() );		//wait untill LCD controller is busy
	putsXLCD(XLCD_Disp1);			//Display string of text
		while( BusyXLCD() );		//wait untill LCD controller is busy

//********** Set the address in second line for display ****************************************************		
	SetDDRamAddr(0xC1);
		while( BusyXLCD() );		//wait untill LCD controller is busy
	putsXLCD(XLCD_Disp2);					//Display string of text
		while( BusyXLCD() );		//wait untill LCD controller is busy
	putcXLCD(0x5F);					//dsiplay some chrecter
	 	while( BusyXLCD() );		//wait untill LCD controller is busy
		 
	SetDDRamAddr(0x20);
		while( BusyXLCD() );		//wait untill LCD controller is busy

	while(1);					//end of program		
		
}	
	
