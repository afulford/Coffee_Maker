/**
  TMR0 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    tmr0.c

  @Summary
    This is the generated driver implementation file for the TMR0 driver using MPLAB(c) Code Configurator

  @Description
    This source file provides APIs for TMR0.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - v3.00
        Device            :  PIC18LF46K22
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.20
*/

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */

/**
  Section: Included Files
*/

#include <xc.h>
#include "tmr0.h"
#include "globalHeader.h"

void updateClock(void);
void checkUserTimers(void);
void updateClockOutput(char);
void checkClockForTarget(void);

/**
  Section: Global Variables Definitions
*/
volatile uint8_t timer0ReloadVal8bit;

/**
  Section: TMR0 APIs
*/


void TMR0_Initialize(void)
{
    // Set TMR0 to the options selected in the User Interface

    // T0PS 1:256; T08BIT 8-bit; T0SE Increment_hi_lo; T0CS T0CKI; TMR0ON enabled; PSA assigned; 
    T0CON = 0xF7;

    // TMR0H 0; 
    TMR0H = 0x00;

    // TMR0L 217; 
    TMR0L = 0xD9;

    // Load TMR0 value to the 8-bit reload variable
    timer0ReloadVal8bit  = 217;

    // Clear Interrupt flag before enabling the interrupt
    INTCONbits.TMR0IF = 0;

    // Enabling TMR0 interrupt.
    INTCONbits.TMR0IE = 1;

    // Start TMR0
    TMR0_StartTimer();
}

void TMR0_StartTimer(void)
{
    // Start the Timer by writing to TMR0ON bit
    T0CONbits.TMR0ON = 1;
}

void TMR0_StopTimer(void)
{
    // Stop the Timer by writing to TMR0ON bit
    T0CONbits.TMR0ON = 0;
}


uint8_t TMR0_Read8bitTimer(void)
{
    uint8_t readVal;

    // read Timer0, low register only
    readVal = TMR0L;

    return readVal;
}

void TMR0_Write8bitTimer(uint8_t timerVal)
{
    // Write to the Timer0 registers, low register only
    TMR0L = timerVal;
 }

void TMR0_Reload8bit(void)
{
    //Write to the Timer0 register
    TMR0L = timer0ReloadVal8bit;
}


void TMR0_ISR(void)
{

    // clear the TMR0 interrupt flag
    INTCONbits.TMR0IF = 0;

    // reload TMR0
    TMR0L = timer0ReloadVal8bit;


// **************************************************
    // Update Clock and possibly set updateLCD flag
    updateClock();
    checkClockForTarget();
    // Read Inputs and possibly set updateLCD flag
    
    //***************************************************
}

void updateClock(void){
    if (++masterCycles >= CYCLES_PER_SECOND){
        masterCycles = 0;
        checkUserTimers();
        if (++masterSecond >= SECONDS_PER_MINUTE){
            masterSecond = 0;
            updateClockOutput('m');
            if (++masterMinute >= MINUTES_PER_HOUR){
                masterMinute = 0;
                newMinuteOccurred = 1;
                updateClockOutput('h');
                if (++masterHour >= HOURS_PER_DAY){
                    masterHour = 0;
                }
            }
        }
    }
}

void checkUserTimers(void){
    if (timer4Set){
        if (++timer4Count > TIMER4_TARGET){
            //timer4Set     = 0;
            timer4Count   = 0;
            timer4Expired = 1;
        }
    }
    if (timer5Set){
        if (++timer5Count > TIMER5_TARGET){
            //timer5Set     = 0;
            timer5Count   = 0;
            timer5Expired = 1;
        }
    }
    if (timer40Set){
        if (++timer40Count > TIMER40_TARGET){
            //timer40Set     = 0;
            timer40Count   = 0;
            timer40Expired = 1;
        }
    }
}

void checkClockForTarget(void){
    if (newMinuteOccurred){
        newMinuteOccurred = 0;
        if (masterHour == targetHour){
           if (masterMinute == targetMinute){
               targetAcquired = 1;
            }
        }
    }
}

<<<<<<< HEAD
void updateClockOutput(void){
    lcdBuffers[0][BUFFER_START_00  ] = inputBuffer[0];
    lcdBuffers[0][BUFFER_START_00+1] = inputBuffer[1];
    lcdBuffers[0][BUFFER_START_00+3] = inputBuffer[2];
    lcdBuffers[0][BUFFER_START_00+4] = inputBuffer[3];
=======
void updateClockOutput(char key){
  switch(key){
    case 'm':
        lcdBuffers[0][BUFFER_START_00+3] =  int(masterMinute)%10;     //tens
        lcdBuffers[0][BUFFER_START_00+4] = (int(masterMinute)/10)%10; //ones
        break;
    case 'h':
        lcdBuffers[0][BUFFER_START_00+0] =  int(masterHour)%10;       //tens
        lcdBuffers[0][BUFFER_START_00+1] = (int(masterHour)/10)%10;   //ones
        break;
  }
  lcdNeedsUpdate = 1;
>>>>>>> d1fa55806609964218023efd111964e6aa1e265a
}

/**
  End of File
*/