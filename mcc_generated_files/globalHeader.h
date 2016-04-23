/* 
 * File:   globalHeader.h
 * Author: sjevtic
 *
 * Created on April 15, 2016, 9:29 AM
 */

#ifndef GLOBALHEADER_H
#define	GLOBALHEADER_H

#ifdef	__cplusplus
extern "C" {
#endif

// I/O Macros
#define MOTOR_ON (LATBbits.LATB4 = 1)
#define MOTOR_OFF (LATBbits.LATB4 = 0)
#define HEATER_ON (LATBbits.LATB5 = 1)
#define HEATER_OFF (LATBbits.LATB5 = 0)
//#define SENSE_TEMP 10
#define GRIND_PUSHED (PORTBbits.RB0 == 1)
#define BREW_PUSHED (PORTBbits.RB1 == 1)
#define SWITCHES_CLOSED 1

// TIMER TARGETS
#define TIMER4_TARGET 4
#define TIMER5_TARGET 5
#define TIMER40_TARGET 40
    
#define CYCLES_PER_SECOND 24
#define TARGET_CLEAR_VALUE 99

#define SECONDS_PER_MINUTE 60
#define MINUTES_PER_HOUR 60
#define HOURS_PER_DAY 24
    
//LCD messages
#define LINE00 "     __:__      \0" //main clock time
#define LINE01 " X   __:__      \0" //main alarm time
#define LINE10 "  1: Set Alarm  \0"
#define LINE11 "  2: Set Clock  \0"
#define LINE20 " Alarm: __:__   \0" //set alarm time
#define LINE21 "1:Grind   2:Brew\0"
#define LINE30 "  Time: __:__   \0" //set clock time
#define LINE31 "                \0"

//LCD Buffer Variable Locations
#define BUFFER_START_00 5
#define BUFFER_START_01 5
#define BUFFER_START_20 8
#define BUFFER_START_30 8
    
#define ASCII_OFFSET 0x30

//timer global variables
unsigned char masterHour = 0;
unsigned char masterMinute = 0;
unsigned char masterSecond = 0;
unsigned char masterCycles = 0;

//alarm global variables
unsigned char targetHour   = TARGET_CLEAR_VALUE;
unsigned char targetMinute = TARGET_CLEAR_VALUE;

//global flags
unsigned char lcdNeedsUpdate = 0;
unsigned char newMinuteOccurred = 0;
unsigned char targetAcquired = 0;
unsigned char brewCommand  = 0; // only set this when ready to fire
unsigned char grindCommand = 0; // only set this when ready to fire

//user timer flags
unsigned char timer4Set  = 0;
unsigned char timer5Set  = 0;
unsigned char timer40Set = 0;
unsigned char timer4Expired  = 0;
unsigned char timer5Expired  = 0;
unsigned char timer40Expired = 0;

//user timer counts
unsigned char timer4Count  = 0;
unsigned char timer5Count  = 0;
unsigned char timer40Count = 0;

//state variables
unsigned char machineState = 0; // [0:3]
unsigned char settingState = 0; // 0 -> nothing, 1 -> grind, 2 -> brew
unsigned char menuState    = 0; // "GUI"

//Character buffers
char lcdBuffers[8][17] = {0};
char inputBuffer[4];

//character buffer position
unsigned char iBuffer = 0;

//special button pushed for BREW and GRIND
//unsigned char grindPushed = 0;
//unsigned char brewPushed  = 0;

#ifdef	__cplusplus
}
#endif

#endif	/* GLOBALHEADER_H */

