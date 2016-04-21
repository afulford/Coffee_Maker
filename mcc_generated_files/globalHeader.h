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
#define MOTOR_ON 10
#define MOTOR_OFF 10
#define HEATER_ON 10
#define HEATER_OFF 10
#define SENSE_TEMP 10
#define GRIND_PUSHED 10
#define BREW_PUSHED 10
#define SWITCHES_CLOSED 1

// TIMER TARGETS
#define TIMER4_TARGET 4
#define TIMER5_TARGET 5
#define TIMER40_TARGET 40
    
#define CYCLES_PER_SECOND 24
#define TARGET_CLEAR_VALUE 99
    
//LCD messages
#define LINE00 "     __:__      \0"
#define LINE01 " X   __:__      \0"
#define LINE10 "  1: Set Alarm  \0"
#define LINE11 "  2: Set Clock  \0"
#define LINE20 "  Alarm: __:__  \0"
#define LINE21 "1:Grind   2:Brew\0"
#define LINE30 "  Time: __:__   \0"
#define LINE31 "                \0"

//LCD Buffer Variable Locations
#define BUFFER_START_00 5
#define BUFFER_START_01 5
#define BUFFER_START_20 9
#define BUFFER_START_30 9

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
char lcdBuffers[8][16];
char inputBuffer[4];

//character buffer position
unsigned char iBuffer = 0;


#ifdef	__cplusplus
}
#endif

#endif	/* GLOBALHEADER_H */

