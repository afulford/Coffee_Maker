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
#define SWITCHES_CLOSED 10

// TIMER TARGETS
#define TIMER4_TARGET 4
#define TIMER5_TARGET 5
#define TIMER40_TARGET 40
    
#define CYCLES_PER_SECOND 24
#define TARGET_CLEAR_VALUE 99

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

//hex_keypad variables
char keypadInput = 'Z';
//#define keypad1 PORTCbits.RC6
//#define keypad2 PORTBbits.RB5
//#define keypad3 PORTCbits.RC0
//#define keypad4 PORTCbits.RC3
//#define keypad5 PORTBbits.RB6
//#define keypad6 PORTCbits.RC1
//#define keypad7 PORTCbits.RC4
//#define keypad8 PORTBbits.RB7
//#define keypad9 PORTCbits.RC2
//#define keypad0 PORTCbits.RC7
//#define keypadS PORTCbits.RC5
//#define keypadP PORTBbits.RB4

//hex_keypad flags
unsigned inputRecieved = 0;

#ifdef	__cplusplus
}
#endif

#endif	/* GLOBALHEADER_H */

