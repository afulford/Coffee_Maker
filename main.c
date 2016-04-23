/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB(c) Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
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

#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/globalHeader.h"
#include "External_LCD/xlcd.h"
void runMenu(char);
void executeMenu(void);
void initLcdBuffers(void);
void LCD_Initialize(void);
void runStateMachine(void);
void updateLcd(unsigned char);
unsigned char readKeypad(void);
unsigned char readSingleKey(void);
void updateLcdBuffer(unsigned char index);
void processClockBuffer (unsigned char index);
/*
                         Main application
 */

void main(void)
{
    unsigned char ch;
    // Initialize the device
    SYSTEM_Initialize();
    LCD_Initialize();
    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global and Peripheral Interrupts
    // Use the following macros to:

    // Enable high priority global interrupts
    //INTERRUPT_GlobalInterruptHighEnable();

    // Enable low priority global interrupts.
    //INTERRUPT_GlobalInterruptLowEnable();

    // Disable high priority global interrupts
    //INTERRUPT_GlobalInterruptHighDisable();

    // Disable low priority global interrupts.
    //INTERRUPT_GlobalInterruptLowDisable();

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    
//	SetDDRamAddr(0x80);
//	while( BusyXLCD() );
//	putsXLCD("  Good Morning,");
//    while( BusyXLCD() );
//    SetDDRamAddr(0xC0);
//	while( BusyXLCD() );
//	putsXLCD("    Stefony");				
//	while( BusyXLCD() );	
	
    while (1)
    {
        if ((ch = readSingleKey()) != 'Z'){
            runMenu(ch);
        }
        runStateMachine();
        
        if (lcdNeedsUpdate){
            lcdNeedsUpdate = 0;
            updateLcd(menuState);
        }
    }
    return;
}

void runStateMachine(void){
    switch(machineState){
            case 0: //idle
                MOTOR_OFF;
                HEATER_OFF;
                
                if (SWITCHES_CLOSED){
                    if (targetAcquired){
                        targetAcquired = 0;
                        // set grind cmd or brew cmd based on flag
                        if (settingState == 0){
                            brewCommand = grindCommand = 0;
                        }
                        else if (settingState == '1'){
                            settingState = 0;
                            grindCommand = 1;
                        }
                        else if (settingState == '2'){
                            settingState = 0;
                            brewCommand = 1;
                        }
                    }
                        
                    if (GRIND_PUSHED){
                    //if (grindPushed){
                        grindCommand = 1;
                    }
                        
                    if (BREW_PUSHED){
                    //if (brewPushed){
                        brewCommand = 1;
                    }
                        
                    if (grindCommand){
                        machineState = 1;
                        grindCommand = 0;
                    }
                    if (brewCommand){
                        machineState = 2;
                        brewCommand = 0;
                    }
                }
                break;
                
            case 1: //grinding
                if (SWITCHES_CLOSED){
                    MOTOR_ON;
                    HEATER_OFF;
                    if (!timer40Set){
                        timer40Set = 1;
                    }
                    if (timer40Expired){
                        timer40Expired = 0;
                        timer40Set = 0;
                        machineState = 2;
                        MOTOR_OFF;
                    }
                    if (BREW_PUSHED){
                    //if (brewPushed){
                        //brewCommand = 1;
                        timer40Set = 0;
                        MOTOR_OFF;
                        machineState = 2;
                    }
                } else {
                    MOTOR_OFF;
                    machineState = 0;
                    timer40Set = 0;
                    timer40Expired = 0;
                    timer40Count = 0;
                }
                break;
                
            case 2: //brewing
                if (SWITCHES_CLOSED){
                    HEATER_ON;
                    MOTOR_OFF;
                    if (SENSE_TEMP){
                        HEATER_OFF;
                        machineState = 3;
                    }
                    if (GRIND_PUSHED){
                    //if (grindPushed){
                        //grindCommand = 1;
                        HEATER_OFF;
                        machineState = 1;
                    }
                } else {
                    HEATER_OFF;
                    machineState = 0;
                }
                break;
                
            case 3: //cleaning
                if (SWITCHES_CLOSED){
                    if (!timer4Set){
                        timer4Set = 1;
                    }
                    if (timer4Expired){
                        //timer4Set = 0;
                        timer4Expired = 0;
                        timer5Set = 1;
                        MOTOR_ON;
                    }
                    if (timer5Expired){
                        timer4Set = 0;
                        timer5Set = 0;
                        timer5Expired = 0;
                        MOTOR_OFF;
                        machineState = 0;
                    }
                } else {
                    MOTOR_OFF;
                    HEATER_OFF;
                    timer4Set = 0;
                    timer5Set = 0;
                    timer4Count = 0;
                    timer5Count = 0;
                    timer5Expired = 0;
                    timer4Expired = 0;
                    machineState = 0;
                }
                break;
        }
}

unsigned char readSingleKey(void){
    char goodChar;
    goodChar = readKeypad();
    
    while(readKeypad() != 'Z');
    return goodChar;
}

unsigned char readKeypad(void){
    unsigned char keypadInput = 'Z';
    unsigned char keypad1 = PORTAbits.RA5;
    unsigned char keypad2 = PORTAbits.RA7;
    unsigned char keypad3 = PORTCbits.RC2;
    unsigned char keypad4 = PORTAbits.RA3;
    unsigned char keypad5 = PORTEbits.RE2;
    unsigned char keypad6 = PORTCbits.RC1;
    unsigned char keypad7 = PORTAbits.RA2;
    unsigned char keypad8 = PORTEbits.RE1;
    unsigned char keypad9 = PORTCbits.RC0;
    unsigned char keypad0 = PORTEbits.RE0;
    unsigned char keypadS = PORTAbits.RA1;
    unsigned char keypadP = PORTAbits.RA6;
    
    if(keypad1 == 1){
        keypadInput = '1';
    }
    else if(keypad2 == 1){
        keypadInput = '2';
    }
    else if(keypad3 == 1){
        keypadInput = '3';
    }
    else if(keypad4 == 1){
        keypadInput = '4';
    }
    else if(keypad5 == 1){
        keypadInput = '5';
    }
    else if(keypad6 == 1){
        keypadInput = '6';
    }
    else if(keypad7 == 1){
        keypadInput = '7';
    }
    else if(keypad8 == 1){
        keypadInput = '8';
    }
    else if(keypad9 == 1){
        keypadInput = '9';
    }
    else if(keypad0 == 1){
        keypadInput = '0';
    }
    else if(keypadS == 1){
        keypadInput = '*';
    }
    else if(keypadP == 1){
        keypadInput = '#';
    }
    return keypadInput;
}

void runMenu(unsigned char key){
    switch(menuState){
        case 0:
            // update LCD only
            menuState = 1;
            break;
        case 1:
            if (key == '1'){
                menuState = 2;
            }
            else if (key == '2'){
                menuState = 3;
            }
            else if (key == '*'){
                menuState = 0;
            }
            break;
        case 2://alarm
            if (key == '#'){
                processClockBuffer(menuState);
                menuState = 4;
            }
            else if (key == '*'){
                menuState = 1;
            }
            else if ((key >= '0') && (key <= '9')){
                inputBuffer[iBuffer++] = (char)key;
                updateLcdBuffer(menuState);
                if (iBuffer > 3) {
                    iBuffer = 0;
                }
            }
            break;
        case 3://clock
            if (key == '#'){
                processClockBuffer(menuState);
                menuState = 0;
            }
            else if (key == '*'){
                menuState = 1;
            }
            else if ((key >= '0') && (key <= '9')){
                inputBuffer[iBuffer++] = (char)key;
                updateLcdBuffer(menuState);
                if (iBuffer > 3) {
                    iBuffer = 0;
                }
            }
            break;
        case 4:
            if (key == '1'){
                settingState = key;
                lcdBuffers[1][1] = 'G';
                menuState = 0;
            }
            if (key == '2'){
                settingState = key;
                lcdBuffers[1][1] = 'B';
                menuState = 0;
            }
    }
    updateLcd(menuState);
}

void updateLcdBuffer (unsigned char index){
    switch (index){
        case 0: //main clock time
            lcdBuffers[0][BUFFER_START_00  ] = inputBuffer[0];
            lcdBuffers[0][BUFFER_START_00+1] = inputBuffer[1];
            lcdBuffers[0][BUFFER_START_00+3] = inputBuffer[2];
            lcdBuffers[0][BUFFER_START_00+4] = inputBuffer[3];
            break;
        case 1: //main alarm time
            lcdBuffers[1][BUFFER_START_01  ] = inputBuffer[0];
            lcdBuffers[1][BUFFER_START_01+1] = inputBuffer[1];
            lcdBuffers[1][BUFFER_START_01+3] = inputBuffer[2];
            lcdBuffers[1][BUFFER_START_01+4] = inputBuffer[3];
            break;
        case 2: //set alarm time
            switch(iBuffer){
                case 1:
                    lcdBuffers[4][BUFFER_START_20  ] = inputBuffer[0];
                    break;
                case 2:
                    lcdBuffers[4][BUFFER_START_20+1] = inputBuffer[1];
                    break;
                case 3:
                    lcdBuffers[4][BUFFER_START_20+3] = inputBuffer[2];
                    break;
                case 4:
                    lcdBuffers[4][BUFFER_START_20+4] = inputBuffer[3];
                    break;
            }
            break;
        case 3: //set clock time
            switch(iBuffer){
                case 1:
                    lcdBuffers[6][BUFFER_START_30  ] = inputBuffer[0];
                    break;
                case 2:
                    lcdBuffers[6][BUFFER_START_30+1] = inputBuffer[1];
                    break;
                case 3:
                    lcdBuffers[6][BUFFER_START_30+3] = inputBuffer[2];
                    break;
                case 4:
                    lcdBuffers[6][BUFFER_START_30+4] = inputBuffer[3];
                    break;
            }
            break;
    }
    //iBuffer = 0;
}

void processClockBuffer (unsigned char index){
    unsigned char iLcd = 0;
    switch (index) {
        case 2:
            targetMinute =  
                ((unsigned char) inputBuffer[3]) - ((unsigned char) ASCII_OFFSET) +
                (((unsigned char) inputBuffer[2] - ASCII_OFFSET) * 10);
            targetHour   =  
                ((unsigned char) inputBuffer[1]) - ((unsigned char) ASCII_OFFSET) +
                (((unsigned char) inputBuffer[0] - ASCII_OFFSET) * 10);
            iLcd = 1;
            break;
        case 3:
            masterMinute =  
                ((unsigned char) inputBuffer[3]) - ((unsigned char) ASCII_OFFSET) +
                (((unsigned char) inputBuffer[2] - ASCII_OFFSET) * 10);
            masterHour   =  
                ((unsigned char) inputBuffer[1]) - ((unsigned char) ASCII_OFFSET) +
                (((unsigned char) inputBuffer[0] - ASCII_OFFSET) * 10);
            iLcd = 0;
            break;
    }
    iBuffer = 0;
    updateLcdBuffer(iLcd);
}

void updateLcd(unsigned char index){
    char* lineOneTarget;
    char* lineTwoTarget;
    
    switch(index){
        case 0:
            lineOneTarget = lcdBuffers[0];
            lineTwoTarget = lcdBuffers[1];
            break;
        case 1:
            lineOneTarget = lcdBuffers[2];
            lineTwoTarget = lcdBuffers[3];
            break;
        case 2:
            lineOneTarget = lcdBuffers[4];
            lineTwoTarget = lcdBuffers[5];
            break;
        case 3:
            lineOneTarget = lcdBuffers[6];
            lineTwoTarget = lcdBuffers[7];
            break;
    }
    //*********** Set the starting address in the LCD RAM for display. This determines the location of display ********	
	SetDDRamAddr(0x80);
		while( BusyXLCD() );		//wait until LCD controller is busy
	putsXLCD(lineOneTarget);			//Display string of text
		while( BusyXLCD() );		//wait until LCD controller is busy
        
     //********** Set the address in second line for display ****************************************************		
	SetDDRamAddr(0xC0);
		while( BusyXLCD() );		//wait until LCD controller is busy
	putsXLCD(lineTwoTarget);					//Display string of text
		while( BusyXLCD() );		//wait until LCD controller is busy
}

void initLcdBuffers(void){
    char ch;
    int i = 0;
    char* ptr1 = LINE00;
    char* ptr2 = LINE01;
    char* ptr3 = LINE10;
    char* ptr4 = LINE11;
    char* ptr5 = LINE20;
    char* ptr6 = LINE21;
    char* ptr7 = LINE30;
    char* ptr8 = LINE31;

    i=0;
    while ((ch=(*(ptr1++))) != '\0'){
        lcdBuffers[0][i++] = ch;
    }
    i=0;
    while ((ch=(*(ptr2++))) != '\0'){
        lcdBuffers[1][i++] = ch;
    }
    i=0;
    while ((ch=(*(ptr3++))) != '\0'){
        lcdBuffers[2][i++] = ch;
    }
    i=0;
    while ((ch=(*(ptr4++))) != '\0'){
        lcdBuffers[3][i++] = ch;
    }
    i=0;
    while ((ch=(*(ptr5++))) != '\0'){
        lcdBuffers[4][i++] = ch;
    }
    i=0;
    while ((ch=(*(ptr6++))) != '\0'){
        lcdBuffers[5][i++] = ch;
    }
    i=0;
    while ((ch=(*(ptr7++))) != '\0'){
        lcdBuffers[6][i++] = ch;
    }
    i=0;
    while ((ch=(*(ptr8++))) != '\0'){
        lcdBuffers[7][i++] = ch;
    }
}

void LCD_Initialize(void){
    unsigned char config = 0xFF;
    ADCON1 = 0xFF;
    config = EIGHT_BIT & LINES_5X7;
    OpenXLCD(config);
    while( BusyXLCD() );
    initLcdBuffers();
    updateLcd(0);
}

/**
 End of File
*/