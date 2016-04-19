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

void runMenu(void);
void updateLCD(void);
char readKeypad(void);
void executeMenu(void);
void runStateMachine(void);
/*
                         Main application
 */

void main(void)
{
    unsigned char ch;
    // Initialize the device
    SYSTEM_Initialize();

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
    
    while (1)
    {
        runMenu();
        ch = readKeypad();
        runStateMachine();
        
        if (lcdNeedsUpdate){
            updateLCD();
        }
    }
    return;
}

void runStateMachine(void){
    switch(machineState){
            case 0: //idle
                MOTOR_OFF;
                HEATER_OFF;
                // if (SWITCHES_CLOSED)
                    // if targetAcquired
                        // set grind cmd or brew cmd based on flag
                    // if GRIND_PUSHED
                        // grindCommand = true
                    // if brewButtonPushed
                        // brewCommand = true
                    //if grindCmd
                        // machineState = 1
                    //if brewCmd
                        // machineState = 2
                break;
                
            case 1: //grinding
                // if (SWITCHES_CLOSED)
                    MOTOR_ON;
                    HEATER_OFF;
                    // if timer40 not set
                        // set timer40
                    // if timer40 expired
                        // machineState = 2
                        //MOTOR_OFF;
                    // if brewButtonPushed
                        // brewCommand = true
                        //MOTOR_OFF
                        //machineState = 2
                // else
                    //MOTOR_OFF;
                    //machineState = 0;
                break;
                
            case 2: //brewing
                // if (SWITCHES_CLOSED)
                    HEATER_ON;
                    MOTOR_OFF;
                    // if SENSE_TEMP is 4V
                        //HEATER_OFF
                        //machineState = 3
                    // if grindButtonPushed
                        // grindCommand = true
                        //HEATER_OFF
                        //machineState = 1
                 // else
                    // HEATER_OFF;
                    //machineState = 0;
                break;
                
            case 3: //cleaning
                //if switches closed
                    //if timer4 not set
                        //set timer4
                    // if timer4 expired
                        //timer4expired = false
                        //set timer 5
                        MOTOR_ON;
                    // if timer5 expired
                        // timer5 expired = false
                        MOTOR_OFF;
                        machineState = 0;
                // else
                    // MOTOR_OFF
                    // HEATER_OFF
                    // machineState = 0
                break;
        }
}

char readKeypad(void){
    // to be developed
    unsigned char keypadInput = 'Z';
    unsigned char keypad1 = PORTCbits.RC5;
    unsigned char keypad2 = PORTCbits.RC2;
    unsigned char keypad3 = PORTAbits.RA7;
    unsigned char keypad4 = PORTCbits.RC4;
    unsigned char keypad5 = PORTCbits.RC3;
    unsigned char keypad6 = PORTAbits.RA6;
    unsigned char keypad7 = PORTDbits.RD3;
    unsigned char keypad8 = PORTDbits.RD0;
    unsigned char keypad9 = PORTCbits.RC0;
    unsigned char keypad0 = PORTDbits.RD1;
    unsigned char keypadS = PORTDbits.RD2;
    unsigned char keypadP = PORTCbits.RC1;
    
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

void runMenu(void){
    // to be developed
}

void updateLCD(void){
    // to be developed
}
/**
 End of File
*/