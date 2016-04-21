
                   Readme File for Code Example: EXTERNAL LCD SUPPORT
                 		     
             -----------------------------------------------------------------------------------------

This file contains the following sections:
1. Code Example Description
2. Folder Contents
3. Suggested Development Resources
4. Reconfiguring the project for a different PIC18F device
5. Revision History

NOTE: The example code makes use of Peripheral Library support available with MCC18.

1. Code Example Description:
----------------------------
In this code example,  XLCD library support for PIC18F4685 is used to interface the device with LCD module 
available on PICDEM2+ board. The software implementation makes use of the following connections described below.

HARDWARE CONNECTION DETAILS:

**DATA_PORT defines the port to which the LCD data lines are connected */
				DATA_PORT     	<-------->  	PORTD
				TRIS_DATA_PORT	<-------->		TRISD

* CTRL_PORT defines the port where the control lines are connected.
* These are just samples, change to match your application.

				RW_PIN 			<-------->		PORTA<2>   /* PORT for RW */ 
				RS_PIN   		<-------->		PORTA<3>   /* PORT for RS */ 
				E_PIN    		<-------->		PORTA<1>   /* PORT for D  */ 



2. Folder Contents:
-------------------
This folder refers the following sub-folders:
a. C:\MCC18\bin\LKR
        This folder will have the device LKR file, it is used for building the project. 
	This file was provided with the MCC18 toolsuite.

b. C:\MCC18\h
        This folder contains C header files useful in building this
        project. Device register and bit definitions are provided in
        the *.h file that follows the device name. These files were provided
        with the MCC18 toolsuite.The peripheral library header files contains the macro definition & API
		prototypes.

c. C:\MCC18\lib
        This folder contains library archive files, which are a
        collection of precompiled object files. The file
        named "*.lib" in the name of device contains the C run-time, start-up & Peripheral
        library. These file were provided with the MCC18 toolsuite.

d. C:\MCC18\src
        This folder contains all the C ,Assembler, Peripheral library source files (*.c,
        *.s) used in demonstrating the described example. 
		
e. C:\MCC18\doc
		This folder contains the Peripheral Library description for all modules.		


3. Suggested Development Resources:
-----------------------------------
		a. MPLAB IDE with MCC18 v3.30 or above
        b. PIC DEM 2+ Development Board with  PIC18F4685 controller

4. Reconfiguring the project for a different PIC18F device:
-------------------------------------------------------------
The Project/Workspace can be easily reconfigured for any PIC18F device.
Please use the following general guidelines:
        a. Change device selection within MPLAB?IDE to a PIC18F device of
        your choice by using the following menu option:
        MPLAB IDE>>Configure>>Select Device

		b. To configure the device settings, either choose from the MPLAB IDE>>Configure>>"Configuration bits" 
		and configure for necessary settings or the corresponding configuration
		macros to be added into the code can be obtained from device.INC file 
		at the following default location:
		"C:\MCC18\mpasm"
		These macros can be added as depicted in example code.
        c. Re-build the MPLAB?project using the menu option:
        MPLAB IDE>>Project>>Build All

        d. Download the hex file into the device and run.

5. Revision History :
---------------------
        04/04/2009 - Initial Release of the Code Example
