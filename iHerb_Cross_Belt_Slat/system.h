/*
 * SlatSystem.h
 *
 * Created: 2/12/2015 1:36:38 PM
 *  Author: avasquez
 */ 


#ifndef SYSTEM_H_
#define SYSTEM_H_

// ###### Include the drive library
#include "drive.h"

//////////////////////////////////////////////////////////////////////////
// * System Constants Definition
//////////////////////////////////////////////////////////////////////////
// %%%%%% EEPROM
#define eepromRUN_TIME_RX		( ( void * ) 0x00 )	// Number of RT Rx
#define eepromPROGRAMMING_RX	( ( void * ) 0x02 )	// Number of PG Rx
#define eepromPOWER				( ( void * ) 0x40 )	// Slat Power
#define eepromRAMP_UP			( ( void * ) 0x21 ) // Slat Ramp Up
#define eepromRAMP_DOWN			( ( void * ) 0x22 )	// Slat Ramp Down
#define eepromDIRECTION			( ( void * ) 0x23 )	// Motor Rotation Direction
#define eepromDURATION			( ( void * ) 0x24 )	// Slat Duration ( 2 Bytes )
#define eepromDELAY				( ( void * ) 0x26 )	// Slat Delay ( 2 Bytes )
#define eepromCONVEYOR_SPEED	( ( void * ) 0x30 ) // Conveyor Speed
#define eepromSLAT_DUR_MULT		( ( void * ) 0x32 ) // Slat Duration Multiplier
#define eepromTIMES_PROGRAMMED	( ( void * ) 0x50 ) // Times Programmed
#define eepromBAD_DATA			( ( void * ) 0x60 ) // Bad Data Registers
#define eepromBAD_DATA_PROC		( ( void * ) 0x70 ) // Bad Data Registers

// %%%%%% Mathematical Constants
#define convSLAT_TIMER_MULT		( ( float ) 5.08 )
#define consSLAT_WIDTH			( ( int ) 159 ) // In mm
#define consCONVEYOR_SPEED_SCALE	( 5 )	// "m" from a simple linear approximation to conveyor speed
#define consCONVEYOR_SPEED_OFFSET	( 60 )	// "b" from a simple linear approximation to conveyor speed
// %%%%%% System Constants
#define consSLAT_MINIMUM_SPEED		( 0 )
#define systemMAXIMUM_RANGE		( ( uint32_t ) 65534 )	// Maximum duration for slat operation
#define systemBYTE_TIMEOUT	( 6 )	// Waiting-on-byte timeout
#define systemAFTER_PG_DELAY	( ( uint16_t ) 3000 )	// Delay for about 150ms after programming

#define eepromCOM_RX_SUCCESS		( ( void * ) 0x90 )	// Device ID Memory Location
#define eepromCOM_RX_DROPPED		( ( void * ) 0x91 )	// Device ID Memory Location
#define eepromCOM_RX_CORRUPT		( ( void * ) 0x92 )	// Device ID Memory Location
#define eepromCOM_RX_ERROR			( ( void * ) 0x93 )	// Device ID Memory Location

#define eepromDEBUG_RX			( ( void * ) 0x80 )	// Device ID Memory Location
#define eepromDEBUG_RX_FINAL	( ( void * ) 0xB0 )	// Device ID Memory Location


// %%%%%% Maths Constants
#define mathTO_MM_SEC				( ( float ) 5.08 )	// Conversion Constant
#define mathSLAT_WIDTH				( ( uint32_t ) 158750 )// System Constant

//////////////////////////////////////////////////////////////////////////
// * Typical AVR libraries
#include <avr/io.h>			// IO
// * EEPROM
#include <avr/eeprom.h>		// EEPROM
// * Interrupts
#include <avr/interrupt.h>	// INTERRUPTS

//////////////////////////////////////////////////////////////////////////
// * Global System Variable
typedef struct mySlatSystems {
	char SystemStatus;	// Status
	uint8_t isMotorToRun;	// Is motor to run 
	uint16_t ConveyorSpeed;	// Conveyor Speed
	
	// Driving Variables
	uint16_t RampUp;
	uint16_t RampDown;
	uint16_t SlatTravelTime;
	uint8_t SlatRunTimeMultiplier;
	float RampUpRate;
	float RampDownRate;
	uint16_t Power;
	uint16_t Duration;
	uint16_t Delay;
	uint8_t Direction;
	
	// Rx Variables
	unsigned char RxBuffer[5];	// Rx Memory Space
	
	// Sensor
	uint8_t HallEffectSensor;
	uint8_t Overcurrent;
	
	// Utilities
	uint32_t clkCount;	// Status
	
	// Utilities
	uint16_t utilRampDown;
	uint8_t CSLock;
	uint8_t SILock;
	uint8_t SLock;
	uint8_t CSHold;
	uint8_t SIHold;
	uint8_t SHold;
	uint8_t Dummy;
	uint16_t longDummy;
	float floatDummy;
} mySlatSystem;

// * USART Rx Data Structure
typedef struct myUSARTS {
	uint8_t PacketRxStatus;
	uint8_t PacketLength;
	uint8_t LoP;
	
	uint16_t TimeoutTickCount;
	
	char PacketData[8];
	char command;	// Current command to be sent
	
	// Utilities
	uint8_t PreviousDroppedPackets, DroppedPackets;
	uint8_t PreviousRxPackets, RxPackets;
	uint8_t PreviousCorruptPackets, CorruptPackets;
	uint8_t PreviousComErrorPackets, ComErrorPackets;
	uint8_t IsPacketCorrupt;
} myUSART;

//////////////////////////////////////////////////////////////////////////
//	** Accessibility Declaration
extern mySlatSystem mySystem;

#endif /* SLATSYSTEM_H_ */
