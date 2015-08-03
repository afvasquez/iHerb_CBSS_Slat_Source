/*
 * drive.c
 *
 * Created: 2/13/2015 9:29:02 AM
 *  Author: avasquez
 */ 

// Include the main library
#include "system.h"

// CW Motor Lookup Table
const char mtrCW_Rotate[8] = { 0x38, 0x21, 0x0A, 0x22, 0x14, 0x11, 0x0C, 0x38 };
const char mtrCCW_Rotate[8] = { 0x38, 0x0C, 0x11, 0x14, 0x22, 0x0A, 0x21, 0x38 };


//////////////////////////////////////////////////////////////////////////
//	###### Motor Ramping Task
//////////////////////////////////////////////////////////////////////////
void motorRamp ( void ) {
	mySystem.Overcurrent = PIND & 0b00000100;
	
	if ( mySystem.Overcurrent ) //not for testing
	{
		switch ( mySystem.SystemStatus )	// Switch on the different status
		{	// Only TWO cases, Ramp Up and Ramp Down
			case ( uint8_t ) 'U':		// RAMP UP CASE
			OCR1A = (( mySystem.clkCount - mySystem.Delay ) * mySystem.RampUpRate ) + 10;	// Could clamp on such small DC
			if ( OCR1A > ( int ) ( mySystem.Power - 10 ) )	// If the motor power is 255 or more, clamp and leave ramp
			{
				mySystem.utilRampDown = mySystem.Duration + mySystem.RampUp + mySystem.Delay;
				OCR1A = mySystem.Power;
				mySystem.SystemStatus = ( uint8_t ) 'R';
			}
			break;
			
			case ( uint8_t ) 'E':		// RAMP DOWN CASE
			OCR1A = mySystem.Power - ( ( mySystem.clkCount ) * mySystem.RampDownRate );
			if ( OCR1A < drive0_SPEED + 10 )	// If the motor power is 255 or more, clamp and leave ramp
			{
				OCR1A = 0;	// Reset System
				
				
				mySystem.clkCount = 0;
				//mySystem.isMotorToRun = 0x00;
				//cli();	// Clear all interrupts
				
				
				// Go back and wait for intput
				mySystem.isMotorToRun = 0xFF;	// Motor is to run
				// IF all else is fine, go ahead and signal for Rx Input Waiting
				// Our first System Status is IDLE ( Waits for Rx Input )
				mySystem.SystemStatus = ( uint8_t ) 'F';	// C - For COM
			}
			break;
		}
	} else {
		OCR1A = 0;
		PORTC = mtrCCW_Rotate[ driveNEUTRAL ];
		mySystem.isMotorToRun = 0x00;
		//cli();	// Clear all interrupts
		mySystem.SystemStatus = ( uint8_t ) 'L';
		PORTD |= _BV(PORTD7);	// Over-Current Flag
	}
}

//////////////////////////////////////////////////////////////////////////
// ## Getting phase code
//////////////////////////////////////////////////////////////////////////
void getPhaseCode( void ) {
	mySystem.HallEffectSensor = PIND;
	mySystem.Overcurrent = mySystem.HallEffectSensor & 0b00000100;
	mySystem.HallEffectSensor = mySystem.HallEffectSensor & 0b01110000;
	mySystem.HallEffectSensor =  mySystem.HallEffectSensor >> 4;
}

//////////////////////////////////////////////////////////////////////////
//	### Motor Drive ISR
//////////////////////////////////////////////////////////////////////////
ISR (HALL_0_Vect) {
	mySystem.HallEffectSensor = PIND & 0b01110000;
	mySystem.HallEffectSensor =  mySystem.HallEffectSensor >> 4;
	
	
	if ( mySystem.Direction ) {
		PORTC = mtrCCW_Rotate[ ( uint8_t ) mySystem.HallEffectSensor ];
	} else {
		PORTC = mtrCW_Rotate[ ( uint8_t ) mySystem.HallEffectSensor ];
	}
}