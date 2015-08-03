/*
 * drive.h
 *
 * Created: 2/13/2015 9:29:14 AM
 *  Author: avasquez
 */ 


#ifndef DRIVE_H_
#define DRIVE_H_

//////////////////////////////////////////////////////////////////////////
//	Drive Constant Parameters
//////////////////////////////////////////////////////////////////////////
#define HALL_0_Vect			PCINT2_vect	// Interrupt vector for first set of input pin interrupts
#define driveCCW_ROTATION	( 0x00 )	// CCW Drive Direction
#define driveCW_ROTATION	( 0xFF )	// CW Direction
#define drive0_SPEED		( ( uint16_t ) 0x0000 )	// 0% Speed
#define driveNEUTRAL		( 7 )		// Set the motor drive to neutral

// CW Motor Lookup Table
extern const char mtrCW_Rotate[8];
extern const char mtrCCW_Rotate[8];

void motorRamp ( void );
void getPhaseCode( void );


#endif /* DRIVE_H_ */