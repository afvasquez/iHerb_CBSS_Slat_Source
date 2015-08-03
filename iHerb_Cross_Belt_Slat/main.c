/*
 * DumbSlat.c
 *
 * Created: 2/12/2015 1:27:09 PM
 *  Author: avasquez
 */ 

// Main System Include ( Should be the only one ) 
#include "system.h"

#define F_CPU 16000000UL
#define BAUDRATE 4800        //The baud rate that we want to use
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)    //The formula that does all the required maths
#define progAFTER_LOCK	( ( uint16_t ) 350 ) // Lock for about 350ms after programming

#define UNIT_FACTOR ( ( uint16_t ) 5 )
#define CONVERSION_FACTOR	158750UL

// Function Prototypes
void vSystemSetup( void );
void usartCleanPacket( void );
void fcnProcessPacket( void );
void usartNetworkStatistics( void );
void USART_SendByte(uint8_t u8Data);
void fcnRunSetup( void );
void fcnDelayRun( void );
void fcnStartRun( void );
void fcnEndRun( void );
void fcnEndProgramming( void );

/// Main System Variable Declaration
mySlatSystem mySystem;
myUSART myCOM;

//////////////////////////////////////////////////////////////////////////
//	###### Main Routine
//////////////////////////////////////////////////////////////////////////
int main(void)
{
	// ** Perform System Setup
	vSystemSetup();
	
    while(1)
    {
		switch ( mySystem.SystemStatus )	// Switch through the different machine states
		{
			case 'U':
			case 'E':
				// Call the Ramp Task
				motorRamp();	// Goes to motor Ramp
			break;
			case 'R':
				if ( mySystem.clkCount > mySystem.utilRampDown ) {
					mySystem.clkCount = 0;
					mySystem.SystemStatus = 'E';	// If there is Delay, do Delay
				}
			break;
			case 'S':	// StartUp Mode
				// Start System Right Away
				//PORTD &= 0b11111101;
				fcnStartRun();
			break;
			case 'D':	// Delay Mode
				// Check for Delay fulfilled
				fcnDelayRun();
			break;
			case 'C':	// This is a waiting state, look for packets and act accordingly
				fcnProcessPacket();
			break;
			case 'e':	// End of run scenario
				fcnEndRun();
			break;
			case 'd':	// Small delay after programming....
				fcnEndProgramming();
			break;
			
			case 'F':
				if ( mySystem.clkCount > 300 ) {
					PCICR &= ~(1 << PCIE2);    // set PCIE0 to enable PCMSK0 scan
					PORTC = mtrCCW_Rotate[ driveNEUTRAL ];
					
					mySystem.SystemStatus = ( uint8_t ) 'e';	// C - For COM
				}
			break;
			default:
			break;
		}
    }
}


//////////////////////////////////////////////////////////////////////////
//	###### System Setup
//////////////////////////////////////////////////////////////////////////
void vSystemSetup( void ) {
	
	////eeprom_write_byte(eepromCOM_RX_CORRUPT, 0x00);
	////eeprom_write_byte(eepromCOM_RX_DROPPED, 0x00);
	////eeprom_write_byte(eepromCOM_RX_SUCCESS, 0x00);
	////eeprom_write_byte(eepromCOM_RX_ERROR, 0x00);
	//eeprom_write_byte(eepromDEBUG_RX_FINAL, 0x00);
	//eeprom_write_byte(eepromCOM_RX_COUNT, 0x00);
	
	//////////////////////////////////////////////////////////////////////////
	//
	//////////////////////////////////////////////////////////////////////////
	// We continue by initializing LED output ports
	DDRD = 0b10001010;
	//PORTD &= 0b01111111;	// Zero on PORTD7
	
	//////////////////////////////////////////////////////////////////////////
	//	# System Timer Setup ( Timer 0 as a 1kHz Interrupt )
	//////////////////////////////////////////////////////////////////////////
	TCNT0=0x00;
	OCR0A = 249;
	TCCR0A = 0x03;
	TCCR0B |= 0x0B;
	TIMSK0 |= 0x0A;		// End of timer 0 Setup
	//////////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////////
	// SERIAL Communication Setup
	//////////////////////////////////////////////////////////////////////////
	DDRD |= 0b00000010;
	DDRD &= 0b11111110;
	UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
	UBRR0L = (uint8_t)(BAUD_PRESCALLER);
	
	UCSR0C = 0b00110110;
	UCSR0B = 0b10010000;	// Tx/Rx Enabled. Rx Interrupt Enabled
	/////////////////////////// END SERIAL COMMUNICATION SETUP ////////////////
	
	//////////////////////////////////////////////////////////////////////////
	//	# PWM Setup
	//////////////////////////////////////////////////////////////////////////
	// ** Using Timer 1
	DDRB |= _BV(PORTB1);	// Set this as out PWM output
	PORTB = 0;
	TCNT1 = 0;				// clear counter
	ICR1 = 255;				// 439.9956 Hz from 16 MHz clock
	TCCR1A |= 0b10000010;	// non-inverting, fast PWM
	TCCR1B |= 0b00000001;	// fast PWM, full speed
	OCR1A = 0x0000;   // 1 % strobe   01D0 max
	
	DDRC |= 0b00111111;		// Set the correct output for the motor and SET to neutral
	PORTC = mtrCCW_Rotate[ driveNEUTRAL ]; // SET to neutral
	
	//////////////////////////////////////////////////////////////////////////
	// System Variable Setup
	//////////////////////////////////////////////////////////////////////////
	mySystem.Dummy = eeprom_read_byte(eepromCONVEYOR_SPEED);
	if ( mySystem.Dummy == 0xFF ) {
		mySystem.Dummy = 0x30;	// Set to iHerb Automatically
		eeprom_write_byte(eepromCONVEYOR_SPEED, mySystem.Dummy);
	}
	mySystem.ConveyorSpeed = ( mySystem.Dummy * 5 ) + 60;	// Obtain on ft/min
	mySystem.SlatTravelTime = ( uint16_t ) ( ( ( float ) mySystem.ConveyorSpeed ) * ( ( float ) mathTO_MM_SEC ) );
	mySystem.SlatTravelTime = ( uint16_t ) ( ( ( uint32_t ) mathSLAT_WIDTH ) / ( ( uint32_t ) mySystem.SlatTravelTime ) );
	
	mySystem.Dummy = eeprom_read_byte(eepromSLAT_DUR_MULT);
	if ( mySystem.Dummy == 0xFF ) {
		mySystem.Dummy = 7;	// Set to iHerb Automatically
		eeprom_write_byte(eepromSLAT_DUR_MULT, mySystem.Dummy);
	}
	mySystem.SlatRunTimeMultiplier = mySystem.Dummy;
	
	//	Ramp Up Duration
	mySystem.Dummy = eeprom_read_byte( eepromRAMP_UP );		// Get the ramp up
	if ( mySystem.Dummy > 3 ) {
		mySystem.Dummy = 0;	// Set to iHerb Automatically
		eeprom_write_byte(eepromRAMP_UP, mySystem.Dummy);
	}
	mySystem.RampUp = ( mySystem.Dummy * 100 ) + 200;			// Ramp up in msec
	
	//	Ramp Down Duration
	mySystem.Dummy = eeprom_read_byte( eepromRAMP_DOWN );	// Get the ramp down
	if ( mySystem.Dummy > 3 ) {
		mySystem.Dummy = 0;	// Set to iHerb Automatically
		eeprom_write_byte(eepromRAMP_DOWN, mySystem.Dummy);
	}
	mySystem.RampDown = ( mySystem.Dummy * 100 ) + 200;			// Ramp down in msec
	
	
	//////////////////////////////////////////////////////////////////////////
	
	
	
	// Data Variable Initialization
	// Main Status
	// Clock Reset
	mySystem.clkCount = 0;
	// Communication Variable Intialization
	myCOM.ComErrorPackets = 0;
	myCOM.CorruptPackets = 0;
	myCOM.DroppedPackets = 0;
	myCOM.RxPackets = 0;		// Reset Network Statistics
	myCOM.PreviousComErrorPackets = 0;
	myCOM.PreviousCorruptPackets = 0;
	myCOM.PreviousDroppedPackets = 0;
	myCOM.PreviousRxPackets = 0;
	usartCleanPacket();
	
	
	//////////////////////////////////////////////////////////////////////////
	// Motor Status 
	//////////////////////////////////////////////////////////////////////////
	getPhaseCode();
	
	if ( mySystem.HallEffectSensor > 0 && mySystem.HallEffectSensor < 7)
	{
		// AS DEBUG: Reset the running counter and start on delay
		
		//////////////////////////////////////////////////////////////////////////
		//	# HALL Effect Sensor Setup
		//////////////////////////////////////////////////////////////////////////
		// **
		// Set the necessary PCINT settings for PB0-2
		//PORTD |= _BV(PORTD3);
		
		PCMSK2 |= 0b01110000;  // set PCINT0 to trigger an interrupt on state change
		mySystem.isMotorToRun = 0xFF;	// Motor is to run
		// IF all else is fine, go ahead and signal for Rx Input Waiting
		// Our first System Status is IDLE ( Waits for Rx Input )
		mySystem.SystemStatus = 'C';	// C - For COM
		
	} else {
		//PORTD |= _BV(PORTD7);	// Motor hookup error
		mySystem.isMotorToRun = 0x00;	// Motor is not to
		
		// IF all else is fine, go ahead and signal for Rx Input Waiting
		// Our first System Status is IDLE ( Waits for Rx Input )
		mySystem.SystemStatus = 'L';	// L - For Locking
	}
	
	// Enable interrupts
	sei();
}

//////////////////////////////////////////////////////////////////////////
// Main Timer ISR
//////////////////////////////////////////////////////////////////////////
// 1kHz Timer ISR
ISR( TIMER0_COMPA_vect ) {
	mySystem.clkCount++;
	
	// Data Timeout
	if ( myCOM.PacketRxStatus == 1 ) {	// Determine if we are taking data timeout	(Currently Receiving a Packet)
		if ( ( ( uint8_t ) ( mySystem.clkCount - myCOM.TimeoutTickCount ) ) > ( ( uint8_t ) systemBYTE_TIMEOUT ) ) {	// If data has been timed out
			
			myCOM.DroppedPackets++;		// Add to the Dropped Packet Count
			usartCleanPacket();	// Reset the data on the packet
		}
	}
}

//////////////////////////////////////////////////////////////////////////
//  Rx RECEIVE // WITHOUT ISR
//////////////////////////////////////////////////////////////////////////
//

//////////////////////////////////////////////////////////////////////////
// USART RX INTERRUPT SERVICE ROUTINE
//////////////////////////////////////////////////////////////////////////
// Rx ISR
ISR( USART_RX_vect ) {
	if ( myCOM.PacketRxStatus != 2 && myCOM.PacketRxStatus != 3 ) {	// Are we waiting for data?
		if ( ( UCSR0A & (1<<RXC0) ) ) {	// If a byte has been received and is waiting to be picked up
			// Reset main clock in order to avoid any overflow drama
			mySystem.clkCount = 0;		// Reset clock
			myCOM.TimeoutTickCount = mySystem.clkCount;	// Reset Timeout Counter
			
			myCOM.PacketRxStatus = 1;	// We are in the process of receiving and packet
			if ( !(UCSR0A & ( 1 << UPE0 )) ) {	// Is Parity fine?
				// Parity OK
				myCOM.PacketData[myCOM.PacketLength] = UDR0;
			} else {	// Error in Rx Detected
				myCOM.PacketData[myCOM.PacketLength] = 0xFE;	// Erroneous Data Byte
			}
			
			
			myCOM.PacketLength++;	// Increment Packet Length
			// Check if LoP counter is being used
			if ( myCOM.LoP < 0xFF ) myCOM.LoP--; // DECREASE: LoP Count
			if ( myCOM.PacketLength == 2 ) {	// Set LoP if this is the second byte
				myCOM.command = myCOM.PacketData[myCOM.PacketLength - 1] & 0b10000000;
				
				// Set the right LOP for the right type of data
				if ( myCOM.command ) {
					// Request one more byte if this is a RUN-TIME command
					myCOM.LoP = 1;
				} else {
					// Request as much data as possible
					myCOM.LoP = 6;
				}
			}
			
			if ( !myCOM.LoP ) {
				myCOM.PacketRxStatus = 2;	// Packet Rx Complete
				UCSR0B = 0b10000000;	// Tx/Rx Enabled. Rx Interrupt Enabled
			}
		} // Timeout omitted, might have to be placed on timer
	}
}

//////////////////////////////////////////////////////////////////////////
// USART Utility: Packet Information Resetter
//////////////////////////////////////////////////////////////////////////
void usartCleanPacket( void ) {
	myCOM.IsPacketCorrupt = 0;	// Packet is not corrupt
	myCOM.PacketLength = 0;		// Reset Packet data: No bytes on current packet
	myCOM.LoP = 0xFF;				// Reset Packet data: Back to DC on Last-Of-Packet
	myCOM.TimeoutTickCount = mySystem.clkCount;	// Reset the Timeout Counter
	myCOM.PacketRxStatus = 0;	// We are waiting for data/packet
	//PORTD &= 0b11111101;
	UCSR0B = 0b10010000;	// Tx/Rx Enabled. Rx Interrupt Enabled
	//PORTD |= 0b00001000;	// Zero on PORTD7
}


//////////////////////////////////////////////////////////////////////////
//	Serial Input Function:
//		This function will allow for timeout if a byte does is not
//		received in the allotted time
//////////////////////////////////////////////////////////////////////////
void fcnProcessPacket( void ) {
	int8_t i, error, flagCS, flagRURD, flagDuration;
	int8_t CS, RURD, Duration;
	uint8_t flag;
	
	if ( myCOM.PacketRxStatus == 2 ) {	// 2 - Received but not verified
		error = 0x00;
		
		// Perform packet verification
		for ( i=0; i<myCOM.PacketLength; i++ ) {
			eeprom_write_byte(eepromDEBUG_RX+i, myCOM.PacketData[i]);
			if ( myCOM.PacketData[i] == 0xFE ) {
				// If there is a single corrupted byte, ignore packet
				error++;
			}
		}
		
		if ( !( myCOM.PacketData[2] & 0b10000000 ) ) {	// If this is a Programming packet 
			if ( error < 3 ) {
				error = 0x00;
			}
		}
		
		// If there is an error, ignore and reset
		if ( !error ) {
			flag = myCOM.PacketData[2] & 0b10000000;
			if ( flag ) {	// If this is a RUN-TIME Packet
				// Check for all bytes to be the same
				if ( myCOM.PacketData[0] == myCOM.PacketData[1] && myCOM.PacketData[1] == myCOM.PacketData[2]  ) {
					//PORTD |= 0x80;	// One on PORTD7
					myCOM.PacketRxStatus = 3;	// Packet accepted and ready to operate on
					
					//	Set Parameters from received Data
					myCOM.RxPackets++;
					fcnRunSetup();
				} else {
					myCOM.ComErrorPackets++;	
					usartCleanPacket();
					mySystem.SystemStatus = 'C';	// Go back to paying attention
				}
			} else {	// If this is a programming packet
				flagCS = 0x00;
				flagDuration = 0x00;
				flagRURD = 0x00;
				CS = 0x00;
				RURD = 0x00;
				Duration = 0x00;
				// Get the necessary data and delay for 1 slat travel time
				for ( i=0;i<myCOM.PacketLength;i++ ) {
					// Store Packet
					
					error = myCOM.PacketData[i] & 0b11000000;
					error = error >> 6;
					if ( error == 0x00 && !flagCS ) {
						CS = myCOM.PacketData[i];
						flagCS = 0xFF;
					} else {
						error = myCOM.PacketData[i] & 0b00110000;
						error = error >> 4;
						if ( error == 0x01 && !flagRURD ) {
							RURD = myCOM.PacketData[i] & 0x0F;
							flagRURD = 0xFF;
						} else if ( error == 0x02 && !flagDuration ) {
							Duration = myCOM.PacketData[i] & 0x0F;
							flagDuration = 0xFF;
						}
					}
				}	// Programming Data Obtained!
				
				if ( flagCS && flagDuration && flagRURD ) {	// Program only if all data is available
					mySystem.Dummy = CS;
					if ( mySystem.Dummy > 58 ) {
						mySystem.Dummy = 0x3A;	// Set to iHerb Automatically
					}
					eeprom_write_byte(eepromCONVEYOR_SPEED, mySystem.Dummy);
					mySystem.ConveyorSpeed = ( mySystem.Dummy * 5 ) + 60;	// Obtain on ft/min
					mySystem.SlatTravelTime = ( uint16_t ) ( ( ( float ) mySystem.ConveyorSpeed ) * ( ( float ) mathTO_MM_SEC ) );
					mySystem.SlatTravelTime = ( uint16_t ) ( ( ( uint32_t ) mathSLAT_WIDTH ) / ( ( uint32_t ) mySystem.SlatTravelTime ) );
					
					eeprom_write_byte(eepromSLAT_DUR_MULT+2, Duration);
					mySystem.Dummy = Duration;
					if ( mySystem.Dummy < 4 ) {
						mySystem.Dummy = 4;	// Set to iHerb Automatically
					}
					eeprom_write_byte(eepromSLAT_DUR_MULT, mySystem.Dummy);
					mySystem.SlatRunTimeMultiplier = mySystem.Dummy;
					
					//	Ramp Up Duration
					mySystem.Dummy = RURD & 0b00001100;		// Get the ramp up
					mySystem.Dummy = mySystem.Dummy >> 2;
					eeprom_write_byte(eepromRAMP_UP, mySystem.Dummy);
					mySystem.RampUp = ( mySystem.Dummy * 100 ) + 200;			// Ramp up in msec
					
					//	Ramp Down Duration
					mySystem.Dummy = RURD & 0b00000011;		// Get the ramp up
					eeprom_write_byte(eepromRAMP_DOWN, mySystem.Dummy);
					mySystem.RampDown = ( mySystem.Dummy * 100 ) + 200;			// Ramp down in msec
					
					// End by entering into a polling loop and waiting some time before any input is taken again
					PORTD |= _BV(PORTD7);
					mySystem.clkCount = 0;	// Reset counter
					mySystem.SystemStatus = 'd';	// delay the system for about 150ms to keep sanity
				} else {	// If all the data is not available, ignore and get data all over again
					usartCleanPacket();
					mySystem.SystemStatus = 'C';	// Go back to paying attention	
				}
			} 
			
			
			
		} else {
			// Clean Packet and don't care
			myCOM.CorruptPackets++;
			// Debug
			//eeprom_write_byte(eepromDEBUG_RX, myCOM.PacketData[0]);
			//eeprom_write_byte(eepromDEBUG_RX+1, myCOM.PacketData[1]);
			//eeprom_write_byte(eepromDEBUG_RX+2, myCOM.PacketData[2]);
			//eeprom_write_byte(eepromDEBUG_RX+3, myCOM.PacketLength);
			// Debug
			usartCleanPacket();
			mySystem.SystemStatus = 'C';	// Go back to paying attention
		}
		// Run Network Statistics
		usartNetworkStatistics();
	}
}


//////////////////////////////////////////////////////////////////////////
//	Utility Network STATISTICS
//////////////////////////////////////////////////////////////////////////
void usartNetworkStatistics( void ) {
	char extraCount;
	
	if ( myCOM.CorruptPackets > myCOM.PreviousCorruptPackets ) {
		extraCount = eeprom_read_byte(eepromCOM_RX_CORRUPT);
		extraCount += myCOM.CorruptPackets - myCOM.PreviousCorruptPackets;
		eeprom_write_byte(eepromCOM_RX_CORRUPT, extraCount);
		myCOM.PreviousCorruptPackets = myCOM.CorruptPackets;
	}
	
	if ( myCOM.DroppedPackets > myCOM.PreviousDroppedPackets ) {
		extraCount = eeprom_read_byte(eepromCOM_RX_DROPPED);
		extraCount += myCOM.DroppedPackets - myCOM.PreviousDroppedPackets;
		eeprom_write_byte(eepromCOM_RX_DROPPED, extraCount);
		myCOM.PreviousDroppedPackets = myCOM.DroppedPackets;
	}
	
	if ( myCOM.RxPackets > myCOM.PreviousRxPackets ) {
		extraCount = eeprom_read_byte(eepromCOM_RX_SUCCESS);
		extraCount += myCOM.RxPackets - myCOM.PreviousRxPackets;
		eeprom_write_byte(eepromCOM_RX_SUCCESS, extraCount);
		myCOM.PreviousRxPackets = myCOM.RxPackets;
	}
	
	if ( myCOM.ComErrorPackets > myCOM.PreviousComErrorPackets ) {
		extraCount = eeprom_read_byte(eepromCOM_RX_ERROR);
		extraCount += myCOM.ComErrorPackets - myCOM.PreviousComErrorPackets;
		eeprom_write_byte(eepromCOM_RX_ERROR, extraCount);
		myCOM.PreviousComErrorPackets = myCOM.ComErrorPackets;
	}
}

//////////////////////////////////////////////////////////////////////////
// Byte Transmission Function
//////////////////////////////////////////////////////////////////////////
void USART_SendByte(uint8_t u8Data) {
	// Wait until last byte has been transmitted
	while( ( UCSR0A & ( 1<< UDRE0 ) ) == 0);
	// Transmit data
	UDR0 = u8Data;
}

//////////////////////////////////////////////////////////////////////////
//	Set Run-Time Parameters upon received data
//////////////////////////////////////////////////////////////////////////
void fcnRunSetup( void ) {
	//uint16_t maxDuration, minDuration;
	
	//if ( myCOM.PacketData[1] & ( unsigned char ) 0b01000000 ) {
		//// The bit for direction is HIGH, go CCW direction
		//mySystem.Direction = 0xFF;
	//} else {
		//// The bit for direction is LOW, go CW direction
		//mySystem.Direction = 0x00;
	//}
	
	mySystem.Direction = 0x00;
		
	// ## Calculate special slat delay
	//mySystem.Dummy = myCOM.PacketData[1] & 0b00111000;
	//mySystem.Dummy = mySystem.Dummy >> 3;
	//mySystem.Delay = mySystem.SlatTravelTime * ( uint16_t ) mySystem.Dummy;	// Add the special 90-Degree Delay
	
	mySystem.Delay = 0;
	mySystem.Duration = ( mySystem.SlatTravelTime * mySystem.SlatRunTimeMultiplier );
	
	//if (mySystem.Delay > 0) {
		//eeprom_write_word(eepromDEBUG_RX_FINAL, mySystem.Delay);
		//if ( mySystem.Duration <= mySystem.Delay + mySystem.RampUp + mySystem.RampDown ) {
			//mySystem.Delay = ( mySystem.SlatTravelTime * mySystem.SlatRunTimeMultiplier ) - 10 - mySystem.RampUp - mySystem.RampDown;
		//}
	//}
		
	// Get POWER and calculate ramp up and ramp down rates
	mySystem.Dummy = myCOM.PacketData[1] & 0b00000111;
	if ( mySystem.Dummy > 0x00 ) { // Run only if the power is not zero
		mySystem.Power = ( ( mySystem.Dummy + 3 ) * 51 ) + 1;
		
		//	Ramp Up Rate
		mySystem.RampUpRate = ( float ) ( ( mySystem.Power - consSLAT_MINIMUM_SPEED ) / ( float ) mySystem.RampUp );		// Ramp Up rate in DC/msec
		//	Ramp Down Rate
		mySystem.RampDownRate = ( float ) ( ( mySystem.Power - consSLAT_MINIMUM_SPEED ) / ( float ) mySystem.RampDown );	// Ramp Down rate in DC/msec
			
		/// MIGHT HAVE TO INCREASE THE NUMBER OF PICKUPS
		mySystem.Duration = ( mySystem.SlatTravelTime * mySystem.SlatRunTimeMultiplier ) - mySystem.Delay - mySystem.RampUp - mySystem.RampDown;
		
		//maxDuration = ( mySystem.SlatTravelTime * mySystem.SlatRunTimeMultiplier );
		//minDuration = ( mySystem.Delay + mySystem.RampUp + mySystem.RampDown );
		
		//eeprom_write_word(eepromBAD_DATA, maxDuration);
		//eeprom_write_word(eepromBAD_DATA+2, minDuration);
		
		if ( mySystem.Delay > 0 ) {
			// Start on Delay Mechanism
			mySystem.SystemStatus = 'D';	// Delay Mode
			mySystem.clkCount = 0;
		} else {
			// Start Right Away
			mySystem.SystemStatus = 'S';	// Start-up Mode
		}
		
		//if ( maxDuration > minDuration ) {
			//// Start the motor drive protocol depending on delay times
			//
		//} else { 
			//usartCleanPacket();
			//mySystem.SystemStatus = 'C';	// Go back to paying attention
		//}
		
		//////////////////////////////////////////////////////////////////////////
		// Store the values that were received
		PCICR |= (1 << PCIE2);    // set PCIE0 to enable PCMSK0 scan
	}
}

//////////////////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////////////////
void fcnDelayRun( void ) {
	if ( mySystem.clkCount > mySystem.Delay - 1 ) {
		// If the delay has been served, get to the next stage of execution
		mySystem.SystemStatus = 'S';	// Go to Start Up Mode
	}
}

void fcnStartRun( void ) {
	PORTD |= _BV(PORTD3);
	//PORTD |= 0b00000010;
	getPhaseCode();
	if ( mySystem.isMotorToRun )
	{
		//PORTC = mtrCW_Rotate[ ( uint8_t ) mySystem.HallEffectSensor ];
		if ( mySystem.Direction ) {
			PORTC = mtrCCW_Rotate[ ( uint8_t ) mySystem.HallEffectSensor ];
		} else {
			PORTC = mtrCW_Rotate[ ( uint8_t ) mySystem.HallEffectSensor ];
		}
		
		mySystem.clkCount = 0;	// Reset Timer Value
		mySystem.SystemStatus = 'U';	// Go to Ramp Up once the delay is done
	}
}

//////////////////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////////////////
void fcnEndRun( void ) {
	PORTD &= ~_BV(PORTD3);
	//PORTD &= 0b11111101;
	usartCleanPacket();
	mySystem.SystemStatus = 'C';
}

//////////////////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////////////////
void fcnEndProgramming( void ) {
	if ( mySystem.clkCount > systemAFTER_PG_DELAY ) {
		PORTD &= ~_BV(PORTD7);
		mySystem.clkCount = 0;	// Reset Clock
		usartCleanPacket();		// Reset Packet
		mySystem.SystemStatus = 'C';
	}
}