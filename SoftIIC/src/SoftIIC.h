/*
 * SoftIIC, a library for IIC communications on any pins.
 * Copyright (C) 2015 cirthix@gmail.com
 * 
 * 
 * This file is part of SoftIIC.
 * 
 * This software is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.  In addition to or superseding this license, this
 * software may not be used in any form which is not completely free without the author's written 
 * consent.
 * 
 * This software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * 
 * See the GNU General Public License for more details. You should have received a copy of the GNU
 * General Public License along with This software. If not, see <http://www.gnu.org/licenses/>.
 * 
 * Authors: cirthix@gmail.com
 */

// This library takes some tricks from https://github.com/todbot/SoftI2CMaster



#ifndef SOFTIIC_h
#define SOFTIIC_h
 
 #include <Arduino.h>
#include <inttypes.h>
#include <avr/wdt.h>
 
// You can define this to gain a tiny bit of speed+space (may matter at 100KHz)
//#define SOFTIIC_OPTIMIZE_NOPULLUPS
 
 // This is necessary for 100KHz host operation at less than 16MHz core clock
//#define SOFTIIC_OPTIMIZE_FAST
 
#ifdef SOFTIIC_OPTIMIZE_FAST
	#pragma GCC optimize("O3") 
#endif
 
 
#define FORCE_INLINE __attribute__((always_inline)) inline
 
 
// Define this if using the extra debug pins for timing debug/analysis
// Note that during debugging, timer2 is used, digitalwrite2 library is pulled in, and the pins SOFTIIC_DEBUGPIN_A and SOFTIIC_DEBUGPIN_B are used.
//#define SOFTIIC_DEGBUG_MODE 
  

// You can externally override the debug pins when using debug mode.
#ifdef SOFTIIC_DEGBUG_MODE 
	#ifndef SOFTIIC_DEBUGPIN_A
		#define SOFTIIC_DEBUGPIN_A 14		// This debug-output pin is used to verify timing relationships between the code and physical sda/scl pins.
	#endif 
	#ifndef SOFTIIC_DEBUGPIN_B
		#define SOFTIIC_DEBUGPIN_B 15		// This debug-output pin is used to verify timing relationships between the code and physical sda/scl pins.
	#endif 
#endif
 
#ifdef SOFTIIC_DEBUGPIN_A
	#include "arduino2.h"  // include the fast I/O 2 functions
#endif

#ifdef SOFTIIC_DEBUGPIN_B
	#include "arduino2.h"  // include the fast I/O 2 functions
#endif
 
 
#ifndef NOP
	#define NOP __asm__ __volatile__ ("nop\n\t") // delay with a single clock of no-operate instruction
#endif

 
#define IIC_SPEED_DEFAULT 100 // Units = KHz
#define CHIP_SIZE 256  // For now, only support 8bit IIC communications.  10bit addresses do exist, but this is not yet supported
 
 const uint8_t TIMEOUT_CLOCKS_BYTE_SLACK = 1;
 const uint8_t TIMEOUT_CLOCKS_BYTE = 8+1+TIMEOUT_CLOCKS_BYTE_SLACK;
 const uint8_t TIMEOUT_CLOCKS_TRANSACTION = 64;  // NOTE: *255
 const uint8_t TIMEOUT_CLOCKS_SENDACK = 4;


//			PLEASE NOTE THAT THIS LIBRARY USES 7 BIT ADDRESSES FOR ALL 'SMART' FUNCTIONS AND 8 BIT ADDRESES FOR LOWER LEVEL FUNCTIONS.
class SoftIIC
{
  public:  
	  
						// Constructor
	  // note: speed is in KHz, ~10-100 is reasonable.    										
    SoftIIC(uint8_t pin_scl, uint8_t pin_sda, uint16_t speed, bool pullups, bool multimastersupport, bool timeout);
	~SoftIIC();
	
						// Higher level master functions
	//MasterDumpAll scans the entire iic address space for devices which ACK an address, reading and printing all registers that were read from each device.
	// Returns: number of detected devices (0-127)
	uint8_t MasterDumpAll();
	
	//MasterDumpRegisters scans an iic address reading and printing all registers that were read from the device.
	// Returns: number of successfully read registers
	uint8_t MasterDumpRegisters(uint8_t addressRW);
	
	// MasterReadByte attempts to read a single byte from the target chip and register and places it in the 'value' pointer location.  
	// Arguments: device_address is the 7-bit iic address, register_address is the register within the chip, and value is a pointer to a uint8_t for the read data.
	// Returns: RETVAL_SUCCESS, RETVAL_PROTOCOL_FAILURE, or RETVAL_TIMEOUT	
	uint8_t MasterReadByte(uint8_t device_address, uint8_t register_address, uint8_t* value);	
	
	// MasterReadByte  attempts to read a single byte from the target chip and register and returns it.  
	// Arguments: device_address is the 7-bit iic address, register_address is the register within the chip.
	// Returns: the byte read.  if no byte was read, returns 0xff.
	uint8_t MasterReadByte(uint8_t device_address, uint8_t register_address);
		
	// MasterWriteByte attempts to write a single byte to the target chip and register. 
	// Returns: RETVAL_SUCCESS, RETVAL_PROTOCOL_FAILURE, or RETVAL_TIMEOUT		
	uint8_t MasterWriteByte(uint8_t device_address, uint8_t register_address, uint8_t value);
	
	// MasterWriteByte attempts to write a single byte to the target chip and register.  After a delay, the register is read and verified.
	// Returns: RETVAL_SUCCESS, RETVAL_PROTOCOL_FAILURE, or RETVAL_TIMEOUT		
	uint8_t MasterWriteByte(uint8_t device_address, uint8_t register_address, uint8_t value, uint8_t writeverify);
	
	// MasterCheckExists inserts an empty transaction to the targeted device and listens for an ACK.
	// Returns: RETVAL_SUCCESS if a device responded to the transaction with an ACK, RETVAL_PROTOCOL_FAILURE or RETVAL_TIMEOUT otherwise
	uint8_t MasterCheckExists(uint8_t device_address);
	
	// Master functions for doing multibyte transfers.  These should be significantly faster for doing things like programming an eeprom.
	uint16_t MasterReadPage (uint8_t device_address, uint8_t register_address, uint16_t number_bytes, uint8_t* bytes);
	uint16_t MasterReadPage (uint8_t device_address, uint8_t register_address, uint16_t number_bytes, uint8_t isverify, uint8_t* bytes);
	uint16_t MasterWritePage(uint8_t device_address, uint8_t register_address, uint16_t number_bytes, uint16_t pagesize, uint8_t* bytes);
	uint16_t MasterWritePage(uint8_t device_address, uint8_t register_address, uint16_t number_bytes, uint16_t pagesize, uint8_t writeverify, uint8_t* bytes);
	
						// Lower level master functions do all of the legwork of handling the iic master protocol.  
 	uint8_t MasterBusRestart();
 	uint8_t MasterStart();
 	uint8_t MasterStart(uint8_t device_address);
 	uint8_t MasterRestart(uint8_t device_address);
 	void 	MasterStop();
 	uint8_t MasterWrite(uint8_t value);
 	uint8_t MasterRead(bool last);
	uint8_t wait_until_bus_is_idle();
	
	
// Watches the I2c bus and prints transactions.  Never returns.  Useful for observing systems and testing the library.
	// Returns: number of bytes snooped.
	uint16_t Snoop(uint16_t to_snoop);
	
// Toggles the debug pin in response to incoming events (0, 1, start, stop)	
	void 		test_input_response_time();
	
	
// Toggles the debug pins a few times.  Useful as a sanity-check or to ensure probing the right pin.
	void 		debug_pin_test();
	void 			PrintSpeed();  
	
// Service one slave request.  Call this function by providing function pointers to functions which will provide the data necessary to handle the transaction.  See example code.
// Returns: number of bytes successfully handled.
	uint16_t SlaveHandleTransaction(
				uint8_t (*fp_respond_to_address)(uint8_t chip_address),
				uint8_t (*fp_respond_to_command)(uint8_t chip_address),
				uint8_t (*fp_respond_to_data)(uint8_t chip_address),
				uint8_t (*fp_get_register_address)(uint8_t chip_address),
				uint8_t (*fp_set_register_address)(uint8_t chip_address, uint8_t reg_address ),
				uint8_t (*fp_generate_byte)(uint8_t chip_address, uint8_t* byte),
				uint8_t (*fp_receive_byte)(uint8_t chip_address, uint8_t byte)
	);

									uint8_t	 		are_timeouts_enabled();
									uint8_t			GetStateSoftIIC();
	
	
  private:  			
	  
			// Pin interactions and precomputed bitmasks
		const uint8_t usePullups			;
		const uint8_t MULTIMASTERSUPPORT	;
		const uint8_t TimeoutsEnabled		;
	  	const uint8_t PIN_SCL				;
		const uint8_t PIN_SDA				;
		const uint8_t _sclBitMask			;
		const uint8_t _sclBitMaskINVERTED	;
		const uint8_t _sdaBitMask			;
		const uint8_t _sdaBitMaskINVERTED	;
		const uint8_t _busBitMask			;
		const volatile uint8_t* _sclPortRegIN			;
		const volatile uint8_t* _sdaPortRegIN 		;
		volatile uint8_t* _sclPortReg  			;
		volatile uint8_t* _sclDirReg   			;
		volatile uint8_t* _sdaPortReg  			;
		volatile uint8_t* _sdaDirReg			;	
	  
			// IIC timing related functions/variables
		const	uint16_t 		IIC_CLOCK_SPEED;				// Units are ticks of TCNT1
			uint16_t 		HalfclockTimeout;				// Units are ticks of TCNT1
			uint16_t 		TransferTimeout;				// Units are ticks of TCNT1
			uint16_t 		ByteTimeout;					// Units are ticks of TCNT1
			uint16_t 		ByteAckTimeout;					// Units are ticks of TCNT1
	  
	  
	  
			// Initialization functions
					void 			SetSpeed(uint16_t speed);  
FORCE_INLINE		void 			spin_until_half_clock_period(); 

			// IIC slave mode timing related functions/variables
FORCE_INLINE		uint8_t 		spin_until_start();
FORCE_INLINE		uint8_t			spin_until_clock_rises();
FORCE_INLINE		uint8_t 		spin_until_clock_falls();

			
			// IIC slave mode lower level functions
FORCE_INLINE		void	 		wait_for_bus_activity();
FORCE_INLINE		uint8_t 		set_next_bus_activity(uint8_t value);
FORCE_INLINE		uint8_t 		get_byte(uint8_t* value);												//Note: does not handle ACK/NACK. 
FORCE_INLINE		uint8_t 		get_byte(uint8_t* value, uint8_t (*my_ack_function)(uint8_t rxbyte));	//Note: handles ACK/NACK
FORCE_INLINE		uint8_t 		set_byte(uint8_t value);
									
			// IIC state functions
					uint8_t 		IIC_STATE; 
					uint8_t 		IIC_STATE_PREVIOUS; 
FORCE_INLINE		uint8_t 		StateIdle();
FORCE_INLINE		uint8_t 		StateStart(); 
FORCE_INLINE		uint8_t 		StateStop(); 
FORCE_INLINE		uint8_t 		StateData();
FORCE_INLINE		uint8_t 		StateClockFell();
FORCE_INLINE		uint8_t 		StateDataBit(); 
FORCE_INLINE		uint8_t 		StateClockLow(); 
FORCE_INLINE		uint8_t 		StateClockHigh();

		
FORCE_INLINE		void 			clock_pull_down();
FORCE_INLINE		void 			data_pull_down();
FORCE_INLINE		void 			clock_release();
FORCE_INLINE		void 			data_release();
FORCE_INLINE		uint8_t			clock_read();
FORCE_INLINE		uint8_t			data_read();
FORCE_INLINE		void 			bus_read();
FORCE_INLINE		void 			bus_read_same_port();
FORCE_INLINE		void 			bus_read_different_port();		
// These items are pointers to hardware which will definitely change state beyond the control of program flow (volatile)
			
			// Timer interaction functions and saved previous state
inline				void 			reset_timer();
inline				void			RestoreTimer1Settings();
inline				void			SaveTimer1Settings();
inline				void			ConfigureTimer1Settings();
FORCE_INLINE		uint16_t 		get_timer();
FORCE_INLINE		void	 		TransferTimerConfigureTransfer();
FORCE_INLINE		void	 		TransferTimerConfigureByte();
FORCE_INLINE		void	 		TransferTimerConfigureByteWithAck();
FORCE_INLINE		void	 		TransferTimerConfigureHalfclock();
FORCE_INLINE		void	 		TimerReset();
FORCE_INLINE		void	 		TimerClearMatch();
FORCE_INLINE		uint8_t 		TimerElapsed();
inline				uint8_t 		GetCurrentTimer1Divider();
inline				uint16_t 		GetRatioOfTimer1Divider(uint8_t divider);
inline				uint16_t 		GetFrequencyOfTimer1Divider(uint8_t divider);
inline				uint8_t 		GetCurrentTimer2Divider();
inline				uint16_t 		GetRatioOfTimer2Divider(uint8_t divider);
inline				uint16_t 		GetFrequencyOfTimer2Divider(uint8_t divider);
					uint8_t 		p_TCCR1A;
					uint8_t 		p_TCCR1B;
					uint8_t 		p_OCR1A;
					uint8_t 		p_OCR1B;
					uint8_t 		p_TIMSK1; 
					uint8_t 		p_TIFR1;   
					uint8_t 		my_TCCR1A;
					uint8_t 		my_TCCR1B;
					uint8_t 		my_OCR1A;
					uint8_t 		my_OCR1B;
					uint8_t 		my_TIMSK1; 
					uint8_t 		my_TIFR1;   
					uint8_t 		timer_is_configured;   
					
					// Debugging functions+state
					uint8_t 		DEBUG_PIN_A_STATE;
					uint8_t 		DEBUG_PIN_B_STATE;
					void			InitDebugpins();
FORCE_INLINE		void			debug_pin_A_test();
FORCE_INLINE		void			debug_pin_B_test();
FORCE_INLINE		void 			debug_pin_A_low();
FORCE_INLINE		void 			debug_pin_B_low();
FORCE_INLINE		void 			debug_pin_A_high();
FORCE_INLINE		void 			debug_pin_B_high();
FORCE_INLINE		void 			debug_pin_A_toggle();
FORCE_INLINE		void 			debug_pin_B_toggle();
FORCE_INLINE		void 			fastprinthexbyte(uint8_t hexbyte);	
FORCE_INLINE		void			functimer_start();
FORCE_INLINE		void			functimer_stop();
};
#endif
