// License: eh, not sure yet.  Free to use for all noncommercial purposes.  If you modify it a little, don't claim it as your own.
// cirthix@gmail.com
// VERSION 0.1


#ifndef SOFTIIC_h
#define SOFTIIC_h

#include <Arduino.h>
#include <inttypes.h>

#ifndef NOP
#define NOP __asm__ __volatile__ ("nop\n\t") // delay with a single clock of no-operate instruction
#endif

#define DEBUGPIN_A 14		// This debug-output pin is used to verify timing relationships between the code and physical sda/scl pins.
#define DEBUGPIN_B 15		// This debug-output pin is used to verify timing relationships between the code and physical sda/scl pins.

#ifdef DEBUGPIN_A
#include "arduino2.h"  // include the fast I/O 2 functions
#endif

#ifdef DEBUGPIN_B
#include "arduino2.h"  // include the fast I/O 2 functions
#endif


#define I2C_SPEED_DEFAULT 100 // Units = KHz
#define CHIP_SIZE 256


//			PLEASE NOTE THAT THIS LIBRARY USES 7 BIT ADDRESSES FOR ALL 'SMART' FUNCTIONS AND 8 BIT ADDRESES FOR LOWER LEVEL FUNCTIONS.
class SoftI2C
{
  public:  
	  
						// Constructors
#ifdef SCL_PIN
#ifdef SDA_PIN
SoftI2C();
#endif
#endif
    SoftI2C(uint8_t pin_scl, uint8_t pin_sda);
    SoftI2C(uint8_t pin_scl, uint8_t pin_sda, bool pullups);
    SoftI2C(uint8_t pin_scl, uint8_t pin_sda, bool pullups, uint16_t speed);
	~SoftI2C();
	
						// Higher level master functions
	//MasterDumpAll scans the entire i2c address space for devices which ACK an address, reading and printing all registers that were read from each device.
	// Returns: number of detected devices (0-127)
	uint8_t MasterDumpAll();
	
	//MasterDumpRegisters scans an i2c address reading and printing all registers that were read from the device.
	// Returns: number of successfully read registers
	uint8_t MasterDumpRegisters(uint8_t addressRW);
	
	// MasterReadByte attempts to read a single byte from the target chip and register and places it in the 'value' pointer location.  
	// Arguments: device_address is the 7-bit i2c address, register_address is the register within the chip, and value is a pointer to a uint8_t for the read data.
	// Returns: RETVAL_SUCCESS, RETVAL_PROTOCOL_FAILURE, or RETVAL_TIMEOUT	
	uint8_t MasterReadByte(uint8_t device_address, uint8_t register_address, uint8_t* value);	
	
	// MasterReadByte  attempts to read a single byte from the target chip and register and returns it.  
	// Arguments: device_address is the 7-bit i2c address, register_address is the register within the chip.
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
	
	// Not yet implemented
	uint8_t MasterReadPage(uint8_t device_address, uint8_t register_address, uint8_t* bytes, uint8_t number_bytes);
	uint8_t MasterWritePage(uint8_t device_address, uint8_t register_address, uint8_t* bytes, uint8_t number_bytes);
	
						// Lower level master functions do all of the legwork of handling the i2c master protocol.  
 	uint8_t MasterBusRestart();
 	uint8_t MasterStart();
 	uint8_t MasterStart(uint8_t device_address);
 	uint8_t MasterRestart(uint8_t device_address);
 	void 	MasterStop();
 	uint8_t MasterWrite(uint8_t value);
 	uint8_t MasterRead(bool last);

// Watches the I2c bus and prints transactions.  Never returns.  Useful for observing systems and testing the library.
	// Returns: number of bytes snooped.
	uint16_t Snoop(uint16_t to_snoop);
	
// Toggles the debug pin in response to incoming events (0, 1, start, stop)	
	void 		test_input_response_time();
	
	
// Toggles the debug pins a few times.  Useful as a sanity-check or to ensure probing the right pin.
	void 		debug_pin_test();
	
// Service one slave request.  Call this function by providing function pointers to functions which will provide the data necessary to handle the transaction.  See example code.
// Returns: number of bytes successfully handled.
	uint8_t SlaveHandleTransaction(
				bool (*fp_respond_to_chip_address)(uint8_t chip_address),
				uint8_t (*fp_get_register_address)(uint8_t chip_address),
				uint8_t (*fp_set_register_address)(uint8_t chip_address, uint8_t reg_address ),
				uint8_t (*fp_generate_byte)(uint8_t chip_address, uint8_t* byte),
				uint8_t (*fp_receive_byte)(uint8_t chip_address, uint8_t byte)
	);

	
  private:  
			uint8_t 	p_TCCR1A;
			uint8_t 	p_TCCR1B;
			uint8_t 	p_OCR1A;
			uint8_t 	p_OCR1B;
			uint8_t 	p_TIMSK1; 
			uint8_t 	p_TIFR1;   
			uint8_t 	PIN_SCL;
			uint8_t 	PIN_SDA;
		
			uint8_t 	_sclBitMask;
			uint8_t 	_sdaBitMask;
			uint8_t 	_busBitMask;
			// These items are pointers to hardware which will definitely change state beyond the control of program flow.
volatile 	uint8_t* 	_sclPortRegIN;
volatile 	uint8_t* 	_sclPortReg;
volatile 	uint8_t* 	_sclDirReg;
volatile 	uint8_t* 	_sdaPortRegIN;
volatile 	uint8_t* 	_sdaPortReg;
volatile 	uint8_t* 	_sdaDirReg;
    
			uint8_t 	usePullups;  
			
	void 	SetSpeed(uint16_t speed);  // note: speed is in KHz, ~10-100 is reasonable
			uint16_t 	I2C_SPEED;						// Units are KHz
			uint16_t 	half_clock_period;				// Units are ticks of TCNT1
			uint16_t 	transmission_timeout;  // units are ticks of TCNT1
			uint8_t 	timeout_is_enabled;
    
			uint8_t 	IIC_STATE; 
			
#ifdef DEBUGPIN_A
			uint8_t 	DEBUG_PIN_A_STATE;
#endif
#ifdef DEBUGPIN_B
			uint8_t 	DEBUG_PIN_B_STATE;
#endif
			void		InitDebugpins();
		void		debug_pin_A_test();
		void		debug_pin_B_test();
  	void 		debug_pin_A_low();
  	void 		debug_pin_B_low();
  	void 		debug_pin_A_high();
  	void 		debug_pin_B_high();
  	void 		debug_pin_A_toggle();
  	void 		debug_pin_B_toggle();
 		void 		fastprinthexbyte(uint8_t hexbyte);	
		
		
			void 		init(uint8_t pin_scl, uint8_t pin_sda, uint8_t pullups, uint16_t speed);
		void 		wait_for_half_clock_period();
 		void 		reset_timer();
 		uint16_t 	get_timer();
  	void 		set_timer(uint16_t value);
 		uint8_t		transmission_timeout_reached();

 		uint8_t 	spin_until_start();
 		uint8_t		spin_until_clock_rises();
 		uint8_t 	spin_until_clock_falls();
 		uint8_t 	get_next_bus_activity();
 		uint8_t 	get_next_raw_bus_activity();
 		uint8_t 	set_next_bus_activity(uint8_t value);
			uint8_t 	get_byte(uint8_t* value);
			uint8_t 	set_byte(uint8_t value);
			
		void 		clock_pull_down();
		void 		data_pull_down();
 		void 		clock_release();
 		void 		data_release();
 		void 		clock_read(uint8_t* SCL);
 		void 		data_read(uint8_t* SDA);
 		void 		bus_read();
 		void		functimer_start();
 		void		functimer_stop();
			uint8_t 	GetCurrentTimer1Divider();
			uint16_t 	GetRatioOfTimer1Divider(uint8_t divider);
			uint16_t 	GetFrequencyOfTimer1Divider(uint8_t divider);
			uint8_t 	GetCurrentTimer2Divider();
			uint16_t 	GetRatioOfTimer2Divider(uint8_t divider);
			uint16_t 	GetFrequencyOfTimer2Divider(uint8_t divider);
};
#endif
