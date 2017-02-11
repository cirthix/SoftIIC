/*
 * SoftIIC, a library for IIC communications on any pins.
 * 
 * Copyright (C) 2015 cirthix@gmail.com
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
 * 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of the GNU
 * General Public License along with This software. If not, see <http://www.gnu.org/licenses/>.
 * 
 * Authors: cirthix@gmail.com
 */

// This library takes some tricks from https://github.com/todbot/SoftI2CMaster

 
 
#include <SoftIIC.h>

			
//static const 		uint8_t 	IIC_STATE_TIMEOUT_SHIFT  		=	7                                                         ;
//static const 		uint8_t 	IIC_STATE_TIMEOUT		  		=	0b00000001<<IIC_STATE_TIMEOUT_SHIFT	                      ;
static const 		uint8_t 	IIC_STATE_MASK_SCL_SHIFT  		=	4                                                         ;
static const 		uint8_t 	IIC_STATE_MASK_SDA_SHIFT  		=	0                                                         ;
static const 		uint8_t 	IIC_STATE_MASK_SCL_CURRENT  	=	0b00000001<<IIC_STATE_MASK_SCL_SHIFT                      ;
static const 		uint8_t 	IIC_STATE_MASK_SDA_CURRENT  	=	0b00000001<<IIC_STATE_MASK_SDA_SHIFT                      ;
static const 		uint8_t 	IIC_STATE_MASK_SCL_PREVIOUS 	=	IIC_STATE_MASK_SCL_CURRENT<<1                             ;
static const 		uint8_t 	IIC_STATE_MASK_SDA_PREVIOUS 	=	IIC_STATE_MASK_SDA_CURRENT<<1                             ;
static const 		uint8_t 	IIC_STATE_MASK_SCL				=	IIC_STATE_MASK_SCL_CURRENT|IIC_STATE_MASK_SCL_PREVIOUS    ;
static const 		uint8_t 	IIC_STATE_MASK_SDA				=	IIC_STATE_MASK_SDA_CURRENT|IIC_STATE_MASK_SDA_PREVIOUS    ;
static const 		uint8_t 	IIC_STATE_MASK_CURRENT      	=	IIC_STATE_MASK_SCL_CURRENT|IIC_STATE_MASK_SDA_CURRENT     ;
static const 		uint8_t 	IIC_STATE_MASK_PREVIOUS     	=	IIC_STATE_MASK_SCL_PREVIOUS|IIC_STATE_MASK_SDA_PREVIOUS   ;


static const 		uint8_t 	IIC_STATE_CLOCK_ZERO_DATA_ZERO	=	(0x00&IIC_STATE_MASK_SCL_PREVIOUS)|(0x00&IIC_STATE_MASK_SCL_CURRENT)|(0x00&IIC_STATE_MASK_SDA_PREVIOUS)|(0x00&IIC_STATE_MASK_SDA_CURRENT) ;
static const 		uint8_t 	IIC_STATE_CLOCK_ZERO_DATA_ROSE	=	(0x00&IIC_STATE_MASK_SCL_PREVIOUS)|(0x00&IIC_STATE_MASK_SCL_CURRENT)|(0x00&IIC_STATE_MASK_SDA_PREVIOUS)|(0xFF&IIC_STATE_MASK_SDA_CURRENT) ;
static const 		uint8_t 	IIC_STATE_CLOCK_ZERO_DATA_FELL	=	(0x00&IIC_STATE_MASK_SCL_PREVIOUS)|(0x00&IIC_STATE_MASK_SCL_CURRENT)|(0xFF&IIC_STATE_MASK_SDA_PREVIOUS)|(0x00&IIC_STATE_MASK_SDA_CURRENT) ;
static const 		uint8_t 	IIC_STATE_CLOCK_ZERO_DATA_HIGH	=	(0x00&IIC_STATE_MASK_SCL_PREVIOUS)|(0x00&IIC_STATE_MASK_SCL_CURRENT)|(0xFF&IIC_STATE_MASK_SDA_PREVIOUS)|(0xFF&IIC_STATE_MASK_SDA_CURRENT) ;
static const 		uint8_t 	IIC_STATE_CLOCK_ROSE_DATA_ZERO	=	(0x00&IIC_STATE_MASK_SCL_PREVIOUS)|(0xFF&IIC_STATE_MASK_SCL_CURRENT)|(0x00&IIC_STATE_MASK_SDA_PREVIOUS)|(0x00&IIC_STATE_MASK_SDA_CURRENT) ;
static const 		uint8_t 	IIC_STATE_CLOCK_ROSE_DATA_ROSE	=	(0x00&IIC_STATE_MASK_SCL_PREVIOUS)|(0xFF&IIC_STATE_MASK_SCL_CURRENT)|(0x00&IIC_STATE_MASK_SDA_PREVIOUS)|(0xFF&IIC_STATE_MASK_SDA_CURRENT) ;
static const 		uint8_t 	IIC_STATE_CLOCK_ROSE_DATA_FELL	=	(0x00&IIC_STATE_MASK_SCL_PREVIOUS)|(0xFF&IIC_STATE_MASK_SCL_CURRENT)|(0xFF&IIC_STATE_MASK_SDA_PREVIOUS)|(0x00&IIC_STATE_MASK_SDA_CURRENT) ;
static const 		uint8_t 	IIC_STATE_CLOCK_ROSE_DATA_HIGH	=	(0x00&IIC_STATE_MASK_SCL_PREVIOUS)|(0xFF&IIC_STATE_MASK_SCL_CURRENT)|(0xFF&IIC_STATE_MASK_SDA_PREVIOUS)|(0xFF&IIC_STATE_MASK_SDA_CURRENT) ;
static const 		uint8_t 	IIC_STATE_CLOCK_FELL_DATA_ZERO	=	(0xFF&IIC_STATE_MASK_SCL_PREVIOUS)|(0x00&IIC_STATE_MASK_SCL_CURRENT)|(0x00&IIC_STATE_MASK_SDA_PREVIOUS)|(0x00&IIC_STATE_MASK_SDA_CURRENT) ;
static const 		uint8_t 	IIC_STATE_CLOCK_FELL_DATA_ROSE	=	(0xFF&IIC_STATE_MASK_SCL_PREVIOUS)|(0x00&IIC_STATE_MASK_SCL_CURRENT)|(0x00&IIC_STATE_MASK_SDA_PREVIOUS)|(0xFF&IIC_STATE_MASK_SDA_CURRENT) ;
static const 		uint8_t 	IIC_STATE_CLOCK_FELL_DATA_FELL	=	(0xFF&IIC_STATE_MASK_SCL_PREVIOUS)|(0x00&IIC_STATE_MASK_SCL_CURRENT)|(0xFF&IIC_STATE_MASK_SDA_PREVIOUS)|(0x00&IIC_STATE_MASK_SDA_CURRENT) ;
static const 		uint8_t 	IIC_STATE_CLOCK_FELL_DATA_HIGH	=	(0xFF&IIC_STATE_MASK_SCL_PREVIOUS)|(0x00&IIC_STATE_MASK_SCL_CURRENT)|(0xFF&IIC_STATE_MASK_SDA_PREVIOUS)|(0xFF&IIC_STATE_MASK_SDA_CURRENT) ;
static const 		uint8_t 	IIC_STATE_CLOCK_HIGH_DATA_ZERO	=	(0xFF&IIC_STATE_MASK_SCL_PREVIOUS)|(0xFF&IIC_STATE_MASK_SCL_CURRENT)|(0x00&IIC_STATE_MASK_SDA_PREVIOUS)|(0x00&IIC_STATE_MASK_SDA_CURRENT) ;
static const 		uint8_t 	IIC_STATE_CLOCK_HIGH_DATA_ROSE	=	(0xFF&IIC_STATE_MASK_SCL_PREVIOUS)|(0xFF&IIC_STATE_MASK_SCL_CURRENT)|(0x00&IIC_STATE_MASK_SDA_PREVIOUS)|(0xFF&IIC_STATE_MASK_SDA_CURRENT) ;
static const 		uint8_t 	IIC_STATE_CLOCK_HIGH_DATA_FELL	=	(0xFF&IIC_STATE_MASK_SCL_PREVIOUS)|(0xFF&IIC_STATE_MASK_SCL_CURRENT)|(0xFF&IIC_STATE_MASK_SDA_PREVIOUS)|(0x00&IIC_STATE_MASK_SDA_CURRENT) ;
static const 		uint8_t 	IIC_STATE_CLOCK_HIGH_DATA_HIGH	=	(0xFF&IIC_STATE_MASK_SCL_PREVIOUS)|(0xFF&IIC_STATE_MASK_SCL_CURRENT)|(0xFF&IIC_STATE_MASK_SDA_PREVIOUS)|(0xFF&IIC_STATE_MASK_SDA_CURRENT) ;

			// Using these values helps us optimize by not performing unnecessary conversions and checks
static const 		uint8_t		IIC_ZERO		=	0x00;		// Rising edge of clock with data = 0
static const 		uint8_t		IIC_ONE			=	IIC_STATE_MASK_SDA_CURRENT;		// Rising edge of clock with data = 1
static const 		uint8_t		IIC_CLOCKFALL	=	IIC_STATE_MASK_SCL_PREVIOUS;		// Falling edge of clock
static const 		uint8_t		IIC_START		=	IIC_STATE_CLOCK_HIGH_DATA_FELL;		// Data going low while clock remains high
static const 		uint8_t		IIC_STOP		=	IIC_STATE_CLOCK_HIGH_DATA_ROSE;		// Data going high while clock remains high
static const 		uint8_t		IIC_TIMEOUT		=	0xFF;		// ANY STATIC CONDITION THAT EXCEEDS THE TIMEOUT TIME	

				
static const		uint8_t 	IIC_RWMASK 		= 	0x01; 	//Byte to AND with address for read/write bit 
static const		uint8_t 	IIC_READ 		= 	0x01; 	//Byte to OR with address for read start and read restart 
static const		uint8_t 	IIC_WRITE 		= 	0x00; 	//Byte to OR with address for write start and write restart 



  // ENUMERATED FUNCTION RETURN VALUE POSSIBILITES
			// Using these values helps us optimize by not performing unnecessary conversions and checks
#define RETVAL_SUCCESS 					IIC_ZERO	// The function has exited successfully.
#define RETVAL_UNEXPECTED_START			IIC_START	// During operation, the bus encountered an unexpected start condition.
#define RETVAL_UNEXPECTED_STOP			IIC_STOP	// During operation, the bus encountered an unexpected stop condition.
#define RETVAL_PROTOCOL_FAILURE			IIC_ONE	// Probably the communication was NAK'd.  Only used for master functions.
#define RETVAL_TIMEOUT					IIC_TIMEOUT	// The bus timed out within the function.
  




 // Tune this with a scope for your chip/compiler such that a setting of '100' yields 100KHz rate.
const		uint16_t 	 TIMER_OVERHEAD_COMPENASTION	=4 ;

////////////// Public Methods ////////////////////////////////////////

	
uint8_t SoftIIC::MasterCheckExists(uint8_t device_address){
	uint8_t retval;		
//	SoftIIC::functimer_start();
		retval=SoftIIC::MasterStart(device_address);
		SoftIIC::MasterStop();
//	SoftIIC::functimer_stop();
	return retval;
	}


uint8_t SoftIIC::MasterReadByte(uint8_t device_address, uint8_t register_address ){
	uint8_t retval=0xFF;
	SoftIIC::MasterReadByte(device_address, register_address, &retval);
	return retval;
}
	
uint8_t SoftIIC::MasterReadByte(uint8_t device_address, uint8_t register_address , uint8_t* value ){
	uint8_t tretval;
  tretval=SoftIIC::MasterStart((device_address<<1)| IIC_WRITE);  		if (tretval != RETVAL_SUCCESS) {  SoftIIC::MasterStop(); return tretval;}
  tretval=SoftIIC::MasterWrite(register_address);					  	if (tretval != RETVAL_SUCCESS) {  SoftIIC::MasterStop(); return tretval;}
  tretval=SoftIIC::MasterRestart((device_address<<1)| IIC_READ);  		if (tretval != RETVAL_SUCCESS) {  SoftIIC::MasterStop(); return tretval;}
  *value = SoftIIC::MasterRead(true);    
  SoftIIC::MasterStop();  
  return RETVAL_SUCCESS;
}

uint16_t SoftIIC::MasterReadPage(uint8_t device_address, uint8_t register_address, uint16_t number_bytes, uint8_t* bytes){
	return SoftIIC::MasterReadPage(device_address, register_address, number_bytes, 0, bytes);
}

uint16_t SoftIIC::MasterReadPage(uint8_t device_address, uint8_t register_address, uint16_t number_bytes, uint8_t isverify, uint8_t* bytes){
	uint8_t tretval;
	uint16_t k=0;
	uint8_t tmp;
	tretval=SoftIIC::MasterStart((device_address<<1)| IIC_WRITE);  		if (tretval != RETVAL_SUCCESS) {  SoftIIC::MasterStop(); return tretval;}
	tretval=SoftIIC::MasterWrite(register_address);					  	if (tretval != RETVAL_SUCCESS) {  SoftIIC::MasterStop(); return tretval;}
	tretval=SoftIIC::MasterRestart((device_address<<1)| IIC_READ);  	if (tretval != RETVAL_SUCCESS) {  SoftIIC::MasterStop(); return tretval;}
	while( k<number_bytes){	
	    tmp=SoftIIC::MasterRead(k==number_bytes-1); // NACK the last transaction to indicate end of sequence	
	    if(isverify) {
			if(tmp!=bytes[k]) {SoftIIC::MasterStop(); return RETVAL_PROTOCOL_FAILURE;}
			}
	    else {bytes[k]=tmp;}
		k++;
	}
	SoftIIC::MasterStop();  
	return RETVAL_SUCCESS;	
}

	
uint8_t SoftIIC::MasterWriteByte(uint8_t device_address, uint8_t register_address, uint8_t data){
	return SoftIIC::MasterWriteByte(device_address, register_address, data, 0);
}
	
uint8_t SoftIIC::MasterWriteByte(uint8_t device_address, uint8_t register_address, uint8_t data, uint8_t writeverify ){	
	if ((SoftIIC::MasterStart((device_address<<1)| IIC_WRITE))) {  SoftIIC::MasterStop(); return RETVAL_PROTOCOL_FAILURE;}
	if (SoftIIC::MasterWrite(register_address)) {  SoftIIC::MasterStop(); return RETVAL_PROTOCOL_FAILURE;}
	if (SoftIIC::MasterWrite(data)) {  SoftIIC::MasterStop(); return RETVAL_PROTOCOL_FAILURE;}
    SoftIIC::MasterStop();  
  
  if (writeverify != 0) {
  delayMicroseconds(10000); // Typical write time for iic eeprom is 5-10ms.  Give the target chip some time to write the data.
    uint8_t readback_value;
    readback_value = SoftIIC::MasterReadByte(device_address, register_address);
    if (readback_value == data) {      return RETVAL_SUCCESS;    }
    else {
//           Serial.print("ERROR: Chip=0x") ;    if (device_address<16) {Serial.print("0");}   Serial.print(device_address,HEX);    
//           Serial.print(", Reg=0x")   ;  if (register_address<16) {Serial.print("0");}   Serial.print(register_address,HEX);      
//           Serial.print(", wrote=0x") ;    if (data<16) {Serial.print("0");}   Serial.print(data,HEX);      
//           Serial.print(", read=0x") ;    if (readback_value<16) {Serial.print("0");}   Serial.print(readback_value,HEX);  
//		   Serial.println("");
 
      return RETVAL_PROTOCOL_FAILURE;
    }
	 return RETVAL_SUCCESS;  
  }  
  return RETVAL_SUCCESS;
}

// By default, do not do write verification
uint16_t SoftIIC::MasterWritePage(uint8_t device_address, uint8_t register_address, uint16_t number_bytes, uint8_t* bytes){ 
	return SoftIIC::MasterWritePage( device_address,  register_address, number_bytes, 0,  bytes );
}


uint16_t SoftIIC::MasterWritePage(uint8_t device_address, uint8_t register_address, uint16_t number_bytes, uint8_t writeverify, uint8_t* bytes ){	
    uint16_t retval;
	uint16_t k=0;
	if ((SoftIIC::MasterStart((device_address<<1)| IIC_WRITE))) {  SoftIIC::MasterStop(); return RETVAL_PROTOCOL_FAILURE;}
	if (SoftIIC::MasterWrite(register_address)) {  SoftIIC::MasterStop(); return RETVAL_PROTOCOL_FAILURE;}
	
	while( k<number_bytes){
	if (SoftIIC::MasterWrite(bytes[k])) {  SoftIIC::MasterStop(); return RETVAL_PROTOCOL_FAILURE;}
		k++;
//		Serial.print(".");
	}		
    SoftIIC::MasterStop(); 
	if (writeverify == 0) {return RETVAL_SUCCESS;	}
	delayMicroseconds(10000); // Typical write time for iic eeprom is 5ms.  Give the target chip some time to write the data.	
	return SoftIIC::MasterReadPage(device_address, register_address, number_bytes, 1, bytes);	
}

/*
uint8_t SoftIIC::MasterDumpAll() {
	Serial.println(F("IIC dump begin"));  Serial.flush();
	uint8_t addressRW;
	uint8_t number_of_chips;
	number_of_chips=0;
		uint8_t retval=0;
	for (addressRW = 0x01; addressRW < 0x7f; addressRW++ )	{
		retval=SoftIIC::MasterCheckExists(addressRW<<1); 
		if (retval == RETVAL_SUCCESS)    {
			number_of_chips++;
			SoftIIC::MasterDumpRegisters(addressRW);
		}
		delay(10);
	}
	Serial.println(F("IIC dump done."));  Serial.flush();
//	Serial.print(F("IIC dump complete, found ")); Serial.print(number_of_chips); Serial.println(F(" chips!"));  Serial.flush();
	return number_of_chips;
}
*/
	
uint8_t SoftIIC::MasterDumpAll() {
	Serial.println(F("IIC dump begin"));  Serial.flush();
	const uint16_t DELAY_BETWEEN_ACTIONS=100;
	uint8_t addressRW;
	uint8_t number_of_chips;
	number_of_chips=0;
		uint8_t retval=0;
	for (addressRW = 0x01; addressRW < 0x7F; addressRW++ )	{	
		wdt_reset();
		delay(DELAY_BETWEEN_ACTIONS); SoftIIC::MasterBusRestart(); delay(DELAY_BETWEEN_ACTIONS); 		
		retval=SoftIIC::MasterCheckExists(addressRW<<1);  // Check 'write' address
		if (retval == RETVAL_SUCCESS)    {
				number_of_chips++;
				Serial.print(F("Chip at 0x")); 	SoftIIC::fastprinthexbyte(addressRW<<1);	Serial.println(""); Serial.flush(); 	
		}		
		delay(DELAY_BETWEEN_ACTIONS); SoftIIC::MasterBusRestart(); delay(DELAY_BETWEEN_ACTIONS); 
		retval=SoftIIC::MasterCheckExists((addressRW<<1)|IIC_READ);  // Check 'read' address		
		if (retval == RETVAL_SUCCESS)    {
				number_of_chips++;
		delay(DELAY_BETWEEN_ACTIONS); SoftIIC::MasterBusRestart(); delay(DELAY_BETWEEN_ACTIONS); 
				SoftIIC::MasterDumpRegisters(addressRW);
		}
	}
	Serial.println(F("IIC dump done."));  Serial.flush();
//	Serial.print(F("IIC dump complete, found ")); Serial.print(number_of_chips); Serial.println(F(" chips!"));  Serial.flush();
	return number_of_chips;
}

uint8_t SoftIIC::MasterDumpRegisters(uint8_t addressRW) {	
	uint8_t number_of_registers=0;
	uint16_t register_address;
	uint8_t value;
	Serial.print(F("DumpRegs 0x")); 	SoftIIC::fastprinthexbyte(addressRW<<1);	Serial.print(":"); Serial.flush();
	for (register_address = 0; register_address < CHIP_SIZE; register_address++ )      {
		uint8_t readretval = SoftIIC::MasterReadByte(addressRW, register_address, &value);
		if(readretval!=RETVAL_SUCCESS) {Serial.print(F("-- "));}
		else {
			SoftIIC::fastprinthexbyte(value);
			Serial.print(" ");
			number_of_registers++;
		}
	}
	Serial.println(";");
	return number_of_registers;
}

 uint8_t SoftIIC::MasterBusRestart() {
	 SoftIIC::MasterStart();
	 SoftIIC::MasterStop();
  return RETVAL_SUCCESS;
}
	
 uint8_t SoftIIC::MasterStart() {
	 // MasterStart() is a forceful function.  It is only called from MasterBusRestart() which forcibly grabs and resets the bus.  It does not play nicely with other masters.
	SoftIIC::ConfigureTimer1Settings();
	SoftIIC::TransferTimerConfigureHalfclock();
	SoftIIC::TimerReset(); 
	SoftIIC::TimerClearMatch(); 
	SoftIIC::data_pull_down();
	SoftIIC::spin_until_half_clock_period();
	SoftIIC::clock_pull_down();
	SoftIIC::spin_until_half_clock_period();
	SoftIIC::spin_until_half_clock_period();
	SoftIIC::spin_until_half_clock_period();
  return RETVAL_SUCCESS;
}

 uint8_t SoftIIC::MasterStart(uint8_t addressRW) {
	SoftIIC::ConfigureTimer1Settings();
	SoftIIC::wait_until_bus_is_idle();
	SoftIIC::data_pull_down();
	SoftIIC::TransferTimerConfigureHalfclock();
	SoftIIC::TimerReset();
	SoftIIC::TimerClearMatch(); 
	SoftIIC::spin_until_half_clock_period();
	SoftIIC::clock_pull_down();
	SoftIIC::spin_until_half_clock_period();
	SoftIIC::spin_until_half_clock_period();
	SoftIIC::spin_until_half_clock_period();
  return SoftIIC::MasterWrite(addressRW);
}

 uint8_t SoftIIC::MasterRestart(uint8_t addressRW) {
	 // Don't need to check for other masters or activity because we already own the bus and know the state.
	SoftIIC::clock_pull_down();
	SoftIIC::data_release();
	SoftIIC::spin_until_half_clock_period();
	SoftIIC::clock_release();
	SoftIIC::spin_until_half_clock_period();
  return SoftIIC::MasterStart(addressRW);
}

 void SoftIIC::MasterStop() {
	SoftIIC::spin_until_half_clock_period();
	SoftIIC::spin_until_half_clock_period();
	SoftIIC::spin_until_half_clock_period();
	SoftIIC::data_pull_down();
	SoftIIC::spin_until_half_clock_period();
	SoftIIC::clock_release();
	SoftIIC::spin_until_half_clock_period();
	SoftIIC::data_release();
	SoftIIC::spin_until_half_clock_period();
	SoftIIC::RestoreTimer1Settings();
}

 uint8_t SoftIIC::MasterWrite(uint8_t value) {
//  Serial.print(F("Writing byte:"));  Serial.println(value,HEX);
	 	uint8_t m=0b10000000;
	while( m>0){	 
		if((value & m) == LOW){SoftIIC::data_pull_down();} else {SoftIIC::data_release();} 
		SoftIIC::spin_until_half_clock_period();
		SoftIIC::clock_release();
		m=m>>1;
		SoftIIC::spin_until_half_clock_period();
		SoftIIC::clock_pull_down();
	}
		
	SoftIIC::data_release(); 
	SoftIIC::spin_until_half_clock_period();
	uint8_t rtn=0x00;
	rtn=SoftIIC::data_read(); SoftIIC::data_release();
	SoftIIC::clock_release();
		SoftIIC::spin_until_half_clock_period();
		SoftIIC::clock_pull_down();
	if(rtn==0x00) {return RETVAL_SUCCESS;} else {return RETVAL_PROTOCOL_FAILURE;}
}


 uint8_t SoftIIC::MasterRead(bool last) {
	uint8_t value = 0x00;
	uint8_t rtn;
	uint8_t m=0b10000000;
	while( m>0){
		SoftIIC::spin_until_half_clock_period();
		SoftIIC::clock_release();
		m=m>>1;
		SoftIIC::spin_until_half_clock_period();
		rtn=SoftIIC::data_read(); value <<= 1; value=value+(rtn>0);
		SoftIIC::clock_pull_down();
	}
		
  // send Ack or Nak
  if((last) == true){SoftIIC::data_release();} else {SoftIIC::data_pull_down();} 
	SoftIIC::spin_until_half_clock_period();
	SoftIIC::clock_release();
	SoftIIC::spin_until_half_clock_period();
	SoftIIC::clock_pull_down();
	SoftIIC::data_release();
	return value;
}

 // This function is not a guarantee of bus-idleness, but is a decent works-in-most-cases compromise.  Note that in a busy bus, this can potentially never return.  Use the watchdog timer.
 // If multimaster support is enabled, this function will wait for a period of no bus activity OR a stop condition before returning (a start condition should immediately follow the successful return of this function)
uint8_t SoftIIC::wait_until_bus_is_idle(){
	if(MULTIMASTERSUPPORT){
		SoftIIC::TimerReset();
		SoftIIC::TransferTimerConfigureTransfer();
		SoftIIC::TimerClearMatch(); 
		SoftIIC::bus_read();
		while(1){
			SoftIIC::bus_read();
			if(SoftIIC::StateStop()){return RETVAL_SUCCESS; }
			if((SoftIIC::StateIdle())!=0){SoftIIC::TimerClearMatch(); }
			if(SoftIIC::TimerElapsed()) {return RETVAL_SUCCESS;}	
		}
	}else {
		SoftIIC::TransferTimerConfigureHalfclock();
		SoftIIC::TimerReset();
		SoftIIC::TimerClearMatch(); 
		// Here, we expect no other masters on the bus, but we don't know what state we may have entered this function on (maybe previous one crashed?)
		// Let's wait a few clocks to let slaves timeout if that is the case.
		uint8_t num_half_clocks_remaining=32;	
		while(num_half_clocks_remaining>0){	
			while((SoftIIC::TimerElapsed())==0) {		
				SoftIIC::bus_read();
				if((SoftIIC::StateIdle())!=0){return RETVAL_PROTOCOL_FAILURE; }		
			}
			num_half_clocks_remaining--;
		}
	}
	return RETVAL_SUCCESS;
}

  void SoftIIC::spin_until_half_clock_period(){	
	if(HalfclockTimeout>0){ 	
		while(1) if(SoftIIC::TimerElapsed()) {return;}
	}
  }
  	
  

  
  // Note: Because the snoop loop must run very quickly, it is critical that printing output does not slow the loop.  All testing done at 8MHz
 void SoftIIC::fastprinthexbyte(uint8_t hexbyte){	 
// 	 SoftIIC::debug_pin_A_high();
	 	 
	 // Baseline, do absoutely nothing: 3.7 microseconds for debug pins to toggle	 

	 // Dumb method: 203 microseconds 
//	 if (hexbyte < 16) {	Serial.print("0");	} Serial.print(hexbyte,HEX);
	 
	//	Serial.write(hexbyte);  //	 no formatting writing byte: 16.8 microseconds 	 
	//	Serial.print("A");  // 29 microseconds		
	//	Serial.print("AB");  // 47 microseconds		
	//	Serial.write('A');  // 17.5 microseconds
	//	Serial.write("AB",2);  // 43 microseconds	
	//	Serial.write('A'); Serial.write('B');  // 34.8 microseconds
	//  Serial.write("AB");  // 43 microseconds		 
	 // Interesting, so according to this, it is best to call serial.write twice, one for each nibble.

	 // Final? method: 39us
	uint8_t mychar=hexbyte>>4;
	if(mychar<0x0A) { Serial.write('0' + mychar);} else  { Serial.write('7' + mychar);}
	mychar=hexbyte&0x0F;
	if(mychar<0x0A) { Serial.write('0' + mychar);} else  { Serial.write('7' + mychar);}

	
// SoftIIC::debug_pin_A_low();
  }


  
uint16_t SoftIIC::Snoop(uint16_t to_snoop){  	
	SoftIIC::ConfigureTimer1Settings();
	SoftIIC::TimerReset();
	SoftIIC::TransferTimerConfigureByte();
	SoftIIC::TimerClearMatch(); 
	uint16_t activity_snooped=0;
	uint8_t tretval;
	uint8_t value;
	Serial.println(F("IIC snoop..."));
	while(activity_snooped<to_snoop){
		tretval=SoftIIC::get_byte(&value);	
		switch(tretval){
			case RETVAL_SUCCESS					:
			SoftIIC::wait_for_bus_activity(); 
			SoftIIC::fastprinthexbyte(value);
			activity_snooped++;		
			if(SoftIIC::StateStart())	Serial.print("\n"); 	
			if(SoftIIC::StateStop())	Serial.print("\\"); 	
			if(SoftIIC::StateData())	{
				if(SoftIIC::StateDataBit())	Serial.print("+");
				else						Serial.print("-");
			}
			if(SoftIIC::TimerElapsed()) 	Serial.print("*");
			break;
			case RETVAL_UNEXPECTED_START		:	Serial.print("\n"); 		break;
			case RETVAL_UNEXPECTED_STOP			:	Serial.print("\\"); 		break;
			case RETVAL_PROTOCOL_FAILURE		:	Serial.print("?"); 			break;
			case RETVAL_TIMEOUT					:	Serial.print("*"); 			break;
			default								:	;
		}	
	}
	Serial.print(F("\nIIC snooping done\n")); Serial.flush();
	SoftIIC::RestoreTimer1Settings();
return activity_snooped;    
}
  


  void SoftIIC::test_input_response_time(){  	
	SoftIIC::ConfigureTimer1Settings();
	SoftIIC::TimerReset();
	SoftIIC::TransferTimerConfigureByte();
	SoftIIC::TimerClearMatch(); 
	Serial.println(F("RTtest"));
	uint8_t tretval;
	uint8_t breakloop=0;
	while(breakloop==0){
		wait_for_bus_activity();
		debug_pin_A_toggle();
		if(SoftIIC::StateStop()){breakloop=1;}
	}
	SoftIIC::RestoreTimer1Settings();
  }
    
  
	
uint16_t SoftIIC::SlaveHandleTransaction(
	uint8_t (*fp_respond_to_address)(uint8_t chip_address),
	uint8_t (*fp_respond_to_command)(uint8_t chip_address),
	uint8_t (*fp_respond_to_data)(uint8_t chip_address),
	uint8_t (*fp_get_register_address)(uint8_t chip_address ),
	uint8_t (*fp_set_register_address)(uint8_t chip_address, uint8_t reg_address ),
	uint8_t (*fp_generate_byte)(uint8_t chip_address, uint8_t* byte),
	uint8_t (*fp_receive_byte)(uint8_t chip_address, uint8_t byte)
){
//	SoftIIC::debug_pin_B_high();			
	SoftIIC::ConfigureTimer1Settings();
	SoftIIC::TimerReset();
	SoftIIC::TransferTimerConfigureTransfer();
	SoftIIC::TimerClearMatch(); 
	uint8_t tretval;
	uint16_t number_of_successful_bytes=0; 
//	wdt_disable();
	
beginning:  // We wait for an address and decide whether or not to ack it.
//	wdt_reset();			
	uint8_t chip_address=0x00;	
	uint8_t rwbit=0x00;
	uint8_t value=0x00;
	uint8_t chipddr_has_been_set=0;
	uint8_t regaddr_has_been_set=0;	
	uint8_t (*my_ack_function)(uint8_t)=fp_respond_to_address;	
	SoftIIC::bus_read();		
	SoftIIC::bus_read();	
	
waiting_for_start:
	SoftIIC::TransferTimerConfigureTransfer();
	tretval=SoftIIC::spin_until_start();
if(tretval==RETVAL_TIMEOUT){goto done_with_iic_transaction;}
if(tretval!=RETVAL_SUCCESS){goto waiting_for_start;}
	SoftIIC::TimerReset();
	SoftIIC::TransferTimerConfigureByte();
	
getting_a_byte:
	tretval=SoftIIC::get_byte(&value, my_ack_function);
	switch (tretval) {
		case RETVAL_SUCCESS 			: 	
			if(chipddr_has_been_set==0){
				chip_address=value>>1;
				rwbit=value & IIC_RWMASK; 
				chipddr_has_been_set=1;
				my_ack_function=fp_respond_to_command;
				if(rwbit==IIC_READ){	goto sending_a_byte;	}
				else {					goto getting_a_byte;	}	
			}
			else if(regaddr_has_been_set==0){
				(*fp_set_register_address)(chip_address, value);
				my_ack_function=fp_respond_to_data;
				regaddr_has_been_set=1;
				if(rwbit==IIC_READ){	goto sending_a_byte;	}
				else {					goto getting_a_byte;	}	
				
			} 
			else {
				// Handle the incoming byte.  Note: uses the built-in register address.
				(*fp_receive_byte)(chip_address, value);
				// Increment the built-in register address.
				(*fp_set_register_address)(chip_address, (*fp_get_register_address)(chip_address)+1);	
				number_of_successful_bytes++;	
				goto getting_a_byte; 
			}
		break;	
		case RETVAL_UNEXPECTED_START	:   goto slave_unexpected_start; 		break;
		case RETVAL_UNEXPECTED_STOP		: 	goto slave_unexpected_stop; 		break;
		case RETVAL_TIMEOUT				: 	goto done_with_iic_transaction; 	break;
		default 						:	goto done_with_iic_transaction; 	break;
	}
	
sending_a_byte:		// Generate a new byte to send.  Note: uses the built-in register address.
	if(((*fp_generate_byte)(chip_address, &value))>0)  {goto 	done_with_iic_transaction;}	
//	Serial.write('c'); SoftIIC::fastprinthexbyte(chip_address); Serial.write('t'); SoftIIC::fastprinthexbyte(value); Serial.println("");
	tretval=SoftIIC::set_byte(value);
	// Increment the built-in register address.	
	(*fp_set_register_address)(chip_address, (*fp_get_register_address)(chip_address)+1);	
	// Look for an ACK to continue transmitting.  NACK terminates the transaction.			
	switch (tretval) {
		case RETVAL_SUCCESS 			: 	number_of_successful_bytes++; goto sending_a_byte; 	break;
		case RETVAL_UNEXPECTED_START	: 	goto slave_unexpected_start; 						break;
		case RETVAL_UNEXPECTED_STOP		: 	goto slave_unexpected_stop; 						break;
		case RETVAL_TIMEOUT				: 	goto done_with_iic_transaction; 					break;
		default 						:	goto done_with_iic_transaction;		 				break;
	}
		
slave_unexpected_start:
	chipddr_has_been_set=0;
	regaddr_has_been_set=0;
	my_ack_function=fp_respond_to_address;
	SoftIIC::TimerReset();
	SoftIIC::TransferTimerConfigureByte();
	goto getting_a_byte;
	
slave_unexpected_stop:
	goto slave_unexpected_start;
	
	
done_with_iic_transaction:	// always exit with a clean interface
	SoftIIC::clock_release();
	SoftIIC::data_release();
//	SoftIIC::debug_pin_B_low();
	SoftIIC::RestoreTimer1Settings();
//	wdt_disable();
	return number_of_successful_bytes;
}

	
	
// As a reminder, all SoftIICSlave functions return the following codes unless otherwise specified:
// RETVAL_SUCCESS 				// The function has exited successfully.
// RETVAL_UNEXPECTED_START	    // During operation, the bus encountered an unexpected start condition.
// RETVAL_UNEXPECTED_STOP	    // During operation, the bus encountered an unexpected stop condition.
// RETVAL_TIMEOUT			    // The bus timed out within the function.




static uint8_t AlwaysAck(uint8_t ignored){ return 1;}

uint8_t SoftIIC::get_byte(uint8_t* value){
return SoftIIC::get_byte(value, AlwaysAck);
}


uint8_t SoftIIC::get_byte(uint8_t* value, uint8_t (*my_ack_function)(uint8_t rxbyte)){ 
	SoftIIC::TimerReset();
	SoftIIC::TransferTimerConfigureByte();
//			SoftIIC::debug_pin_A_high();		SoftIIC::debug_pin_A_low();	
	uint8_t my_bit=7;	
	
	if(SoftIIC::spin_until_clock_falls()) {goto get_byte_failure;}
	data_release();
	IIC_STATE=0x00;
	getting_a_bit:	
		SoftIIC::bus_read();
		if(SoftIIC::StateData())					{
//			SoftIIC::debug_pin_B_high();		SoftIIC::debug_pin_B_low();	
			*value|=((SoftIIC::StateDataBit())>0);
			if(my_bit==0x00) {	goto getbyte_handle_ack;	}
			*value=*value<<1;				
			my_bit--;
			goto getting_a_bit; // If successful, skip the safety checks and get the next bit.
		}
		
		if(SoftIIC::StateStart())  		{goto get_byte_unexpected_start;}
		if(SoftIIC::StateStop()) 		{goto get_byte_unexpected_stop;}
		if(SoftIIC::TimerElapsed()) 	{goto get_byte_failure;}
		
	goto getting_a_bit;
	
	
	
	//	Because we can be expected to ACK/NACK a received byte immediately after receiving it, the last bit must be done very quickly.
// There is not enough time to handle start/stop/timeout detection+handling.
// This optimization greatly reduces rising-edge to ack-function delay which is absolutely critical for proper operation at high speeds.
// Note that not much safety is given up here, spin_until_clock_rises can internally time-out.  It will have to basically time out twice or deal with an invalid ACK routine.
get_last_bit:
	if(SoftIIC::spin_until_clock_falls()) {goto get_byte_failure;}
		if(SoftIIC::StateStart())  		{goto get_byte_unexpected_start;}
		if(SoftIIC::StateStop()) 		{goto get_byte_unexpected_stop;}
	SoftIIC::spin_until_clock_rises();
//	SoftIIC::debug_pin_B_high();		SoftIIC::debug_pin_B_low();	
	*value|=((SoftIIC::StateDataBit())>0);
	goto getbyte_handle_ack;
	
//	getting_a_bit:		
//		if(SoftIIC::spin_until_clock_falls()) {goto get_byte_failure;}
//		*value=*value<<1;	
//		if(SoftIIC::spin_until_clock_rises()) {goto get_byte_failure;}	
//		*value|=((SoftIIC::StateDataBit())>0);	
//		//SoftIIC::debug_pin_B_high();		SoftIIC::debug_pin_B_low();	
//		if(my_bit==0) {	goto getbyte_handle_ack;	}	
//		my_bit--;	
//	goto getting_a_bit;
	
	
	getbyte_handle_ack:	
	SoftIIC::TransferTimerConfigureByteWithAck();
//	SoftIIC::debug_pin_B_high();		SoftIIC::debug_pin_B_low();	
	
	// If I do not want to ACK the received byte (eg: it is for a different chip or an unsupported command), exit as though a stop condition occured.
		if(my_ack_function(*value)==0){
		if(SoftIIC::spin_until_clock_falls()) {goto get_byte_failure;}	
		data_release();	
		goto get_byte_unexpected_stop;
		}		
		
//	SoftIIC::debug_pin_A_high();	SoftIIC::debug_pin_A_low();
	if(SoftIIC::spin_until_clock_falls()) {goto get_byte_failure;}	
	data_pull_down();		
//	SoftIIC::debug_pin_A_high();	SoftIIC::debug_pin_A_low();
	if(SoftIIC::spin_until_clock_rises()) {goto get_byte_failure;}
//	SoftIIC::debug_pin_A_high();	SoftIIC::debug_pin_A_low();		
//	IIC_STATE=_busBitMask;	
	
	/////////////// At the cost of having to deal with holding the data line low after this function ends (have set_byte and get_byte each spin until low and release), we can exit much earlier.
//	if(SoftIIC::spin_until_clock_falls()) {goto get_byte_failure;}
//	data_release(); 	
	
//	SoftIIC::debug_pin_A_high();	SoftIIC::debug_pin_A_low();
	
	
	
	return RETVAL_SUCCESS;
	
	
	
	get_byte_failure:
//	SoftIIC::debug_pin_A_low();		
//	SoftIIC::debug_pin_A_high();
//	SoftIIC::debug_pin_A_low();	
	return RETVAL_TIMEOUT;
	
	
	get_byte_unexpected_start:
//	SoftIIC::debug_pin_A_low();		
//	SoftIIC::debug_pin_A_high();
//	SoftIIC::debug_pin_A_low();	
	return RETVAL_UNEXPECTED_START;
	
	get_byte_unexpected_stop:
//	SoftIIC::debug_pin_A_low();		
//	SoftIIC::debug_pin_A_high();
//	SoftIIC::debug_pin_A_low();	
	return RETVAL_UNEXPECTED_STOP;
}
	
	
uint8_t SoftIIC::set_byte(uint8_t value){ 
	SoftIIC::TimerReset();
	SoftIIC::TransferTimerConfigureByte(); 
	uint8_t my_bit=0b10000000;	
		
	setting_a_bit:	
		SoftIIC::bus_read();
		if(SoftIIC::StateClockFell())					{
			if(value &  my_bit){		data_release();	}	else{		data_pull_down();		}
			SoftIIC::debug_pin_A_high();		SoftIIC::debug_pin_A_low();	
			my_bit=my_bit>>1;
			if(my_bit==0x00) {	goto getting_an_ack;	}
			goto setting_a_bit; // If successful, skip the safety checks and get the next bit.
		}
		// These checks unfortunately have to be bypassed for performance reasons.
//		if(SoftIIC::StateStart())  		{goto set_byte_failure;}
//		if(SoftIIC::StateStop()) 		{goto set_byte_failure;}
		if(SoftIIC::TimerElapsed()) 	{goto set_byte_failure;}
	goto setting_a_bit;
	
	
	
	
	
		
getting_an_ack:
	SoftIIC::TransferTimerConfigureByteWithAck(); 
	if(SoftIIC::spin_until_clock_rises()) {goto set_byte_failure;}
//	SoftIIC::debug_pin_B_high();	SoftIIC::debug_pin_B_low();	
	if(SoftIIC::spin_until_clock_falls()) {goto set_byte_failure;}
	data_release();		
	if(SoftIIC::spin_until_clock_rises()) {goto set_byte_failure;}
	SoftIIC::debug_pin_B_high();	SoftIIC::debug_pin_B_low();		
	
		// Note that we do not have to do another bus_read here because spin_until_clock_rises saves IIC_STATE.
	return (SoftIIC::StateDataBit()); // The return value is 0x00 (RETVAL_SUCCESS) in the case of an ACK and if NACK, IIC_STATE_MASK_SDA_CURRENT.
		
	set_byte_failure:
//	SoftIIC::debug_pin_A_high();	SoftIIC::debug_pin_A_low();	
		SoftIIC::spin_until_clock_falls();
		data_release();
		return 0xFF;
		
	}
	

 uint8_t SoftIIC::set_next_bus_activity(uint8_t value){
	 uint8_t tretval;
	tretval=SoftIIC::spin_until_clock_falls(); 		if(tretval!=RETVAL_SUCCESS) {return tretval;}	
	if(value==0x00){
		data_pull_down();	
	}	else{
		data_release();	
	}
	tretval=SoftIIC::spin_until_clock_rises();		if(tretval!=RETVAL_SUCCESS) {return tretval;}
	return RETVAL_SUCCESS;
}
  





// Note: The 'spin until x' functions are different/separate because they do the bus_read at the end of the loop to allow for an immediate exit under high speed operation.
 uint8_t SoftIIC::spin_until_start(){
	 uint8_t number_overflows=255;
//		SoftIIC::debug_pin_B_high();
	while(number_overflows>0){
		SoftIIC::bus_read();		
		if(SoftIIC::StateStart()) {	return RETVAL_SUCCESS; }
		if(SoftIIC::TimerElapsed()) {number_overflows--;}		
	}	
//		SoftIIC::debug_pin_B_low();
	return IIC_TIMEOUT;
}

 uint8_t SoftIIC::spin_until_clock_rises(){
//while(1){
//	if((SoftIIC::clock_read())){ return  RETVAL_SUCCESS;	}	 
//	if(SoftIIC::TimerElapsed()) {return IIC_TIMEOUT;}
//}
	// It may seem that this function could be optimized as the data line is ignorable while the clock is low, but having IIC_STATE be correct avoids a second read afterwards.
	while(1){
		SoftIIC::bus_read();		if(SoftIIC::StateClockHigh()){return RETVAL_SUCCESS; }
		SoftIIC::bus_read();		if(SoftIIC::StateClockHigh()){return RETVAL_SUCCESS; }
//		SoftIIC::bus_read();		if(SoftIIC::StateClockHigh()){return RETVAL_SUCCESS; }
//		SoftIIC::bus_read();		if(SoftIIC::StateClockHigh()){return RETVAL_SUCCESS; }
		if(SoftIIC::TimerElapsed()) {return IIC_TIMEOUT;}	
	}
}

 uint8_t SoftIIC::spin_until_clock_falls(){
	 // Waiting for clock to fall is a different story, since we don't care about the data state once clock is low.
	 
spin_again_for_clock_fall:
	if(SoftIIC::clock_read()==0) {return RETVAL_SUCCESS;}
	if(SoftIIC::clock_read()==0) {return RETVAL_SUCCESS;}
//	if(SoftIIC::clock_read()==0) {return RETVAL_SUCCESS;}
//	if(SoftIIC::clock_read()==0) {return RETVAL_SUCCESS;}
	if(SoftIIC::TimerElapsed()) {return IIC_TIMEOUT;}
	goto spin_again_for_clock_fall;
//	while(1){
//		SoftIIC::bus_read();
//		if(SoftIIC::StateClockLow()) {return  RETVAL_SUCCESS;}
//		if(SoftIIC::StateStart()) {return  RETVAL_UNEXPECTED_START;}
//		if(SoftIIC::StateStop()) {return  RETVAL_UNEXPECTED_STOP;}
//		if(SoftIIC::TimerElapsed()) {return IIC_TIMEOUT;}	
//	}
}

// Only used for debugging.  
 void SoftIIC::wait_for_bus_activity(){
	while(1){
	//	SoftIIC::functimer_start();
		SoftIIC::bus_read();
	//	SoftIIC::functimer_stop();
	if(SoftIIC::StateStart()) 					{ return; }
	if(SoftIIC::StateData()) 					{ return; }
	if(SoftIIC::StateStop()) 					{ return; }
	if(SoftIIC::TimerElapsed()) { return; }	
	}
}
  
  void SoftIIC::clock_pull_down(){  
		*_sclPortReg  &= _sclBitMaskINVERTED;  
		*_sclDirReg   |=  _sclBitMask; 
	}
	
  void SoftIIC::clock_release(){
		*_sclDirReg   &= _sclBitMaskINVERTED;  		
		#ifndef SOFTIIC_OPTIMIZE_NOPULLUPS
			if(usePullups) { *_sclPortReg  |=  _sclBitMask; }
		#endif
	}
 	
     void SoftIIC::data_pull_down(){  
		*_sdaPortReg  &= _sdaBitMaskINVERTED;  
		*_sdaDirReg   |=  _sdaBitMask; 
	}
	
     void SoftIIC::data_release(){
		*_sdaDirReg   &= _sdaBitMaskINVERTED;  		
		#ifndef SOFTIIC_OPTIMIZE_NOPULLUPS
			if(usePullups) { *_sdaPortReg  |=  _sdaBitMask; }
		#endif
	}
	
	 // For clock_read, data_read, and bus_read, should I look at dirReg?
	 uint8_t SoftIIC::clock_read(){
		return ((*_sclPortRegIN) & (_sclBitMask)); 
	}

	 uint8_t SoftIIC::data_read(){
		return ((*_sdaPortRegIN) & (_sdaBitMask)); 
	 }
	
	 
 
 void SoftIIC::bus_read(){		
	 // Could re-add support for different-port configurations, but this will always be problematic.  May be best to not allow and ignore.
//bus_read:
 SoftIIC::bus_read_same_port();	 
// if((IIC_STATE_PREVIOUS^IIC_STATE)) {return;} else {goto bus_read;}  // This didn't work well, but the idea isn't bad.
}

 
 void SoftIIC::bus_read_same_port(){
	IIC_STATE_PREVIOUS=IIC_STATE;
	IIC_STATE=*_sclPortRegIN;
	IIC_STATE&=_busBitMask;
};


 void SoftIIC::bus_read_different_port(){		
	uint8_t sampled_io_scl=*_sclPortRegIN;
	uint8_t sampled_io_sda=*_sdaPortRegIN;	
	sampled_io_sda=((sampled_io_sda & _sdaBitMask)>0)<<IIC_STATE_MASK_SDA_SHIFT;
	sampled_io_scl=((sampled_io_scl & _sclBitMask)>0)<<IIC_STATE_MASK_SCL_SHIFT;	
	IIC_STATE=((IIC_STATE&IIC_STATE_MASK_CURRENT)<<1);
	IIC_STATE|=sampled_io_scl|sampled_io_sda;
}


  uint8_t SoftIIC::StateIdle(){
	 return((IIC_STATE_PREVIOUS == _busBitMask) & (IIC_STATE == _busBitMask)); 	 
 }
  
 uint8_t SoftIIC::StateStart(){
	 return((IIC_STATE_PREVIOUS == _busBitMask) & (IIC_STATE == _sclBitMask)); 	 
 } 
 
 uint8_t SoftIIC::StateStop(){
	 return((IIC_STATE_PREVIOUS == _sclBitMask) & (IIC_STATE == _busBitMask)); 	 
 }
 
 uint8_t SoftIIC::StateData(){
 	 return(((~IIC_STATE_PREVIOUS) & IIC_STATE) & _sclBitMask); 	
 }
 
  uint8_t SoftIIC::StateClockFell(){
 	 return(((~IIC_STATE) & IIC_STATE_PREVIOUS) & _sclBitMask); 	 	 
 }
 
 
 
 uint8_t SoftIIC::StateDataBit(){
	 return(IIC_STATE & (_sdaBitMask)); 	 
 }
 
 uint8_t SoftIIC::StateClockLow(){
	 return(~(IIC_STATE & _sclBitMask)); 	 
 }
 
 uint8_t SoftIIC::StateClockHigh(){
	 return((IIC_STATE & _sclBitMask)); 		 
 }










	
	/////////////////////////////////////////		SUPPORTING FUNCTIONS
	
	SoftIIC::SoftIIC(uint8_t pin_scl, uint8_t pin_sda, uint16_t speed, bool pullups, bool multimastersupport, bool timeout)
		:PIN_SCL				(pin_scl)						        
		,PIN_SDA				(pin_sda)						        
		,_sclBitMask			(digitalPinToBitMask(pin_scl))						
		,_sclBitMaskINVERTED	(~(digitalPinToBitMask(pin_scl)))	
		,_sclPortRegIN			(portInputRegister(digitalPinToPort(pin_scl)))	
		,_sdaBitMask			(digitalPinToBitMask(pin_sda))						
		,_sdaBitMaskINVERTED	(~(digitalPinToBitMask(pin_sda)))	
		,_sdaPortRegIN 			(portInputRegister(digitalPinToPort(pin_sda)))		
		,_busBitMask			(_sclBitMask|_sdaBitMask)					
		,usePullups				(pullups)		                        
		,MULTIMASTERSUPPORT		(multimastersupport)                    
		,TimeoutsEnabled		(timeout)
		,IIC_CLOCK_SPEED		(speed)
	{
	if(pullups==true){
		pinMode(pin_scl, INPUT_PULLUP);
		pinMode(pin_scl, INPUT_PULLUP);
	}else{
		pinMode(pin_scl, INPUT);
		pinMode(pin_scl, INPUT);
	}

	_sclPortReg  	=	portOutputRegister(digitalPinToPort(pin_scl));
	_sclDirReg   	=	portModeRegister(digitalPinToPort(pin_scl));
	_sdaPortReg  	=	portOutputRegister(digitalPinToPort(pin_sda));
	_sdaDirReg		=	portModeRegister(digitalPinToPort(pin_sda));		
			
		// Just don't deal with different ports, results WILL fail sometimes due to incorrect start/stop conditions.
	if(_sclPortReg != _sdaPortReg){Serial.println(F("Warning: IIC port mismatch!")); while(1);}
		
		noInterrupts();		
		
		my_TCCR1B=0x00;
		HalfclockTimeout=10000;
		while(	HalfclockTimeout>1000)	{
			my_TCCR1B++;
			uint16_t timer1_tick_rate=SoftIIC::GetFrequencyOfTimer1Divider(my_TCCR1B);
			HalfclockTimeout=((timer1_tick_rate)/(2*IIC_CLOCK_SPEED));	
		}
				
		TransferTimeout=2*HalfclockTimeout*TIMEOUT_CLOCKS_TRANSACTION-TIMER_OVERHEAD_COMPENASTION;
		ByteTimeout=2*HalfclockTimeout*TIMEOUT_CLOCKS_BYTE-TIMER_OVERHEAD_COMPENASTION;
		ByteAckTimeout=ByteTimeout+2*HalfclockTimeout*TIMEOUT_CLOCKS_SENDACK;
		HalfclockTimeout=HalfclockTimeout-TIMER_OVERHEAD_COMPENASTION;
				
		
		SoftIIC::clock_release();
		SoftIIC::data_release();
		
		SoftIIC::InitDebugpins();
		SoftIIC::debug_pin_test();
		
		my_TCCR1A= 0; // turn off PWM	
		my_TCCR1B |= (1<<WGM12);  // Enables CTC mode with TOP set to OCR1A
		my_OCR1A = 0;
		my_OCR1B = 0; 
		my_TIMSK1 = 0 ; 
		my_TIFR1 = 0 ; 			
		
		interrupts();
	}
			
	
	
	
    SoftIIC::~SoftIIC(){
		// Restore pin settings
		pinMode(PIN_SCL, INPUT);
		pinMode(PIN_SDA, INPUT);
	}
	
	
	
	void SoftIIC::PrintSpeed(){
	#ifdef SOFTIIC_DEGBUG_MODE
		SoftIIC::ConfigureTimer1Settings();
		uint16_t timer1_tick_rate=SoftIIC::GetFrequencyOfTimer1Divider(SoftIIC::GetCurrentTimer1Divider());
		Serial.print(F("CPU speed (in KHz):"));  Serial.println(F_CPU/1000);  Serial.flush();
		Serial.print(F("IIC speed (in KHz):"));  Serial.println(IIC_CLOCK_SPEED);  Serial.flush();
		Serial.print(F("Timer compensation:"));  Serial.println(TIMER_OVERHEAD_COMPENASTION);
		Serial.print(F("Timer1 divider:"));  Serial.println(SoftIIC::GetCurrentTimer1Divider());
		Serial.print(F("Timer1 tick rate (in KHz):"));  Serial.println(timer1_tick_rate);
		Serial.print(F("IIC halfclock period in timer1 ticks:"));  Serial.println(HalfclockTimeout); Serial.flush();
		Serial.print(F("IIC transaction timeout in IIC clocks:"));  Serial.println(TIMEOUT_CLOCKS_TRANSACTION); Serial.flush();		
		Serial.print(F("IIC byte timeout in IIC clocks:"));  Serial.println(TIMEOUT_CLOCKS_BYTE); Serial.flush();	
		Serial.print(F("Timeouts are "));  if(SoftIIC::are_timeouts_enabled()) {Serial.println(F("enabled."));} else {Serial.println(F("disabled."));}

		SoftIIC::RestoreTimer1Settings();
//		Serial.print(F("clk/2:"));  Serial.println(HalfclockTimeout); Serial.flush();	
#endif			
	}
	
	uint8_t	SoftIIC::are_timeouts_enabled(){		return TimeoutsEnabled;		}
	
  void SoftIIC::TimerReset(){
  TCNT1=0;
  }  
  void SoftIIC::TimerClearMatch(){
  TIFR1 |= (1 << OCF1A);
  }  
  
  // Note: you should almost always do a timer reset before reconfiguring.
	void SoftIIC::TransferTimerConfigureTransfer(){ OCR1A=TransferTimeout;}
	void SoftIIC::TransferTimerConfigureByte(){ OCR1A=ByteTimeout;}
	void SoftIIC::TransferTimerConfigureByteWithAck(){  OCR1A=ByteAckTimeout;}
	void SoftIIC::TransferTimerConfigureHalfclock(){ OCR1A=HalfclockTimeout;}
	uint16_t SoftIIC::get_timer(){ return TCNT1;  }	
	
	
	
	uint8_t SoftIIC::TimerElapsed(){if(TIFR1 & (1 << OCF1A)){TIFR1 |= (1 << OCF1A); return 1;} return 0;}
	// Alternate method (larger size, probably slower)
//	uint8_t SoftIIC::TimerElapsed(){
//	uint8_t my_TIFR1=TIFR1;
//	TIFR1 |= (1 << OCF1A);
//	return (my_TIFR1 & (1 << OCF1A));	
//	}
	
 
uint8_t  SoftIIC::GetCurrentTimer1Divider(){return TCCR1B&0b00000111;}
uint8_t  SoftIIC::GetCurrentTimer2Divider(){return TCCR2B&0b00000111;}

uint16_t SoftIIC::GetFrequencyOfTimer1Divider(uint8_t divider){ // returns ticks per millisecond
	if(SoftIIC::GetRatioOfTimer1Divider(divider)==0) {return 0;}
	return ((F_CPU/1000)/SoftIIC::GetRatioOfTimer1Divider(divider));
}

uint16_t SoftIIC::GetFrequencyOfTimer2Divider(uint8_t divider){ // returns ticks per millisecond
	if(SoftIIC::GetRatioOfTimer2Divider(divider)==0) {return 0;}
	return ((F_CPU/1000)/SoftIIC::GetRatioOfTimer2Divider(divider));
}


uint16_t SoftIIC::GetRatioOfTimer1Divider(uint8_t divider){ // returns clocks per tick
  switch (divider) {
  case 0x01:                return (    1 );     break;
  case 0x02:                return (    8 );     break;
  case 0x03:                return (   64 );     break;
  case 0x04:                return (  256 );     break;
  case 0x05:                return ( 1024 );     break;
  default:                  return (    0 );     
  }
}

uint16_t SoftIIC::GetRatioOfTimer2Divider(uint8_t divider){ // returns clocks per tick
  switch (divider) {
  case 0x01:                return (   1 );     break;
  case 0x02:                return (   8 );     break;
  case 0x03:                return (  32 );     break;
  case 0x04:                return (  64 );     break;
  case 0x05:                return (  128 );     break;
  case 0x06:                return (  256 );     break;
  case 0x07:                return ( 1024 );     break;
  default:                  return (    0 );     
  }
}

	void  SoftIIC::RestoreTimer1Settings(){
		TCCR1A  = 0x00;
		OCR1A	= p_OCR1A;
		OCR1B	= p_OCR1B;
		TIMSK1	= p_TIMSK1;   
		TIFR1	= p_TIFR1;  			
		TCCR1A	= p_TCCR1A;
		TCCR1B	= p_TCCR1B;
		interrupts();
	}
	void  SoftIIC::SaveTimer1Settings(){
		p_OCR1A		= OCR1A;
		p_OCR1B		= OCR1B;
		p_TIMSK1	= TIMSK1;  	 
		p_TIFR1		= TIFR1;  	 	
		p_TCCR1A	= TCCR1A;
		p_TCCR1B	= TCCR1B;		
	}
	void  SoftIIC::ConfigureTimer1Settings(){
		noInterrupts();
		SoftIIC::SaveTimer1Settings();
		TCCR1A  = 0x00;
		OCR1A 	= my_OCR1A 	;
		OCR1B 	= my_OCR1B 	;
		TIMSK1 	= my_TIMSK1 ;
		TIFR1 	= my_TIFR1 	;
		TCCR1A	= my_TCCR1A	;
	if(SoftIIC::are_timeouts_enabled()) {		TCCR1B 	= my_TCCR1B ;}
	else {TCCR1B=0x00;}
	}


//////////////////////////// DEBUGGING STUFF IS BELOW THIS LINE

 void SoftIIC::InitDebugpins(){			
#ifdef DEBUGPIN_A
		pinMode2(DEBUGPIN_A, OUTPUT);
		SoftIIC::debug_pin_A_low();
#endif
#ifdef DEBUGPIN_B
		pinMode2(DEBUGPIN_B, OUTPUT);
		SoftIIC::debug_pin_B_low();
#endif
 }
	
 void SoftIIC::debug_pin_test(){
	 SoftIIC::debug_pin_A_test();
	 SoftIIC::debug_pin_B_test();	 
 }
 
 
 void SoftIIC::debug_pin_A_test(){
#ifdef DEBUGPIN_A
//	 Serial.println(F("Testing debug IO A"));
 for (uint8_t i=8; i>0; i--){	 SoftIIC::debug_pin_A_toggle();	 delayMicroseconds(100); }
#endif
}
	 void SoftIIC::debug_pin_B_test(){
#ifdef DEBUGPIN_B
//	 Serial.println(F("Testing debug IO B"));
 for (uint8_t i=8; i>0; i--){	 SoftIIC::debug_pin_B_toggle();	 delayMicroseconds(100); }
#endif
}
	
 void SoftIIC::debug_pin_A_high(){
#ifdef DEBUGPIN_A
	DEBUG_PIN_A_STATE=1;	digitalWrite2(DEBUGPIN_A, HIGH);
#endif
}

 void SoftIIC::debug_pin_A_low(){
#ifdef DEBUGPIN_A
	DEBUG_PIN_A_STATE=0;	digitalWrite2(DEBUGPIN_A, LOW );
#endif
}

 void SoftIIC::debug_pin_A_toggle(){
#ifdef DEBUGPIN_A
	if(DEBUG_PIN_A_STATE!=0) 	{ SoftIIC::debug_pin_A_low();}
	else 						{ SoftIIC::debug_pin_A_high();}
#endif
}

 void SoftIIC::debug_pin_B_high(){
#ifdef DEBUGPIN_B
	DEBUG_PIN_B_STATE=1;	digitalWrite2(DEBUGPIN_B, HIGH);
#endif
}

 void SoftIIC::debug_pin_B_low(){
#ifdef DEBUGPIN_B
	DEBUG_PIN_B_STATE=0;	digitalWrite2(DEBUGPIN_B, LOW );
#endif
}

 void SoftIIC::debug_pin_B_toggle(){
#ifdef DEBUGPIN_B
	if(DEBUG_PIN_B_STATE!=0) 	{ SoftIIC::debug_pin_B_low();}
	else 						{ SoftIIC::debug_pin_B_high();}	
#endif
}
	
	// NOTE: this debugging feature uses timer2.
	  void SoftIIC::functimer_start(){
#ifdef SOFTIIC_DEGBUG_MODE
  		TCCR2A= 0; // turn off PWM
		TCCR2B = 0x00; // set clock scaler to 0
		OCR2A = 0;
		OCR2B = 0; 
		TIMSK2 = 0 ; 
		TCNT2=0;
//		TCCR2B = 0x03; // set clock scaler to 8
		TCCR2B = 0x01; // set clock scaler to 1
#endif
}
    void SoftIIC::functimer_stop(){
#ifdef SOFTIIC_DEGBUG_MODE
		uint16_t mytimer=TCNT2;
		mytimer=(mytimer*SoftIIC::GetRatioOfTimer2Divider(SoftIIC::GetCurrentTimer2Divider()))-1;
		Serial.print(F("K:\t")); Serial.println(mytimer,DEC); Serial.flush();
#endif
	}
