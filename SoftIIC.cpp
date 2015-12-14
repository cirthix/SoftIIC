// SEE SOFTIIC.H FOR INFO
// VERSION 0.1
#include <SoftIIC.h>
  
  
  // IIC STATE MACHINE
#define TRANSACTION_STATE_GETTING_START				1
#define TRANSACTION_STATE_GETTING_CHIPADDRESS		2
#define TRANSACTION_STATE_SENDING_ACK				3 // NEVER USED, LUMPED IN WITH GETTING_BYTE
#define TRANSACTION_STATE_SENDING_NACK				4 // NEVER USED, LUMPED IN WITH GETTING_BYTE
#define TRANSACTION_STATE_GETTING_BYTE				5
#define TRANSACTION_STATE_SENDING_BYTE				6
#define TRANSACTION_STATE_WAITING_FOR_ACK			7
 
  
#define IIC_STATE_MASK_SDA_CURRENT  0b00000001
#define IIC_STATE_MASK_SDA_PREVIOUS 0b00000010
#define IIC_STATE_MASK_SDA			0b00000011
#define IIC_STATE_MASK_SCL_CURRENT  0b00000100
#define IIC_STATE_MASK_SCL_PREVIOUS 0b00001000
#define IIC_STATE_MASK_SCL			0b00001100
#define IIC_STATE_MASK_CURRENT      0b00000101
#define IIC_STATE_MASK_PREVIOUS     0b00001010

#define IIC_ZERO		0x00		// Rising edge of clock with data = 0
#define IIC_ONE			0x01		// Rising edge of clock with data = 1
#define IIC_CLOCKFALL	0x02		// Falling edge of clock
#define IIC_START		0x03		// Data going low while clock remains high
#define IIC_STOP		0x04		// Data going high while clock remains high
#define IIC_TIMEOUT		0xFF		// ANY STATIC CONDITION THAT EXCEEDS THE TIMEOUT TIME
			
#define	IIC_ACK			0xf0
#define	IIC_NACK		0xf1
			
// ENUMERATED FUNCTION RETURN VALUE POSSIBILITES
#define RETVAL_SUCCESS 					0x00	// The function has exited successfully.
#define RETVAL_UNEXPECTED_START			0x01	// During operation, the bus encountered an unexpected start condition.
#define RETVAL_UNEXPECTED_STOP			0x02	// During operation, the bus encountered an unexpected stop condition.
#define RETVAL_PROTOCOL_FAILURE			0x03	// Probably the communication was NAK'd
#define RETVAL_TIMEOUT					0xFF	// The bus timed out within the function.

const		uint8_t 	I2C_RWMASK 	= 0x01; 	//Byte to AND with address for read/write bit 
const		uint8_t 	I2C_READ 	= 0x01; 	//Bit to or with address for read start and read restart 
const		uint8_t 	I2C_WRITE 	= 0x00; 	//Bit to or with address for write start and write restart 

#define HALF_CLOCK_TIMER_OVERHEAD_COMPENASTION	24  // Tune this with a scope for your chip/compiler such that a setting of '100' yields 100KHz rate.


////////////// Public Methods ////////////////////////////////////////



	
uint8_t SoftI2C::MasterCheckExists(uint8_t device_address){
	SoftI2C::MasterBusRestart();  delayMicroseconds(10000);
	uint8_t retval;		
//	SoftI2C::functimer_start();
		retval=SoftI2C::MasterStart(device_address);
		SoftI2C::MasterStop();
//	SoftI2C::functimer_stop();
	return retval;
	}


uint8_t SoftI2C::MasterReadByte(uint8_t device_address, uint8_t register_address ){
	uint8_t retval=0xFF;
	SoftI2C::MasterReadByte(device_address, register_address, &retval);
	return retval;
}
	
uint8_t SoftI2C::MasterReadByte(uint8_t device_address, uint8_t register_address , uint8_t* value ){
	SoftI2C::MasterBusRestart();  delayMicroseconds(10000);
	uint8_t tretval=RETVAL_SUCCESS;
  tretval=SoftI2C::MasterStart((device_address<<1)| I2C_WRITE);  		if (tretval != RETVAL_SUCCESS) {  SoftI2C::MasterStop(); return tretval;}
  tretval=SoftI2C::MasterWrite(register_address);					  	if (tretval != RETVAL_SUCCESS) {  SoftI2C::MasterStop(); return tretval;}
  tretval=SoftI2C::MasterRestart((device_address<<1)| I2C_READ);  		if (tretval != RETVAL_SUCCESS) {  SoftI2C::MasterStop(); return tretval;}
  *value = SoftI2C::MasterRead(true);    
  SoftI2C::MasterStop();  
  return RETVAL_SUCCESS;
}

uint8_t SoftI2C::MasterWriteByte(uint8_t device_address, uint8_t register_address, uint8_t data){
	return SoftI2C::MasterWriteByte(device_address, register_address, data, 0);
}
	
uint8_t SoftI2C::MasterWriteByte(uint8_t device_address, uint8_t register_address, uint8_t data, uint8_t writeverify ){	
	SoftI2C::MasterBusRestart();  delayMicroseconds(10000);
	if ((SoftI2C::MasterStart((device_address<<1)| I2C_WRITE))) {  SoftI2C::MasterStop(); return RETVAL_PROTOCOL_FAILURE;}
	if (SoftI2C::MasterWrite(register_address)) {  SoftI2C::MasterStop(); return RETVAL_PROTOCOL_FAILURE;}
	if (SoftI2C::MasterWrite(data)) {  SoftI2C::MasterStop(); return RETVAL_PROTOCOL_FAILURE;}
    SoftI2C::MasterStop();  
  
  if (writeverify != 0) {
  delayMicroseconds(10000); // Typical write time for i2c eeprom is 5ms.  Give the target chip some time to write the data.
    uint8_t readback_value;
    readback_value = SoftI2C::MasterReadByte(device_address, register_address);
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
	
uint8_t SoftI2C::MasterDumpAll() {
	Serial.println(F("I2C dump begin"));  Serial.flush();
	uint8_t addressRW;
	uint8_t number_of_chips;
	number_of_chips=0;
		uint8_t retval=0;
	for (addressRW = 0x01; addressRW < 0x7f; addressRW++ )	{
		retval=SoftI2C::MasterCheckExists(addressRW<<1); 
		if (retval == RETVAL_SUCCESS)    {
			number_of_chips++;
			SoftI2C::MasterDumpRegisters(addressRW);
		}
		delay(50);
	}
	Serial.print(F("I2C dump complete, found ")); Serial.print(number_of_chips); Serial.println(F(" chips!"));  Serial.flush();
	return number_of_chips;
}

uint8_t SoftI2C::MasterDumpRegisters(uint8_t addressRW) {	
	uint8_t number_of_registers=0;
	uint8_t register_address;
	uint8_t value;
	Serial.print(F("Dumping registers for 0x")); 
	if (addressRW < 16) {	Serial.print("0");	}
	Serial.print(addressRW, HEX);
	Serial.println(" !"); Serial.flush();
	for (register_address = 0; register_address < CHIP_SIZE-1; register_address++ )      {
		uint8_t readretval = SoftI2C::MasterReadByte(addressRW, register_address, &value);
		if(readretval!=RETVAL_SUCCESS) {Serial.print(F("---- "));}
		else {
			if (value < 16) {	Serial.print("0");	}
			Serial.print(value, HEX);
			Serial.print(" ");
			number_of_registers++;
		}
	}
	Serial.println(";");
	return number_of_registers;
}

 uint8_t SoftI2C::MasterBusRestart() {
	 SoftI2C::MasterStart();
	 SoftI2C::MasterStop();
  return RETVAL_SUCCESS;
}
	
 uint8_t SoftI2C::MasterStart() {
	// check that both sda and scl are high here maybe?
	SoftI2C::reset_timer(); 
	SoftI2C::data_pull_down();
	SoftI2C::wait_for_half_clock_period();
	SoftI2C::clock_pull_down();
	SoftI2C::wait_for_half_clock_period();
	SoftI2C::wait_for_half_clock_period();
	SoftI2C::wait_for_half_clock_period();
  return RETVAL_SUCCESS;
}

 uint8_t SoftI2C::MasterStart(uint8_t addressRW) {
	// check that both sda and scl are high here maybe?
	SoftI2C::reset_timer(); 
	SoftI2C::data_pull_down();
	SoftI2C::wait_for_half_clock_period();
	SoftI2C::clock_pull_down();
	SoftI2C::wait_for_half_clock_period();
	SoftI2C::wait_for_half_clock_period();
	SoftI2C::wait_for_half_clock_period();
  return SoftI2C::MasterWrite(addressRW);
}

 uint8_t SoftI2C::MasterRestart(uint8_t addressRW) {
	// check that both sda and scl are high here maybe?
	SoftI2C::clock_pull_down();
	SoftI2C::data_release();
	SoftI2C::wait_for_half_clock_period();
	SoftI2C::clock_release();
	SoftI2C::wait_for_half_clock_period();
  return SoftI2C::MasterStart(addressRW);
}


 void SoftI2C::MasterStop() {
	SoftI2C::wait_for_half_clock_period();
	SoftI2C::wait_for_half_clock_period();
	SoftI2C::wait_for_half_clock_period();
	SoftI2C::data_pull_down();
	SoftI2C::wait_for_half_clock_period();
	SoftI2C::clock_release();
	SoftI2C::wait_for_half_clock_period();
	SoftI2C::data_release();
	SoftI2C::wait_for_half_clock_period();
}


 uint8_t SoftI2C::MasterWrite(uint8_t value) {
//  Serial.print(F("Writing byte:"));  Serial.println(value,HEX);
	if((value & 0b10000000) == LOW){SoftI2C::data_pull_down();} else {SoftI2C::data_release();} 
	SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_release();
		SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_pull_down();
	if((value & 0b01000000) == LOW){SoftI2C::data_pull_down();} else {SoftI2C::data_release();} 
	SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_release();
		SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_pull_down();
	if((value & 0b00100000) == LOW){SoftI2C::data_pull_down();} else {SoftI2C::data_release();} 
	SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_release();
		SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_pull_down();
	if((value & 0b00010000) == LOW){SoftI2C::data_pull_down();} else {SoftI2C::data_release();} 
	SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_release();
		SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_pull_down();
	if((value & 0b00001000) == LOW){SoftI2C::data_pull_down();} else {SoftI2C::data_release();} 
	SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_release();
		SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_pull_down();
	if((value & 0b00000100) == LOW){SoftI2C::data_pull_down();} else {SoftI2C::data_release();} 
	SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_release();
		SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_pull_down();
	if((value & 0b00000010) == LOW){SoftI2C::data_pull_down();} else {SoftI2C::data_release();} 
	SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_release();
		SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_pull_down();
	if((value & 0b00000001) == LOW){SoftI2C::data_pull_down();} else {SoftI2C::data_release();} 
	SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_release();
		SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_pull_down();
	SoftI2C::data_release(); 
	SoftI2C::wait_for_half_clock_period();
	uint8_t rtn=0x00;
	SoftI2C::data_read(&rtn); SoftI2C::data_release();
	SoftI2C::clock_release();
		SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_pull_down();
	if(rtn==LOW) {return RETVAL_SUCCESS;} else {return RETVAL_PROTOCOL_FAILURE;}
}


 uint8_t SoftI2C::MasterRead(bool last) {
	uint8_t value = 0x00;
	uint8_t rtn;
	SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_release();
		SoftI2C::wait_for_half_clock_period();
	SoftI2C::data_read(&rtn); value <<= 1; value=value+(rtn>0);
		SoftI2C::clock_pull_down();
	SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_release();
		SoftI2C::wait_for_half_clock_period();
	SoftI2C::data_read(&rtn); value <<= 1; value=value+(rtn>0);
		SoftI2C::clock_pull_down();
	SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_release();
		SoftI2C::wait_for_half_clock_period();
	SoftI2C::data_read(&rtn); value <<= 1; value=value+(rtn>0);
		SoftI2C::clock_pull_down();
	SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_release();
		SoftI2C::wait_for_half_clock_period();
	SoftI2C::data_read(&rtn); value <<= 1; value=value+(rtn>0);
		SoftI2C::clock_pull_down();
	SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_release();
		SoftI2C::wait_for_half_clock_period();
	SoftI2C::data_read(&rtn); value <<= 1; value=value+(rtn>0);
		SoftI2C::clock_pull_down();
	SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_release();
		SoftI2C::wait_for_half_clock_period();
	SoftI2C::data_read(&rtn); value <<= 1; value=value+(rtn>0);
		SoftI2C::clock_pull_down();
	SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_release();
		SoftI2C::wait_for_half_clock_period();
	SoftI2C::data_read(&rtn); value <<= 1; value=value+(rtn>0);
		SoftI2C::clock_pull_down();
	SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_release();
		SoftI2C::wait_for_half_clock_period();
	SoftI2C::data_read(&rtn); value <<= 1; value=value+(rtn>0);
		SoftI2C::clock_pull_down();
  // send Ack or Nak
  if((last) == true){SoftI2C::data_release();} else {SoftI2C::data_pull_down();} 
	SoftI2C::wait_for_half_clock_period();
	SoftI2C::clock_release();
		SoftI2C::wait_for_half_clock_period();
		SoftI2C::clock_pull_down();
	return value;
}


  void SoftI2C::wait_for_half_clock_period(){	
	if(half_clock_period>0){
	//	uint16_t numtests=0;
	//	SoftI2C::reset_timer(); 	  	
		while(SoftI2C::get_timer()<half_clock_period) {
	//		Serial.println(TCNT1);
	//		NOP;
	//		numtests++;
		}
	//	Serial.println(numtests); Serial.flush();
		SoftI2C::reset_timer(); 	
	}
	return;
  }
  


  void SoftI2C::test_input_response_time(){  
	Serial.println(F("IIC snooping..."));
	uint8_t tretval;
	uint8_t breakloop=0;
	while(breakloop==0){
		tretval=get_next_bus_activity();
//		debug_pin_A_toggle();
		switch(tretval){
			case IIC_START					:	break;
			case IIC_STOP					:	breakloop=1; break;
			case IIC_ZERO					:	break;
			case IIC_ONE					:	break;
			case IIC_TIMEOUT				:	break;
			default							:	;
		}		  
	}
	Serial.print(F("\n\nDone with IIC snooping.\n")); Serial.flush();
  }
  
//  uint16_t SoftI2C::Snoop(uint16_t to_snoop){  
//	uint16_t activity_snooped=0;
//	uint8_t tretval;
//	uint8_t value=0x00;
//	uint8_t bitnumber=0;
//	Serial.println(F("IIC snooping..."));
//	while(activity_snooped<to_snoop){
//		tretval=get_next_bus_activity();
//		switch(tretval){
//			case IIC_START					:	Serial.print("\n"); 	bitnumber=0;	break;
//			case IIC_STOP					:	Serial.print("\\"); 	bitnumber=0;	break;
//			case IIC_ZERO					:				
//				if(bitnumber>=8) {activity_snooped++;if (value < 16) { 	Serial.print("0");	}Serial.print(value,HEX); Serial.print("+"); value=0x00; bitnumber=0;} 
//				else { value=(value<<1); bitnumber++;}											break;
//			case IIC_ONE					:	
//				if(bitnumber>=8) {activity_snooped++;if (value < 16) { 	Serial.print("0");	}Serial.print(value,HEX); Serial.print("-"); value=0x00; bitnumber=0;} 
//				else { value=(value<<1)+0x01; bitnumber++;}										break;
//			case IIC_TIMEOUT				:	Serial.print("*"); 						bitnumber=0;	break;
//			default							:	;
//		}		  
//	}
//	Serial.print(F("\n\nDone with IIC snooping.\n")); Serial.flush();
//return activity_snooped;    
//}
  
  
  // Note: Because the snoop loop must run very quickly, it is critical that printing output does not slow the loop.  All testing done at 8MHz
 void SoftI2C::fastprinthexbyte(uint8_t hexbyte){	 
// 	 SoftI2C::debug_pin_A_high();
	 	 
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

	
// SoftI2C::debug_pin_A_low();
  }


  
uint16_t SoftI2C::Snoop(uint16_t to_snoop){  
	uint16_t activity_snooped=0;
	uint8_t tretval;
	uint8_t value;
	Serial.println(F("IIC snoop..."));
	while(activity_snooped<to_snoop){
		tretval=SoftI2C::get_byte(&value);	
		switch(tretval){
			case RETVAL_SUCCESS					:
			tretval=SoftI2C::get_next_bus_activity(); 
			SoftI2C::fastprinthexbyte(value);
			activity_snooped++;			
				switch(tretval){
					case IIC_START					:	Serial.print("\n"); 		break;
					case IIC_STOP					:	Serial.print("\\"); 		break;
					case IIC_ZERO					:	Serial.print("+"); 			break;
					case IIC_ONE					:	Serial.print("-"); 			break;
					case IIC_TIMEOUT				:	Serial.print("*"); 			break;
					default							:	;
				}		  
																				break;
			case RETVAL_UNEXPECTED_START		:	Serial.print("\n"); 		break;
			case RETVAL_UNEXPECTED_STOP			:	Serial.print("\\"); 		break;
			case RETVAL_PROTOCOL_FAILURE		:	Serial.print("?"); 			break;
			case RETVAL_TIMEOUT					:	Serial.print("*"); 			break;
			default								:	;
		}	
	}
	Serial.print(F("\n\nDone with IIC snooping.\n")); Serial.flush();
return activity_snooped;    
}
  
  
  
  
  
  
  
  
  
	
uint8_t SoftI2C::SlaveHandleTransaction(
	bool (*fp_respond_to_chip_address)(uint8_t chip_address),
	uint8_t (*fp_get_register_address)(uint8_t chip_address ),
	uint8_t (*fp_set_register_address)(uint8_t chip_address, uint8_t reg_address ),
	uint8_t (*fp_generate_byte)(uint8_t chip_address, uint8_t* byte),
	uint8_t (*fp_receive_byte)(uint8_t chip_address, uint8_t byte)
){
	uint8_t TRANSACTION_STATE=TRANSACTION_STATE_GETTING_START;
	uint8_t tretval;
	uint8_t regaddr_has_been_set=0;
	uint8_t number_of_successful_bytes=0;
	uint8_t chip_address=0x00;	
	uint8_t value=0x00;
	SoftI2C::reset_timer();
	while(1){
		if(SoftI2C::transmission_timeout_reached()==true) {goto 	done_with_iic_transaction;}		
		switch(TRANSACTION_STATE){

			
			case TRANSACTION_STATE_GETTING_START:
				if(SoftI2C::spin_until_start()>0) {goto 	done_with_iic_transaction;}		
				SoftI2C::spin_until_clock_falls();	
				TRANSACTION_STATE=TRANSACTION_STATE_GETTING_CHIPADDRESS;
			break;	

	
			case TRANSACTION_STATE_GETTING_CHIPADDRESS:				
				tretval=SoftI2C::get_byte(&chip_address);
				switch (tretval) {
					case RETVAL_SUCCESS 			: 
					if((chip_address&I2C_RWMASK)==I2C_WRITE){
							TRANSACTION_STATE=TRANSACTION_STATE_GETTING_BYTE;
						} else {
							TRANSACTION_STATE=TRANSACTION_STATE_SENDING_BYTE;
						}
						chip_address=chip_address>>1;
						if( ((*fp_respond_to_chip_address)(chip_address)) == true ){
							if(SoftI2C::set_next_bus_activity(IIC_ACK)!=RETVAL_SUCCESS) {goto 	done_with_iic_transaction;}	
						} else {
							if(SoftI2C::set_next_bus_activity(IIC_NACK)!=RETVAL_SUCCESS) {goto 	done_with_iic_transaction;}	
						}

					break;
					case RETVAL_UNEXPECTED_START	: 	SoftI2C::spin_until_clock_falls();		TRANSACTION_STATE=TRANSACTION_STATE_GETTING_CHIPADDRESS; break;
					case RETVAL_UNEXPECTED_STOP		: 	goto done_with_iic_transaction; break;
					case RETVAL_PROTOCOL_FAILURE	: 	goto done_with_iic_transaction; break;
					case RETVAL_TIMEOUT				: 	goto done_with_iic_transaction; break;
					default 						:	goto done_with_iic_transaction; break;
				}
			break;	
	
			case TRANSACTION_STATE_GETTING_BYTE:
				tretval=SoftI2C::get_byte(&value);
				switch (tretval) {
					case RETVAL_SUCCESS 			: 
						if(regaddr_has_been_set==0){
							(*fp_set_register_address)(chip_address, value);
							regaddr_has_been_set=1;
						} else {
							// Handle the incoming byte.  Note: uses the built-in register address.
							(*fp_receive_byte)(chip_address, value);
							// Increment the built-in register address.
							(*fp_set_register_address)(chip_address, (*fp_get_register_address)(chip_address)+1);				
						}
						// TODO: add more granular ACK/NACK support here
						if(SoftI2C::set_next_bus_activity(IIC_ACK)!=RETVAL_SUCCESS) {goto 	done_with_iic_transaction;}		
						number_of_successful_bytes++;			
					break;						
					case RETVAL_UNEXPECTED_START	: 	SoftI2C::spin_until_clock_falls();		TRANSACTION_STATE=TRANSACTION_STATE_GETTING_CHIPADDRESS; break;
					case RETVAL_UNEXPECTED_STOP		: 	goto done_with_iic_transaction; break;
					case RETVAL_PROTOCOL_FAILURE	: 	goto done_with_iic_transaction; break;
					case RETVAL_TIMEOUT				: 	goto done_with_iic_transaction; break;
					default 						:	goto done_with_iic_transaction; break;
				}
			break;	
	
			case TRANSACTION_STATE_SENDING_BYTE:
					// Generate a new byte to send.  Note: uses the built-in register address.
				if(((*fp_generate_byte)(chip_address, &value))>0)  {goto 	done_with_iic_transaction;}	
	//			Serial.write('c'); SoftI2C::fastprinthexbyte(chip_address); Serial.write('t'); SoftI2C::fastprinthexbyte(value); Serial.println("");
				tretval=SoftI2C::set_byte(value);
				switch (tretval) {
					case RETVAL_SUCCESS 			: 				
						// Increment the built-in register address.
						(*fp_set_register_address)(chip_address, (*fp_get_register_address)(chip_address)+1);	
						number_of_successful_bytes++;
						// Look for an ACK to continue transmitting.  NACK terminates the transaction.
						TRANSACTION_STATE=TRANSACTION_STATE_WAITING_FOR_ACK; 
					break;
					case RETVAL_UNEXPECTED_START	: 	SoftI2C::spin_until_clock_falls();		TRANSACTION_STATE=TRANSACTION_STATE_GETTING_CHIPADDRESS; break;
					case RETVAL_UNEXPECTED_STOP		: 	goto done_with_iic_transaction; break;
					case RETVAL_PROTOCOL_FAILURE	: 	goto done_with_iic_transaction; break;
					case RETVAL_TIMEOUT				: 	goto done_with_iic_transaction; break;
					default 						:	goto done_with_iic_transaction; break;
				}
			break;	
			
			case TRANSACTION_STATE_WAITING_FOR_ACK:
				tretval=SoftI2C::get_next_bus_activity();	
				switch (tretval) {		
					case IIC_ZERO 		: 	SoftI2C::spin_until_clock_falls();		TRANSACTION_STATE=TRANSACTION_STATE_SENDING_BYTE;							break;
					case IIC_ONE		: 	goto done_with_iic_transaction; break;
					case IIC_START		: 	SoftI2C::spin_until_clock_falls();		TRANSACTION_STATE=TRANSACTION_STATE_GETTING_CHIPADDRESS; break;
					case IIC_STOP		: 	goto done_with_iic_transaction; break;
					case IIC_TIMEOUT	: 	goto done_with_iic_transaction; break;
					default 		:	goto done_with_iic_transaction; break;
				}
			break;	
			
	
			default: ;
		}
	}	
	done_with_iic_transaction:	// always exit with a clean interface
	SoftI2C::clock_release();
	SoftI2C::data_release();
	return number_of_successful_bytes;
}

	
	
// As a reminder, all SoftI2CSlave functions return the following codes unless otherwise specified:
// RETVAL_SUCCESS 				// The function has exited successfully.
// RETVAL_UNEXPECTED_START	    // During operation, the bus encountered an unexpected start condition.
// RETVAL_UNEXPECTED_STOP	    // During operation, the bus encountered an unexpected stop condition.
// RETVAL_PROTOCOL_FAILURE	    // Probably the communication was NAK'd
// RETVAL_TIMEOUT			    // The bus timed out within the function.


 uint8_t SoftI2C::set_next_bus_activity(uint8_t value){
	uint8_t tretval;
	if(value==IIC_ZERO){
	tretval=SoftI2C::spin_until_clock_falls(); 		if(tretval!=RETVAL_SUCCESS) {return tretval;}
	data_pull_down(); 
	tretval=SoftI2C::spin_until_clock_rises();		if(tretval!=RETVAL_SUCCESS) {return tretval;}
	}
	if(value==IIC_ONE){SoftI2C::debug_pin_A_high();
	tretval=SoftI2C::spin_until_clock_falls();		if(tretval!=RETVAL_SUCCESS) {return tretval;}
	data_release(); 
	tretval=SoftI2C::spin_until_clock_rises();		if(tretval!=RETVAL_SUCCESS) {return tretval;}
	SoftI2C::debug_pin_A_low();
	}
	if(value==IIC_ACK){
	tretval=SoftI2C::spin_until_clock_falls();		if(tretval!=RETVAL_SUCCESS) {return tretval;}
	data_pull_down(); 
	tretval=SoftI2C::spin_until_clock_rises();		if(tretval!=RETVAL_SUCCESS) {return tretval;}
	tretval=SoftI2C::spin_until_clock_falls();		if(tretval!=RETVAL_SUCCESS) {return tretval;}
	data_release();

	}
	if(value==IIC_NACK){
	tretval=SoftI2C::spin_until_clock_falls();		if(tretval!=RETVAL_SUCCESS) {return tretval;}
	data_release();
	tretval=SoftI2C::spin_until_clock_rises();		if(tretval!=RETVAL_SUCCESS) {return tretval;}
	tretval=SoftI2C::spin_until_clock_falls();		if(tretval!=RETVAL_SUCCESS) {return tretval;}
	}
	return RETVAL_SUCCESS;
}
  
	
	
uint8_t SoftI2C::set_byte(uint8_t value){  
	uint8_t tretval;
	uint8_t my_bit=0b10000000;
	SoftI2C::reset_timer();
	while(my_bit>0){
	if((value&my_bit)==0) {tretval=SoftI2C::set_next_bus_activity(IIC_ZERO);}
		else {tretval=SoftI2C::set_next_bus_activity(IIC_ONE);}
		if(tretval!=RETVAL_SUCCESS) {return tretval;}
		my_bit=my_bit>>1;
	}
	return RETVAL_SUCCESS;
}
	
	
	
	
	
	
	
	
	
	
	
  


 
uint8_t SoftI2C::get_byte(uint8_t* value){  
	uint8_t tretval;
	uint8_t my_bit=0b10000000;
	*value=0x00;
	SoftI2C::reset_timer();
	while(my_bit>0){
		tretval=get_next_bus_activity();
		switch(tretval){
			case IIC_START		:	return RETVAL_UNEXPECTED_START; 	break;
			case IIC_ONE		:	*value=*value+my_bit; my_bit=my_bit>>1;break;
			case IIC_ZERO		:	my_bit=my_bit>>1; break;
			case IIC_STOP		:	return RETVAL_UNEXPECTED_STOP; 		break;
			case IIC_TIMEOUT	:	return RETVAL_TIMEOUT; 				break;
			default				:	;
		}		  
	}
	return RETVAL_SUCCESS;
}

// Note: The 'spin until x' functions are different/separate because they do the bus_read at the end of the loop to allow for an immediate exit under high speed operation.
 uint8_t SoftI2C::spin_until_start(){
	 SoftI2C::debug_pin_B_high();
	while(1){
		if(IIC_STATE==0b00001110){	return RETVAL_SUCCESS; }
		if(SoftI2C::transmission_timeout_reached()==true) {return IIC_TIMEOUT;}	
		SoftI2C::bus_read();
	}
}

 uint8_t SoftI2C::spin_until_clock_rises(){
	// If needed, this function could be optimized as the data line is ignorable while the clock is low.
	while(1){
		if((IIC_STATE&IIC_STATE_MASK_SCL_CURRENT)!=0){return RETVAL_SUCCESS; }
		if(SoftI2C::transmission_timeout_reached()==true) {return IIC_TIMEOUT;}	
		SoftI2C::bus_read();
	}
}

 uint8_t SoftI2C::spin_until_clock_falls(){
	while(1){
		switch(IIC_STATE){
			case 0b00001101: return RETVAL_UNEXPECTED_STOP; break;
			case 0b00001110: return RETVAL_UNEXPECTED_START; break;
			case 0b00001000: return RETVAL_SUCCESS; break;
			case 0b00001001: return RETVAL_SUCCESS; break;
			case 0b00001010: return RETVAL_SUCCESS; break;
			case 0b00001011: return RETVAL_SUCCESS; break;
			case 0b00000000: return RETVAL_SUCCESS; break;
			case 0b00000001: return RETVAL_SUCCESS; break;
			case 0b00000010: return RETVAL_SUCCESS; break;
			case 0b00000011: return RETVAL_SUCCESS; break;
			default :;
		}
		if(SoftI2C::transmission_timeout_reached()==true) {return IIC_TIMEOUT;}	
		SoftI2C::bus_read();
	}
}


 uint8_t SoftI2C::get_next_bus_activity(){
//	 SoftI2C::debug_pin_A_high();
	while(1){
		SoftI2C::bus_read();
//	This code relies on the format of IIC_STATE
//#define IIC_STATE_MASK_SDA_CURRENT  0b00000001
//#define IIC_STATE_MASK_SDA_PREVIOUS 0b00000010
//#define IIC_STATE_MASK_SCL_CURRENT  0b00000100
//#define IIC_STATE_MASK_SCL_PREVIOUS 0b00001000
		switch(IIC_STATE){
			case 0b00000100: 	   return IIC_ZERO; break; //debug_pin_A_low(); in all these cases
			case 0b00000110: 	   return IIC_ZERO; break;
			case 0b00000101: 	   return IIC_ONE; break;
			case 0b00000111: 	   return IIC_ONE; break;
			case 0b00001101: 	   return IIC_STOP; break;
			case 0b00001110: 	   return IIC_START; break;
			default :;
		}
		if(SoftI2C::transmission_timeout_reached()==true) {return IIC_TIMEOUT;}	
	}
}
	
 uint8_t SoftI2C::get_next_raw_bus_activity(){	
	while(1){
		SoftI2C::bus_read();
//	This code relies on the format of IIC_STATE
//#define IIC_STATE_MASK_SDA_CURRENT  0b00000001
//#define IIC_STATE_MASK_SDA_PREVIOUS 0b00000010
//#define IIC_STATE_MASK_SCL_CURRENT  0b00000100
//#define IIC_STATE_MASK_SCL_PREVIOUS 0b00001000
		switch(IIC_STATE){
			case 0b00000100: return IIC_ZERO; break;
			case 0b00000110: return IIC_ZERO; break;
			case 0b00000101: return IIC_ONE; break;
			case 0b00000111: return IIC_ONE; break;
			case 0b00001101: return IIC_STOP; break;
			case 0b00001110: return IIC_START; break;
			case 0b00001000: return IIC_CLOCKFALL; break;
			case 0b00001001: return IIC_CLOCKFALL; break;
			case 0b00001010: return IIC_CLOCKFALL; break;
			case 0b00001011: return IIC_CLOCKFALL; break;
			default :;
		}
		if(SoftI2C::transmission_timeout_reached()==true) {return IIC_TIMEOUT;}	
	}
}
  
  void SoftI2C::reset_timer(){  TCNT1=0; 	TCCR1B = 0x01;  }
  uint16_t SoftI2C::get_timer(){ return TCNT1;  }
  void SoftI2C::set_timer(uint16_t value){  TCNT1=value;  }
    	
   uint8_t SoftI2C::transmission_timeout_reached(){
		if(timeout_is_enabled==1) {
			if(SoftI2C::get_timer()>transmission_timeout) {return 1;}
		}
		return 0;
	}
  
  void SoftI2C::clock_pull_down(){  
		*_sclPortReg  &=~ _sclBitMask;  
		*_sclDirReg   |=  _sclBitMask; 
	}
	
  void SoftI2C::clock_release(){
		*_sclDirReg   &=~ _sclBitMask;  
		if(usePullups) { *_sclPortReg  |=  _sclBitMask; }
	}
 	
     void SoftI2C::data_pull_down(){  
		*_sdaPortReg  &=~ _sdaBitMask;  
		*_sdaDirReg   |=  _sdaBitMask; 
	}
	
     void SoftI2C::data_release(){
		*_sdaDirReg   &=~ _sdaBitMask;  
		if(usePullups) { *_sdaPortReg  |=  _sdaBitMask; }
	}
	
	 void SoftI2C::clock_read(uint8_t* SCL){
		*SCL= ((*_sclPortRegIN) & (_sclBitMask)); 
	}

	 void SoftI2C::data_read(uint8_t* SDA){
		*SDA= ((*_sdaPortRegIN) & (_sdaBitMask))>0; 
	}
	
 void SoftI2C::bus_read(){		
	// This is probably known at compile-time, so let the compiler pick and optimize away the if-else
	if(_sclPortRegIN == _sdaPortRegIN){
	// READ BOTH PINS SIMULTANEOUSLY
		uint8_t sampled_io=*_sclPortRegIN;
		IIC_STATE=((IIC_STATE&IIC_STATE_MASK_CURRENT)<<1);
		if(sampled_io & _sclBitMask) {IIC_STATE=IIC_STATE+IIC_STATE_MASK_SCL_CURRENT;}
		if(sampled_io & _sdaBitMask) {IIC_STATE=IIC_STATE+IIC_STATE_MASK_SDA_CURRENT;}
	}else {
	// READ BOTH PINS SEPERATELY
		uint8_t sampled_io_scl=*_sclPortRegIN;
		uint8_t sampled_io_sda=*_sdaPortRegIN;	
		IIC_STATE=((IIC_STATE&IIC_STATE_MASK_CURRENT)<<1);
		if(sampled_io_scl & _sclBitMask) {IIC_STATE=IIC_STATE+IIC_STATE_MASK_SCL_CURRENT;}
		if(sampled_io_sda & _sdaBitMask) {IIC_STATE=IIC_STATE+IIC_STATE_MASK_SDA_CURRENT;}
	}		
}



























	
	/////////////////////////////////////////		SUPPORTING FUNCTIONS
	
	
	
	
	
#ifdef SCL_PIN
#ifdef SDA_PIN
    SoftI2C::SoftI2C(){	SoftI2C::init(SCL_PIN, SDA_PIN, true, I2C_SPEED_DEFAULT);	}
#endif
#endif

    SoftI2C::SoftI2C(uint8_t pin_scl, uint8_t pin_sda){		SoftI2C::init(pin_scl, pin_sda, true, I2C_SPEED_DEFAULT);	}
	SoftI2C::SoftI2C(uint8_t pin_scl, uint8_t pin_sda, bool pullups){SoftI2C::init(pin_scl, pin_sda, pullups, I2C_SPEED_DEFAULT);	} 
	SoftI2C::SoftI2C(uint8_t pin_scl, uint8_t pin_sda, bool pullups, uint16_t speed){ SoftI2C::init(pin_scl, pin_sda, pullups, speed);	}
			
    SoftI2C::~SoftI2C(){
		// Restore pin settings
		pinMode(PIN_SCL, INPUT);
		pinMode(PIN_SDA, INPUT);
		// Restore timer1 settings 
		TCCR1A	= p_TCCR1A;
		TCCR1B	= p_TCCR1B;
		OCR1A	= p_OCR1A;
		OCR1B	= p_OCR1B;
		TIMSK1	= p_TIMSK1;   
		TIFR1	= p_TIFR1;  	
		interrupts();	 
	}


	
 void    SoftI2C::init(uint8_t pin_scl, uint8_t pin_sda, uint8_t pullups, uint16_t speed){
		usePullups=pullups;
		PIN_SCL = pin_scl;
		PIN_SDA = pin_sda;
	if(pullups==true){
		pinMode(PIN_SCL, INPUT_PULLUP);
		pinMode(PIN_SDA, INPUT_PULLUP);
	}else{
		pinMode(PIN_SCL, INPUT);
		pinMode(PIN_SDA, INPUT);
	}
	
		_sclBitMask = digitalPinToBitMask(pin_scl);  
		_sclPortRegIN = portInputRegister(digitalPinToPort(pin_scl));		
		_sclPortReg  = portOutputRegister(digitalPinToPort(pin_scl));
		_sclDirReg   = portModeRegister(digitalPinToPort(pin_scl));   
		_sdaBitMask = digitalPinToBitMask(pin_sda);   
		_sdaPortRegIN = portInputRegister(digitalPinToPort(pin_sda));
		_sdaPortReg  = portOutputRegister(digitalPinToPort(pin_sda));
		_sdaDirReg   = portModeRegister(digitalPinToPort(pin_sda)); 
		_busBitMask =_sclBitMask|_sdaBitMask;
	
		noInterrupts();
	   		// Save timer1 settings 
		p_TCCR1A	= TCCR1A;
		p_TCCR1B	= TCCR1B;
		p_OCR1A		= OCR1A;
		p_OCR1B		= OCR1B;
		p_TIMSK1	= TIMSK1;  	 
		p_TIFR1		= TIFR1;  	 
			// Set new timer1 settings
		TCCR1A= 0; // turn off PWM
		TCCR1B = 0x01; // set clock scaler to 1:1
		OCR1A = 0;
		OCR1B = 0; 
		TIMSK1 = 0 ; 
		TIFR1 = 0 ; 
		
		SoftI2C::SetSpeed(speed);
		
		SoftI2C::clock_release();
		SoftI2C::data_release();
		
		SoftI2C::InitDebugpins();
		SoftI2C::debug_pin_test();
		interrupts();
	}

 
	
	void SoftI2C::SetSpeed(uint16_t speed){		
	I2C_SPEED=speed;					// Units=KHz
	uint16_t timer1_tick_rate=SoftI2C::GetFrequencyOfTimer1Divider(SoftI2C::GetCurrentTimer1Divider());
	half_clock_period=((timer1_tick_rate)/(2*I2C_SPEED));	
	if(half_clock_period>HALF_CLOCK_TIMER_OVERHEAD_COMPENASTION){half_clock_period=half_clock_period-HALF_CLOCK_TIMER_OVERHEAD_COMPENASTION;} else {half_clock_period=0;}
	transmission_timeout=half_clock_period*128;
	Serial.print(F("CPU speed (in KHz):"));  Serial.println(F_CPU/1000);  Serial.flush();
	Serial.print(F("I2C speed (in KHz):"));  Serial.println(I2C_SPEED);  Serial.flush();
	Serial.print(F("Timer compensation:"));  Serial.println(HALF_CLOCK_TIMER_OVERHEAD_COMPENASTION);
	Serial.print(F("Timer1 divider:"));  Serial.println(SoftI2C::GetCurrentTimer1Divider());
	Serial.print(F("Timer1 tick rate (in KHz):"));  Serial.println(timer1_tick_rate);
	Serial.print(F("Transaction timeout (in timer1 ticks):"));  Serial.println(transmission_timeout);
	Serial.print(F("I2C halfclock period in timer1 ticks:"));  Serial.println(half_clock_period); Serial.flush();	
	}

 
uint8_t  SoftI2C::GetCurrentTimer1Divider(){return TCCR1B&0b00000111;}
uint8_t  SoftI2C::GetCurrentTimer2Divider(){return TCCR2B&0b00000111;}

uint16_t SoftI2C::GetFrequencyOfTimer1Divider(uint8_t divider){ // returns ticks per millisecond
	return ((F_CPU/1000)/SoftI2C::GetRatioOfTimer1Divider(divider));
}

uint16_t SoftI2C::GetFrequencyOfTimer2Divider(uint8_t divider){ // returns ticks per millisecond
	return ((F_CPU/1000)/SoftI2C::GetRatioOfTimer2Divider(divider));
}


uint16_t SoftI2C::GetRatioOfTimer1Divider(uint8_t divider){ // returns clocks per tick
  switch (divider) {
  case 0x01:                return (    1 );     break;
  case 0x02:                return (    8 );     break;
  case 0x03:                return (   64 );     break;
  case 0x04:                return (  256 );     break;
  case 0x05:                return ( 1024 );     break;
  default:                  return (    0 );     
  }
}

uint16_t SoftI2C::GetRatioOfTimer2Divider(uint8_t divider){ // returns clocks per tick
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


//////////////////////////// DEBUGGING STUFF IS BELOW THIS LINE

 void SoftI2C::InitDebugpins(){			
#ifdef DEBUGPIN_A
		pinMode2(DEBUGPIN_A, OUTPUT);
		SoftI2C::debug_pin_A_low();
#endif
#ifdef DEBUGPIN_B
		pinMode2(DEBUGPIN_B, OUTPUT);
		SoftI2C::debug_pin_B_low();
#endif
 }
	
 void SoftI2C::debug_pin_test(){
	 SoftI2C::debug_pin_A_test();
	 SoftI2C::debug_pin_B_test();	 
 }
 
 
 void SoftI2C::debug_pin_A_test(){
#ifdef DEBUGPIN_A
	 Serial.println(F("Testing debug IO A"));
 for (uint8_t i=8; i>0; i--){	 SoftI2C::debug_pin_A_toggle();	 delayMicroseconds(100); }
#endif
}
	 void SoftI2C::debug_pin_B_test(){
#ifdef DEBUGPIN_B
	 Serial.println(F("Testing debug IO B"));
 for (uint8_t i=8; i>0; i--){	 SoftI2C::debug_pin_B_toggle();	 delayMicroseconds(100); }
#endif
}
	
 void SoftI2C::debug_pin_A_high(){
#ifdef DEBUGPIN_A
	DEBUG_PIN_A_STATE=1;	digitalWrite2(DEBUGPIN_A, HIGH);
#endif
}

 void SoftI2C::debug_pin_A_low(){
#ifdef DEBUGPIN_A
	DEBUG_PIN_A_STATE=0;	digitalWrite2(DEBUGPIN_A, LOW );
#endif
}

 void SoftI2C::debug_pin_A_toggle(){
#ifdef DEBUGPIN_A
	if(DEBUG_PIN_A_STATE!=0) 	{ SoftI2C::debug_pin_A_low();}
	else 						{ SoftI2C::debug_pin_A_high();}
#endif
}

 void SoftI2C::debug_pin_B_high(){
#ifdef DEBUGPIN_B
	DEBUG_PIN_B_STATE=1;	digitalWrite2(DEBUGPIN_B, HIGH);
#endif
}

 void SoftI2C::debug_pin_B_low(){
#ifdef DEBUGPIN_B
	DEBUG_PIN_B_STATE=0;	digitalWrite2(DEBUGPIN_B, LOW );
#endif
}

 void SoftI2C::debug_pin_B_toggle(){
#ifdef DEBUGPIN_B
	if(DEBUG_PIN_B_STATE!=0) 	{ SoftI2C::debug_pin_B_low();}
	else 						{ SoftI2C::debug_pin_B_high();}	
#endif
}
	
	// NOTE: this debugging feature uses timer2.
	  void SoftI2C::functimer_start(){
  		TCCR2A= 0; // turn off PWM
		TCCR2B = 0x00; // set clock scaler to 0
		OCR2A = 0;
		OCR2B = 0; 
		TIMSK2 = 0 ; 
		TCNT2=0;
		TCCR2B = 0x03; // set clock scaler to 8
}
    void SoftI2C::functimer_stop(){
		uint16_t mytimer=TCNT2;
		mytimer=mytimer*SoftI2C::GetRatioOfTimer2Divider(SoftI2C::GetCurrentTimer2Divider());
		Serial.print(F("K:\t")); Serial.println(mytimer,DEC);
	}
