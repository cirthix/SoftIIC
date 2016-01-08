#include <avr/pgmspace.h>
#include <SoftIIC.h>

// Note: these can be *any* pins, not just a4/a5.
#define SDA_PIN  A4
#define SCL_PIN  A5

#define SERIAL_PORT_SPEED 38400
#define IIC_SPEED 50

static uint8_t current_register_address_for_50 = 0x00;
static uint8_t current_register_address_for_51 = 0x00;

PROGMEM const uint8_t MY_VIRTUAL_EEPROM50[]             = {0x00, 0x0F, 0x01, 0x02, 0x03, 0xab};
PROGMEM const uint8_t MY_VIRTUAL_EEPROM51[]             = {0xF0, 0xFF, 0xF1, 0xAA, 0x09, 0xa4};

   SoftIIC  my_SoftIIC = SoftIIC(SCL_PIN, SDA_PIN, true, IIC_SPEED, true);
  
void setup() {
  Serial.begin(SERIAL_PORT_SPEED);  
  noInterrupts();
}

void loop() {

  // Last, act as A 24c04 eeprom (read-only) slave
  uint8_t successful_bytes = 0;
  uint16_t TOTAL_EXPECTED_BYTES = 512;
  while (successful_bytes < TOTAL_EXPECTED_BYTES) {
    successful_bytes = successful_bytes + my_SoftIIC.SlaveHandleTransaction(respond_to_address, respond_to_command, respond_to_data, get_current_register_address, set_current_register_address, read_iic_slave, write_iic_slave);
  }

//  delay(10000);
}



//////////////////////////////////////////////////////////// These functions should be edited to give the iic slave a 'personality'. ////////////////////////////////////////////////////////////////



uint8_t virtualeeprom(uint8_t chipaddress, uint8_t registeraddress) {
  uint8_t retval = 0xFF;
  if (chipaddress == 0x50 && registeraddress < (sizeof(MY_VIRTUAL_EEPROM50) / sizeof(uint8_t))) {    retval = pgm_read_byte_near(MY_VIRTUAL_EEPROM50 + registeraddress);  }
  if (chipaddress == 0x51 && registeraddress < (sizeof(MY_VIRTUAL_EEPROM51) / sizeof(uint8_t))) {    retval = pgm_read_byte_near(MY_VIRTUAL_EEPROM51 + registeraddress);  }
  return retval;
}


uint8_t respond_to_address(uint8_t chipaddr){  
  if((chipaddr>>1)==0x50) {return 0x01;}
  if((chipaddr>>1)==0x51) {return 0x01;}
 return 0x00;  
}


uint8_t respond_to_command(uint8_t commandaddr){  
 return 0x01;  
}


uint8_t respond_to_data(uint8_t commandaddr){  
 return 0x01;  
}


uint8_t get_current_register_address(uint8_t chipaddr) {
  if (chipaddr == 0x50) {    return current_register_address_for_50;  }
  if (chipaddr == 0x51) {    return current_register_address_for_51;  }
  return 0x00;
}


uint8_t set_current_register_address(uint8_t chipaddr, uint8_t regaddr) {
  if (chipaddr == 0x50) {    current_register_address_for_50 = regaddr;  }
  if (chipaddr == 0x51) {    current_register_address_for_51 = regaddr;  }
  return 0x00;
}

uint8_t read_iic_slave(uint8_t chipaddress, uint8_t* value) {
  uint8_t registeraddress = get_current_register_address(chipaddress);
  *value = virtualeeprom( chipaddress, registeraddress);
  return 0x00;
}

uint8_t write_iic_slave(uint8_t chipaddr, uint8_t value) {
  // Don't do anything with writes for this demo.
  return 0x00;
}
