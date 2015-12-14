#include <avr/pgmspace.h>
#include <SoftIIC.h>

// Note: these can be *any* pins, not just a4/a5.
#define SDA_PIN  A4
#define SCL_PIN  A5
SoftI2C* my_SoftI2C;


#define SERIAL_PORT_SPEED 115200

static uint8_t current_register_address_for_50 = 0x00;
static uint8_t current_register_address_for_51 = 0x00;
static uint8_t current_register_address_for_52 = 0x00;
static uint8_t current_register_address_for_53 = 0x00;

PROGMEM const uint8_t MY_VIRTUAL_EEPROM50[]             = {0x00, 0xFF, 0x01, 0x02, 0x03, 0xab};
PROGMEM const uint8_t MY_VIRTUAL_EEPROM51[]             = {0x01};
PROGMEM const uint8_t MY_VIRTUAL_EEPROM52[]             = {0x02};
PROGMEM const uint8_t MY_VIRTUAL_EEPROM53[]             = {0x03};


void setup() {
  Serial.begin(SERIAL_PORT_SPEED);
}

void loop() {

  my_SoftI2C = new SoftI2C(SCL_PIN, SDA_PIN, true, 100);

  // First act as a master and  dump all registers of all slaves
  // note: library not yet multimaster-safe.  assumes the toher master is not trying to perform any transactions
  my_SoftI2C->MasterDumpAll();
  delay(1000);

  // Then act as a fly on the wall and monitor the i2c bus, listening for traffic
  my_SoftI2C->Snoop(1024);

  delay(1000);

  // Last, act as four slave devices at once, emulating a 24c04 eeprom (read-only)
  uint8_t successful_bytes = 0;
  uint16_t TOTAL_EXPECTED_BYTES = 1024;
  while (successful_bytes < TOTAL_EXPECTED_BYTES) {
    successful_bytes = successful_bytes + my_SoftI2C->SlaveHandleTransaction(addr_check_i2c_slave, get_current_register_address, set_current_register_address, read_i2c_slave, write_i2c_slave);
  }

  delay(1000);

  delete my_SoftI2C;

  delay(10000);
}



//////////////////////////////////////////////////////////// These functions should be edited to give the i2c slave a 'personality'. ////////////////////////////////////////////////////////////////



uint8_t virtualeeprom(uint8_t chipaddress, uint8_t registeraddress) {
  uint8_t retval = 0xFF;
  if (chipaddress == 0x50 && registeraddress < (sizeof(MY_VIRTUAL_EEPROM50) / sizeof(uint8_t))) {    retval = pgm_read_byte_near(MY_VIRTUAL_EEPROM50 + registeraddress);  }
  if (chipaddress == 0x51 && registeraddress < (sizeof(MY_VIRTUAL_EEPROM51) / sizeof(uint8_t))) {    retval = pgm_read_byte_near(MY_VIRTUAL_EEPROM51 + registeraddress);  }
  if (chipaddress == 0x52 && registeraddress < (sizeof(MY_VIRTUAL_EEPROM52) / sizeof(uint8_t))) {    retval = pgm_read_byte_near(MY_VIRTUAL_EEPROM52 + registeraddress);  }
  if (chipaddress == 0x53 && registeraddress < (sizeof(MY_VIRTUAL_EEPROM53) / sizeof(uint8_t))) {    retval = pgm_read_byte_near(MY_VIRTUAL_EEPROM53 + registeraddress);  }
  return retval;
}

bool addr_check_i2c_slave(uint8_t chipaddr) {
  if (chipaddr == 0x50) {    return true;  }
  if (chipaddr == 0x51) {    return true;  }
  if (chipaddr == 0x52) {    return true;  }
  if (chipaddr == 0x53) {    return true;  }
  return false;
}

uint8_t get_current_register_address(uint8_t chipaddr) {
  if (chipaddr == 0x50) {    return current_register_address_for_50;  }
  if (chipaddr == 0x51) {    return current_register_address_for_51;  }
  if (chipaddr == 0x52) {    return current_register_address_for_52;  }
  if (chipaddr == 0x53) {    return current_register_address_for_53;  }
  return 0x00;
}


uint8_t set_current_register_address(uint8_t chipaddr, uint8_t regaddr) {
  if (chipaddr == 0x50) {    current_register_address_for_50 = regaddr;  }
  if (chipaddr == 0x51) {    current_register_address_for_51 = regaddr;  }
  if (chipaddr == 0x52) {    current_register_address_for_52 = regaddr;  }
  if (chipaddr == 0x53) {    current_register_address_for_53 = regaddr;  }
  return 0x00;
}

uint8_t read_i2c_slave(uint8_t chipaddress, uint8_t* value) {
  uint8_t registeraddress = get_current_register_address(chipaddress);
  *value = virtualeeprom( chipaddress, registeraddress);
  return 0x00;
}

uint8_t write_i2c_slave(uint8_t chipaddr, uint8_t value) {
  // Don't do anything with writes for this demo.
  return 0x00;
}


