#include <avr/pgmspace.h>
#include <SoftIIC.h>

// Note: these can be *any* pins, not just a4/a5.
#define SDA_PIN  A4
#define SCL_PIN  A5

#define SERIAL_PORT_SPEED 38400
#define IIC_SPEED 50


   SoftIIC  my_SoftIIC = SoftIIC(SCL_PIN, SDA_PIN, true, IIC_SPEED, true);
  
void setup() {
  Serial.begin(SERIAL_PORT_SPEED);  
  noInterrupts();
}

void loop() {
  my_SoftIIC.MasterDumpAll();
  delay(1000);

   my_SoftIIC.MasterWriteByte( 0x50,  0x00,  0xAB);
    uint8_t value;
   my_SoftIIC.MasterReadByte( 0x50,  0x00,  &value);
  delay(1000);
   
}

