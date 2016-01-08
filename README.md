# SoftIIC
A software IIC/I2C library aimed at the arduino/atmega platform. Master and multi-slave supported.

SoftIIC is a class which enables easy communication to other IIC devices, as a master or slave, or in the presence of other masters.




Master mode features:

  With a 16MHz core clock, 100KHz IIC master is supported.  Slower core clocks may also work.
  
  Can tolerate the presence of other masters in most cases.  (Safety not guaranteed)
  
  The bus-restart functionality is supported.
  
  Multi-byte reads and writes are supported.
  
  Clock stretching in master mode is not supported yet.




Slave mode features:

  With a 16MHz core clock, 100KHz IIC slave is supported.  This is about the limit of operation.
  
  Multiple simultaneous slave addresses are supported.
  
  Function callback system for granular ACK/NACK support.
  
  The bus-restart functionality is supported.
  
  Multi-byte reads and writes are supported.
  
  Clock stretching in slave mode is not supported/used.
  



SoftIIC requires:
Timer1
Two I/O pins on the same port (for simultaneous sampling)


SoftIIC currently is tested on:
Atmega168

