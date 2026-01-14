I2C for data transfer between BBB and Arduino

Use Arduino for testing communication between Arduino and BeagleBoneBlack by sending data to arduino (slave) from BBB (master)

In main.h:
 - SLAVE_ADDRESS is used to set slave address
 - DMA_USED is used to set/unset to use DMA in I2C
 
I2C pins:
 - P9_17: i2c1_scl
 - P9_18: i2c1_sda
 
Test:
 - Write data into /dev/i2c1: 'echo "Hi everyone!" | sudo tee /dev/i2c1'
