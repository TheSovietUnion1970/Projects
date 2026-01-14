#ifndef MAIN_H
#define MAIN_H

/* PROGRAM CONTROL */
// TX_TRIGGER 0 means every FIFO has 0 + 1 byte, DMA is activated and transfer data (avoid 2 more bytes to be transfered)
#define TX_TRIGGER 0 // here meaning remaining free TX_TRIGGER bytes will de-activate TX interuupt

/* USER CONTROL */
#define SLAVE_ADDRESS 0x40 /* Slave address of the slave device */

#define PRINT_DEBUG 0 /* Print debug info print */

#define END_STRING '@' /* Mark end string sent with this will replace '\n' after writtting
                        data into /dev/i2c1 */

#define DMA_USED 0 /* Usd DMA or not */

#endif