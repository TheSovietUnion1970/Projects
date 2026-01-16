SPI for data transfer between master (spi0) ans slave (spi1) on the same board

In main.h:
 - SPI0_TX:
 	- DMA_USED_TX - used for DMA transfter for master spi0
 	- PRINT_DEBUG_TX - used for printk for master spi0
 - SPI0_RX
 	- DMA_USED_RX - used for DMA transfter for slave spi1
 	- PRINT_DEBUG_RX - used for printk for slave spi1
 	- FIFO_DRAINED - used for draining FIFO
 
SPI pins:
 - SPI0_TX:
	- P9_17: spi0_cs0
	- P9_21: spi0_d0 (MISO)
	- P9_18: spi0_d1 (MOSI)
	- P9_22: spi0_sclk
 - SPI0_RX:
	- P9_28: spi1_cs0
	- P9_29: spi1_d0 (MISO)
	- P9_30: spi1_d1 (MOSI)
	- P9_31: spi1_sclk
 
Test:
 - Write data into /dev/spi0: 'echo "Hi everyone!" | sudo tee /dev/spi0'
 - Read data from /dev/spi1: 'sudo cat /dev/spi1'

Can use TX (non-dma or dma) with RX (non-dma or dma) with some notes below due to constrains when running master and slave spi on the same BBB board:
 - TX(non-dma) and RX(non-dma) -> not using FIFO_DRAINED
 - TX(dma) and RX(non-dma) -> should use FIFO_DRAINED
