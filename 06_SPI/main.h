#ifndef MAIN_H
#define MAIN_H

/* ################# [SPI0_TX] ################# */
/* USER CONTROL */
#define DMA_USED_TX 1
#define PRINT_DEBUG_TX 1 /* Print debug info print */

/* PROGRAM CONTROL */
#if (DMA_USED_TX)
#define TX_TRIGGER 0 /* it's must be fixed */
#else
#define TX_TRIGGER 0 /* it's must be fixed */
#endif


/* ################# [SPI1_RX] ################# */
/* USER CONTROL */
#define DMA_USED_RX 0 /* Use DMA RX or not */
#define PRINT_DEBUG_RX 0 /* Print debug info print */

/* PROGRAM CONTROL */
#if (DMA_USED_RX) /* DMA is used */
#define RX_TRIGGER 0 /* it's must be fixed*/
#else /* Non-DMA is used */
#define RX_TRIGGER 0 /* it's must be fixed */
#define FIFO_DRAINED 1 /* USER CONTROL */
#endif

#endif