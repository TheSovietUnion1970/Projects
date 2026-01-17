UART for data transfer between BBB and Arduino

UART pins:
 - P9_26: uart1_rxd
 - P9_24: uart1_txd

Test:
 - Write data into /dev/uart1: 'echo "Hi everyone!" | sudo tee /dev/uart1'
 - Then Arduino will echo the data back

