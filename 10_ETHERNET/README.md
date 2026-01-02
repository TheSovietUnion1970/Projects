- Ethernet is in dual mode

- Can test the functionality by ping, ssh, ...

- Before loading eth.ko, it's recommended to remove existed module usb out of kernel:
"echo 4a100000.switch | sudo tee /sys/bus/platform/drivers/cpsw-switch/unbind"
