# BluebaruPhoneKeyFw
Firmware for an automatic RSSI-based smartphone key (a la Tesla) for cars with remote fobs. Built for the Adafruit Feather nRF52832 dev kit which has built in battery charging circuitry so you can plug it into a USB port in your car and it will charge while it is running.

To get it working, you'll need to open up a donor key fob and solder wires to lock and unlock buttons. I used a breadboard breakout to wire up a few MOSFETs so that the GPIOs on the nRF52 can control the remote buttons.

## Design
Note: the UART protocol is for the BLE data channel
![Firmware Design](https://github.com/corymayer/BluebaruPhoneKeyFw/raw/master/Phone%20Key%20FW%20Design.png?raw=true)
