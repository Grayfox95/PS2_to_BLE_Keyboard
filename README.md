# ESP-IDF PS/2 to BLE HID Keyboard converter
by Antonio Perra 07.04.2025

## About
This project implements a PS/2 to BT HID converter for ESP32 to renovate old PS/2 Keyboards.

The development is based on the [ESP-IDF BLE HID Example](https://github.com/espressif/esp-idf/tree/a25e7ab59ed197817d4a78e139220b2707481f67/examples/bluetooth/bluedroid/ble/ble_hid_device_demo) from which I wrote the PS/2 library (for interfacing with the PS/2 protocol) and the PS2 to HID convertion.

The entire application has been developed and validated with a Model SK-2500 keyboard. 
However, the PS/2 is a standard protocol and the same application should work with all the keyboards supporting PS/2 Set 2.

### Features
The project provides the following features:

1. Map between all the standard keys.
2. Map of multimedia keys.
3. Volume control.
4. Special functionality keys.
5. Low power mode.
6. LED control
7. Coffee button menagement. 

### PS/2 Library
The PS/2 library uses two GPIOs of the ESP32 to interact with CLK and DATA signals of the PS/2 keyboard.
This library allows to:

1. Receive and analyze the MAKE and BREAK codes from the keyboard.
2. Send codes to the keyboard.
3. Parity check and retransmission in case of wrong code.
4. Keyboard's LED management.

## How to use it
The ESP must be connected to the CLK and DATA PS/2 keyboard signals.
The current configuration consists of:

- PS/2 CLK --> GPIO 4
- PS/2 DATA --> GPIO 5

Moreover, a GPIO has been dedicated to the Coffee LED:

- Coffee LED --> GPIO 0

This configuration can be modified acting on the local defines of ps2_dev.c file.

## Low power mode
The ESP32 enters in deep sleep low power mode after 2 minutes of inactivity. 
It is resumed as soon as the CLK signal moves.

## Coffee button
When the coffee button has been pressed a specific routine is executed.
It consists of sending a HID char to host every 60 seconds.

## BUGs and improvements 


### Missing features
- [ ] Improve power consumption by porting to a single core application.
- [ ] Develop battery level measurement.
- [ ] Develop control to switch between multiple bluetooth devices.

### Known problems
- [] '<' and '>' keys are not printed.


