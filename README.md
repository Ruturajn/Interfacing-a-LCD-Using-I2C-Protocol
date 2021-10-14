# Interfacing-a-LCD-Using-I2C-Protocol

![image](https://user-images.githubusercontent.com/56625259/137286905-bc60f71c-6f31-4001-8794-075833581c21.png)

This projects aims to interface a Liquid Crystal Display with the STM32F407 micrcontroller in 4-bit mode using the I2C protocol. The I2C module connected to the back of the LCD
is named PC58574T. It has 4 Input Pins namely VCC, GND, SCL and SDA. The output pins available on this module are P0-P7. The connection between the LCD and the Output Pins of
the PC58574T are made as follows:

| PC58574T | LCD |
|---|---|
| P0 | RS |
| P1 | R/W |
| P2 | EN |
| P3 | Back Light |
| P4 | D4 |
| P5 | D5 |
| P6 | D6 |
| P7 | D7 |

To connect this LCD module to the STM32F407 Discovery Board:
| LCD | STM32F4-DISC |
|---|---|
| VCC | 5V |
| GND | GND |
| SCL | PB6 |
| SDA | PB7 |
