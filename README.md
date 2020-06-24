LED strip controller for RC models based on Microchip PIC12F1822

based on work from https://github.com/DzikuVx/drone_led_strip_controller

Although some flight controller software, like Cleanflight (and hardware) allows to attach WS2812B based RGB LED strips, it is far from perfect. For example, LED Strip support might disable SoftSerial or Sonar support depending on used hardware. And if you want add some bling to drone or other RC model not running Cleanflight, you are on your own.

It is small, independent, WS2812B LED strips controller designed to fit on 250 class quadcopters or other RC models.

Features:

Weight: less than 5g
5V input, can be taken directly from BEC
Supports 8 RGB LED strips based on WS2812B
1 button operated (or RC channel input), both can be used at the same time (different pins).

Single button allows to change pattern and color. Short press changes patterns, long press changes color.

Patterns:

- All LEDs OFF
- All LEDs ON, constant color and brightness
- SingleWander : This will turn on one single led in the strip in sequence
- Rapid : this will turn on 3 consecutive leds in the strip in sequence
- BoldWander : this will turn on 2 consecutive leds in the strip in sequence
- BolderWander : this will turn on 4 consecutive leds in the strip in sequence
- DoubleWander : this will turn on 2 opposite leds in the strip in sequence
- Chase : this will turn on 2 leds (with gap in between) in the strip in sequence
- Flash : this will flash all leds in the strip
- Breathing : this will progressively turn ON and OFF (breathing effect) all leds in the strip
- Rainbow : this will display a rainbow effect on the led strip

Colors:

- RED
- GREEN
- BLUE
- PURPLE
- YELLOW
- CYAN
- WHITE

Speed of animation can be changed using the RC channel input (min speed = min RC pulse of 800ms, max speed = RC pulse of 1500 ms
Above 1750ms RC pulse will emulate a button press, to change pattern and/or color.

Last pattern and color used are saved in EEPROM and will be restored at next power up.

