/****************************************************************************************************************
 * PIC12F1822 WS2812b RC Led Strip Project

- Version 1.0
- 2 April 2020
- Project settings are defined in the system.h file
- based on work from https://github.com/DzikuVx/drone_led_strip_controller

 Copyright 2020 Patrick Tissot
 This program is free software: you can redistribute it and/or modify it under the terms of the GNU General
 Public License as published by the Free Software Foundation, either version 2 of the License, or (at your
 option) any later version.

 This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
 implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 for more details.  You should have received a copy of the GNU General Public License along with this program.
 If not, see <http://www.gnu.org/licenses/>.

 Development Environment
       To compile this you need:
        - XC8 2.20 PRO or higher (www.microchip.com) with -s optimization for size
        - Microchip MPLAB X IDE Version 5.35 or higher (www.microchip.com)
 ***************************************************************************************************************/

// Global includes
#include <xc.h>
#include <stdint.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = ON        // Watchdog Timer Enable (WDT enabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), high trip point selected.)
#pragma config LVP = ON         // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

#define _XTAL_FREQ          32000000UL


/*
 *  Structure of the LED array
 *
 * cRGB:     RGB  for WS2812S/B/C/D, SK6812, SK6812Mini, SK6812WWA, APA104, APA106
 * cRGBW:    RGBW for SK6812RGBW
 */
struct cRGB  { uint8_t g; uint8_t r; uint8_t b; };

// Change this setting based on your LED strip length (be aware of RAM size limitation!)
#define NUM_PIXELS          8

// Assembly function used to send the 24bits info to WS2812 leds
void send_frame(void);

// System definitions
#define FCY                 8000000UL
#define TMR0_PRESCALER      256
// 32 MHz / 4 = 8 MHz => 0.125 µs instruction cycle

#define TICK                1       // 1ms interrupt Tick rate
#define COUNT               (FCY/1000000UL*TICK*1000/TMR0_PRESCALER)
#if (COUNT > 255)
#error "Change TMR0 prescaler or FCY"
#endif
#define TMR0_RELOAD         (255-COUNT)

// Hardware mapping definitions
// LED PIN is fixed to RA2 (see Send_Frame.asm)
#define INPUT_PIN           RA5     // connected to PWM output from RC receiver or left floating
#define LED_MULTIPLIER      4
#define BUTTON_PIN          RA4     // connected to push button (to ground), uses internal pull-up
#define SLEEP_TIME          40
#define SPEED_MAX           20
#define SPEED_MID           10
#define COLORS              7
#define LONG_PUSH_THRESHOLD 20
#define MAX_CYCLE           255
#define MODES               11
#define PWM_PRESS_MIN       1750    // pulse 1750ms mini to detect "button" press through RC
#define PWM_PRESS_MAX       2200    // pulse 2200ms max to detect "button" press through RC
#define PWM_SPEED_MIN       800     // pulse 800ms maps to minimum speed (led pattern change) through RC
#define PWM_SPEED_MAX       1500    // pulse 1500ms maps to maximum speed (led pattern change) through RC

// EEPROM parameters
#define EEPROM_MODE_ADDRESS     0
#define EEPROM_COLOR_ADDRESS    1
#define EEPROM_SPEED_ADDRESS    2

// State machine used for button press decoding
#define IDLE                0
#define PRESS               1
#define SHORTPRESS          2
#define LONGPRESS           3

// COLOR patterns (can be adjusted if necessary)
const uint8_t colors[COLORS][3] = {
    {1, 0, 0}, // RED
    {0, 1, 0}, // GREEN
    {0, 0, 1}, // BLUE
    {1, 0, 1}, // PURPLE
    {1, 1, 0}, // YELLOW
    {0, 1, 1}, // CYAN
    {1, 1, 1} // WHITE
};

// Globals

volatile uint8_t COLOR;
volatile uint8_t count;

struct cRGB currentColor;
struct cRGB off;
struct cRGB background;
struct cRGB output[NUM_PIXELS] __at(0x40); // force LED array to be at fixed RAM address (for assembly routine taking care of LED refresh)
uint8_t my_brightness = 250; // this can be changed to accomodate lower brightness level 

uint8_t previous_mode;
uint8_t mode = 0;
uint8_t pushStartCycle;
uint8_t i;
uint8_t previous_colorIndex;
uint8_t colorIndex = 0;
uint8_t cycle = 0;
uint8_t currentCycle;
uint16_t previous_speed;
uint16_t speed;

uint8_t controllState;

// these variables will be accessed under interrupt, need to be declared as volatile
volatile uint16_t risingStart = 0;
volatile uint16_t channelLength = 0;
volatile uint8_t timer_1ms;


// running average of data (using 20% new value / 80% old value)
uint16_t smooth(uint16_t data, uint16_t smoothedVal) {
    smoothedVal = ((data * 2) / 10) + (smoothedVal * 8) / 10;

    return smoothedVal;
}

// Interrupt Service Request handler
void __interrupt() isr(void) {
    // Is this interrupt on change pin interrupting?
    if (IOCIE && IOCIF) {
        IOCIF = 0;
        IOCAF = 0;
        TMR1ON = 0;
        if (INPUT_PIN) { // rising edge detected
            risingStart = TMR1;
        } else { // falling edge detected
            uint16_t val;
            if (TMR1 > risingStart)
                val = TMR1 - risingStart;
            else
                val = (65535U - risingStart) + TMR1;
            // val now contains PPM pulse duration 
            channelLength = smooth(val, channelLength);
            // to get channel value, perform some filtering
        }
        TMR1ON = 1;
    }
    
    // Is this timer1 interrupting?
    if (TMR0IE && TMR0IF) {
        // Reset the timer1 interrupt flag
        TMR0IF = 0;
        TMR0 = TMR0_RELOAD; // reload timer 0 for next TICK interrupt period
        LATAbits.LATA1 ^= 1; // for debug and interrupt monitoring
        timer_1ms = 1;
    }
}

// Init System timer (TMR0) used by Application for TICK generation
void Init_Timer(void) {
    // Configure Timer 0 for system timer
    TMR0IF = 0; // Clean TMR0 flag
    T0CS = 0; // clock source = internal system clock

    // prescaler
    PSA = 0; // prescaler assigned to TMR0
    OPTION_REGbits.PS = 0b111; // Prescaler 1:256
    TMR0 = TMR0_RELOAD; // reload value = TICK ms

    // Enable System Timer Interrupts
    TMR0IE = 1; // Enable TMR0 Interrupt
}

// Configure Timer 2 for Delay routine
void Init_Timer2(void) {
    T2CON = 0b00110000; // prescaler 1/8 gives 1 us resolution
}

// Duration is a multiple of 1 us
// this routine will block until the time is ellapsed (only interrupts are allowed)
void delay_us(uint8_t duration) {
    if (duration > 5)
        PR2 = duration - 4; // compensate for function overhead
    else
        PR2 = 1;
    TMR2 = 0; // reset Timer2
    TMR2IF = 0;
    TMR2ON = 1; // start Timer2
    while (!TMR2IF)
        NOP();
    TMR2ON = 0; // shutdown Timer2
}

// Duration is a multiple of 1 ms
void delay_ms(uint16_t duration) {
    uint16_t j;
    for (j = 0; j < (duration * 10); j++) {
        delay_us(100);
        CLRWDT();
    }
}

// Routine to write a single byte into EEPROM space
void DATAEE_WriteByte(uint8_t bAdd, uint8_t bData) {
    uint8_t GIEBitValue = 0;

    EEADRL = bAdd; // Data Memory Address to write
    EEDATL = bData; // Data Memory Value to write
    EECON1bits.EEPGD = 0; // Point to DATA memory
    EECON1bits.CFGS = 0; // Deselect Configuration space
    EECON1bits.WREN = 1; // Enable writes

    GIEBitValue = INTCONbits.GIE;
    INTCONbits.GIE = 0; // Disable INTs
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1; // Set WR bit to begin write
    // Wait for write to complete
    while (EECON1bits.WR) {
    }

    EECON1bits.WREN = 0; // Disable writes
    INTCONbits.GIE = GIEBitValue;
}

// Routine to read a single byte from EEPROM space
uint8_t DATAEE_ReadByte(uint8_t bAdd) {
    EEADRL = bAdd; // Data Memory Address to read
    EECON1bits.CFGS = 0; // Deselect Configuration space
    EECON1bits.EEPGD = 0; // Point to DATA memory
    EECON1bits.RD = 1; // EE Read
    NOP(); // NOPs may be required for latency at high frequencies
    NOP();

    return (EEDATL);
}

// Initialize I/O ports
void Init_Ports(void) {
    nWPUEN = 0; // PortA pull-ups enabled by individual PORT Latch values
    ANSELA = 0; // all pins digital
    WPUA = 0b00111000; // disable Weak pull-ups on outputs
    LATA = 0; // this will turn ON debug LED
    TRISA = 0b00111000; // Output bits RA0, RA2
    IOCAP = 0b00100000; // RA5 triggers interrupt for both rising and falling changes
    IOCAN = 0b00100000;
    IOCIF = 0;
    IOCIE = 1;
}

//
void setColor(void) {
    currentColor.r = colors[colorIndex][0] * my_brightness;
    currentColor.g = colors[colorIndex][1] * my_brightness;
    currentColor.b = colors[colorIndex][2] * my_brightness;

    background.r = colors[colorIndex][0] * 50;
    background.g = colors[colorIndex][1] * 50;
    background.b = colors[colorIndex][2] * 50;
}

// This will turn off all LEDs on the strip
void modeOff(void) {
    for (i = 0; i < NUM_PIXELS; i++) {
        output[i] = off;
    }
}

// This will turn on all LEDs on the strip (with currentcolor setting)
void modeOn(void) {
    for (i = 0; i < NUM_PIXELS; i++) {
        output[i] = currentColor;
    }
}

// returns 
uint8_t getDivider(uint8_t denom, uint8_t cycle) {
    return cycle / denom;
}

// This will turn on one single led in the strip in sequence
void modeSingleWander(void) {
    currentCycle = getDivider(2, currentCycle);

    for (i = 0; i < NUM_PIXELS; i++) {
        output[i] = background;
    }

    output[currentCycle % NUM_PIXELS] = currentColor;
}

// this will turn on 3 consecutive leds in the strip in sequence
void modeRapid(void) {
    for (i = 0; i < NUM_PIXELS; i++) {
        output[i] = background;
    }

    if (currentCycle == NUM_PIXELS << 2) {
        cycle = 0;
        currentCycle = 0;
    }

    if (currentCycle < NUM_PIXELS) {
        output[currentCycle % NUM_PIXELS] = currentColor;

        if (currentCycle + 1 < NUM_PIXELS) {
            output[(currentCycle + 1) % NUM_PIXELS] = currentColor;
        }
        if (currentCycle + 2 < NUM_PIXELS) {
            output[(currentCycle + 2) % NUM_PIXELS] = currentColor;
        }
    }
}

// this will turn on 2 consecutive leds in the strip in sequence
void modeBoldWander(void) {
    currentCycle = getDivider(2, currentCycle);

    for (i = 0; i < NUM_PIXELS; i++) {
        output[i] = background;
    }

    output[currentCycle % NUM_PIXELS] = currentColor;
    output[(currentCycle + 1) % NUM_PIXELS] = currentColor;
}

// this will turn on 4 consecutive leds in the strip in sequence
void modeBolderWander(void) {
    currentCycle = getDivider(2, currentCycle);

    for (i = 0; i < NUM_PIXELS; i++) {
        output[i] = off;
    }

    output[currentCycle % NUM_PIXELS] = currentColor;
    output[(currentCycle + 1) % NUM_PIXELS] = currentColor;
    output[(currentCycle + 2) % NUM_PIXELS] = currentColor;
    output[(currentCycle + 3) % NUM_PIXELS] = currentColor;
}

// this will turn on 2 opposite leds in the strip in sequence
void modeDoubleWander(void) {
    currentCycle = getDivider(2, currentCycle);

    for (i = 0; i < NUM_PIXELS; i++) {
        output[i] = off;
    }

    output[currentCycle % NUM_PIXELS] = currentColor;
    output[(MAX_CYCLE - currentCycle) % NUM_PIXELS] = currentColor;
}

// this will turn on 2 leds (with gap in between) in the strip in sequence
void modeChase(void) {
    currentCycle = getDivider(2, currentCycle);

    for (i = 0; i < NUM_PIXELS; i++) {
        output[i] = off;
    }

    output[currentCycle % NUM_PIXELS] = currentColor;
    output[(currentCycle + (NUM_PIXELS / 2)) % NUM_PIXELS] = currentColor;
}

// this will flash all leds in the strip
void modeFlash(void) {
    currentCycle = getDivider(2, currentCycle);

    struct cRGB state = off;

    if ((currentCycle % 16 == 12) || (currentCycle % 16 == 15)) {
        state = currentColor;
    }

    for (i = 0; i < NUM_PIXELS; i++) {
        output[i] = state;
    }
}

// this will progressively turn ON and OFF (breathing effect) all leds in the strip
void modeBreathing(void) {
    struct cRGB my_color;
    uint16_t value;
    currentCycle = currentCycle & 0x3F;
    if (currentCycle > 31)
        currentCycle = 63 - currentCycle;
    value = (uint16_t)(currentCycle * currentCycle) >> 2;
    currentCycle = value;
    my_color.r = colors[colorIndex][0] * currentCycle;
    my_color.g = colors[colorIndex][1] * currentCycle;
    my_color.b = colors[colorIndex][2] * currentCycle;
    for (i = 0; i < NUM_PIXELS; i++) {
        output[i] = my_color;
    }
}

// this will display a rainbow effect on the led strip
void modeRainbow(void) {
    static uint8_t offset;
    struct cRGB my_color;
    uint8_t index;

    for (i = 0; i < NUM_PIXELS; i++) {
        index = (i + offset) & 0x07;
        my_color.r = colors[index][0] * 85;
        my_color.g = colors[index][1] * 85;
        my_color.b = colors[index][2] * 85;
        output[i] = my_color;
    }
    if (currentCycle % 5 == 0)
        offset++;
}

uint8_t abs_u8(int8_t value) {
    if (value < 0)
        return (-value);
    else
        return (value);
}

// do actual work
void loop(void) {
    // Test RC input pulse duration
    
    // first, compare pulse to speed control range
    if ((channelLength > PWM_SPEED_MIN) && (channelLength < PWM_SPEED_MAX)) {
        speed = ((channelLength - PWM_SPEED_MIN) * SPEED_MAX) / (PWM_SPEED_MAX - PWM_SPEED_MIN);
    }

    // then compare pulse to "button" press threshold (or actual button press)
    if (((channelLength > PWM_PRESS_MIN) && (channelLength < PWM_PRESS_MAX)) || (BUTTON_PIN == 0)) {
        if (controllState == IDLE) {
            controllState = PRESS;
            pushStartCycle = cycle;
        }
    }

    // decide button status based on previous state
    if ((channelLength < 1750) && (BUTTON_PIN == 1)) {
        if (controllState == PRESS) {
            controllState = SHORTPRESS;
        }
        if (controllState == LONGPRESS) {
            controllState = IDLE;
        }
    }

    // if button is pressed
    if (controllState == PRESS) {
        // and this is a long press
        if (abs_u8(cycle - pushStartCycle) > LONG_PUSH_THRESHOLD) {
            // Change color procedure
            colorIndex++;

            if (colorIndex == COLORS) {
                colorIndex = 0;
            }
            
            setColor();
            cycle = 0;
            pushStartCycle = 0;
            controllState = LONGPRESS;
        }
    }

    // if button is short pressed
    if (controllState == SHORTPRESS) {
        if (abs_u8(cycle - pushStartCycle) > 2) {
            // Change mode procedure
            mode++;

            if (mode == MODES) {
                mode = 0;
            }
            
            cycle = 0;
            pushStartCycle = 0;
            controllState = IDLE;
        }
    }
    
    // memorize currentcycle
    currentCycle = cycle;
    
    // based on selected operating mode, dispatch to appropriate LED display routine
    switch (mode) {
        case 0:
            modeOn();
            break;

        case 1:
            modeSingleWander();
            break;

        case 2:
            modeBoldWander();
            break;

        case 3:
            modeDoubleWander();
            break;

        case 4:
            modeChase();
            break;

        case 5:
            modeFlash();
            break;

        case 6:
            modeBolderWander();
            break;

        case 7:
            modeRapid();
            break;

        case 8:
            modeBreathing();
            break;

        case 9:
            modeRainbow();
            break;

        case 10:
            modeOff();
            break;

    }
    
    // Update LEDs
    send_frame();
    cycle++;
    
    // wait for next frame to display
    delay_ms(SLEEP_TIME - speed);
}

// Set up the timer for input capture
void Init_Timer1(void) {
    T1CON = 0b00110001; // Timer1 clock source is instruction clock (FOSC/4)
    // 1:8 Prescale value (1us timebase), Timer 1 ON
}

// Main Rountine (the application)
//DOM-IGNORE-BEGIN
//                    _          __ __
//    _ __ ___   __ _(_)_ __    / / \ \
//   | '_ ` _ \ / _` | | '_ \  | |   | |
//   | | | | | | (_| | | | | | | |   | |
//   |_| |_| |_|\__,_|_|_| |_| | |   | |
//                              \_\ /_/

void main(void) {
    // Set up the IO pins
    Init_Ports();
    OSCCON = 0b01110000; //Setup internal oscillator for 32MHz = 8M x 4

    Init_Timer2();
    Init_Timer1();
    
    // try to restore previous operating mode from EEPROM
    mode = DATAEE_ReadByte(EEPROM_MODE_ADDRESS);
    if (mode >= MODES) {
        mode = 0;
    }
    previous_mode = mode;

    // try to restore previous color mode from EEPROM
    colorIndex = DATAEE_ReadByte(EEPROM_COLOR_ADDRESS);
    if (colorIndex >= COLORS) {
        colorIndex = 0;
    }
    previous_colorIndex = colorIndex;

    // try to restore previous speed setting from EEPROM
    speed = DATAEE_ReadByte(EEPROM_SPEED_ADDRESS);
    if (speed > SPEED_MAX) {
        speed = SPEED_MID;
    }
    previous_speed = speed;

    // prepare LED strip
    setColor();
    controllState = IDLE;

    Init_Timer();

    // Enable the interrupts
    PEIE = 1;
    GIE = 1;

    // Program Main loop
    while (1) {
        // If 1ms timer expired, then launch loop
        if (timer_1ms) {
            timer_1ms = 0;
            loop();
        }
        
        // is it time to save current settings to EEPROM?
        if (cycle == 255) {
            // if mode changed since last save, then store in EEPROM
            if (mode != previous_mode) {
                previous_mode = mode;
                DATAEE_WriteByte(EEPROM_MODE_ADDRESS, mode);
            }
            // if color changed since last save, then store in EEPROM
            if (colorIndex != previous_colorIndex) {
                previous_colorIndex = colorIndex;
                DATAEE_WriteByte(EEPROM_COLOR_ADDRESS, colorIndex);
            }
            // if speed changed since last save, then store in EEPROM
            if (speed != previous_speed) {
                previous_speed = speed;
                DATAEE_WriteByte(EEPROM_SPEED_ADDRESS, speed);
            }
        }
    }
}
