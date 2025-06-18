#define _XTAL_FREQ 20000000  // define the system clock frequency as 20 mhz for delay functions

#include <xc.h>              // include xc8 standard header for pic microcontrollers
#include <stdint.h>          // include standard integer types
#include <stdio.h>           // include standard input/output library for sprintf

// configuration bits for the pic16f877a microcontroller
#pragma config FOSC = HS              // use high-speed external oscillator
#pragma config WDTE = OFF             // watchdog timer disabled
#pragma config PWRTE = OFF            // power-up timer disabled
#pragma config BOREN = ON             // brown-out reset enabled
#pragma config LVP = OFF              // low-voltage programming disabled
#pragma config CPD = OFF              // data memory code protection disabled
#pragma config WRT = OFF              // program memory write protection off
#pragma config CP = OFF               // code protection disabled

// lcd pin definitions
#define RS      RD0                  // register select pin
#define EN      RD3                  // enable pin
#define D4      RD4                  // data pin 4
#define D5      RD5                  // data pin 5
#define D6      RD6                  // data pin 6
#define D7      RD7                  // data pin 7

#define RS_dir  TRISD0               // direction register for rs
#define EN_dir  TRISD3               // direction register for en
#define D4_dir  TRISD4               // direction register for d4
#define D5_dir  TRISD5               // direction register for d5
#define D6_dir  TRISD6               // direction register for d6
#define D7_dir  TRISD7               // direction register for d7

// output devices
#define BUZZER        RC0           // buzzer connected to rc0
#define LED           RC1           // indicator led on rc1
#define EMERGENCY_LED RC4           // emergency led on rc4
#define LIGHT_LED     RC5           // light-dependent led on rc5

#define BUZZER_dir    TRISC0        // direction for buzzer pin
#define LED_dir       TRISC1        // direction for led pin
#define EMERGENCY_LED_dir TRISC4    // direction for emergency led pin
#define LIGHT_LED_dir     TRISC5    // direction for light led pin

// ultrasonic sensor connections
#define TRIG     RB0                // trigger pin connected to rb0
#define ECHO     RB4                // echo pin connected to rb4
#define TRIG_dir TRISB0             // direction register for trigger
#define ECHO_dir TRISB4             // direction register for echo

#define ALERT_DISTANCE 10           // distance threshold to trigger alert
#define PWM_FREQ     50             // pwm frequency for servo
#define PWM_MIN      5              // minimum pwm duty cycle
#define PWM_MAX      25             // maximum pwm duty cycle

volatile uint16_t distance = 0;     // global variable to hold measured distance
volatile uint8_t emergency_mode = 0; // flag for emergency mode state
char display_text[20];              // buffer for lcd text display

// function declarations
void LCD_Init(void);                       // initialize lcd
void LCD_Command(uint8_t);                // send command to lcd
void LCD_Data(uint8_t);                   // send data to lcd
void LCD_String(uint8_t, uint8_t, const char *); // display string on lcd
void LCD_PulseEnable(void);               // enable pulse for lcd

void SR04_Init(void);                     // initialize ultrasonic sensor
uint16_t SR04_GetDistance(void);          // get distance from ultrasonic

void PWM1_Init(uint16_t);                 // initialize pwm for servo
void PWM1_SetDuty(uint8_t);               // set pwm duty cycle
void Servo_SetAngle(uint8_t);             // rotate servo based on angle
void Buzzer_LED_Control(uint16_t);        // control buzzer and led alerts
uint16_t ADC_Read(uint8_t);               // read analog value from adc

void main(void) {
    // setup input pins
    TRISB5 = 1;           // set rb5 as input for emergency button
    TRISA0 = 1;           // set ra0 as input for ldr sensor
    OPTION_REGbits.nRBPU = 0; // enable portb pull-ups

    // setup adc
    ADCON1 = 0x80;        // left justify result, vref = vdd, an0 analog
    ADCON0 = 0x01;        // enable adc and select channel an0

    // initialize components
    SR04_Init();          // setup ultrasonic pins
    LCD_Init();           // setup lcd display
    PWM1_Init(PWM_FREQ);  // start pwm with given frequency

    // set output directions
    BUZZER_dir = 0;
    LED_dir = 0;
    EMERGENCY_LED_dir = 0;
    LIGHT_LED_dir = 0;

    // enable interrupt on portb change
    INTCONbits.RBIE = 1;  // enable rb interrupt
    INTCONbits.RBIF = 0;  // clear interrupt flag
    INTCONbits.GIE = 1;   // enable global interrupt

    // initial lcd message
    LCD_String(0, 0, "System Ready");
    __delay_ms(2000);
    LCD_Command(0x01);    // clear lcd

    while (1) {
        if (emergency_mode) {
            // emergency mode active
            LCD_Command(0x01);
            LCD_String(0, 0, "EMERGENCY MODE!");
            PWM1_SetDuty(0);             // stop servo
            Buzzer_LED_Control(0);       // activate emergency buzzer pattern
            __delay_ms(200);
            continue;
        }

        distance = SR04_GetDistance();   // get current distance from sensor
        uint16_t ldr_value = ADC_Read(0);  // read ldr analog value
        uint8_t ldr_percent = 100 - ((ldr_value * 100) / 1023); // convert to inverted %

        // control light led based on ambient light
        if (ldr_percent < 50)
            LIGHT_LED = 1;
        else
            LIGHT_LED = 0;

        // update lcd with distance
        LCD_Command(0x01);
        sprintf(display_text, "Dist: %3u cm", distance);
        LCD_String(0, 0, display_text);

        // update lcd with light level
        sprintf(display_text, "Light: %3u%%", ldr_percent);
        LCD_String(1, 0, display_text);

        Servo_SetAngle(distance);        // adjust servo based on distance
        Buzzer_LED_Control(distance);    // update buzzer and leds

        __delay_ms(200);                 // delay for sensor stability
    }
}

void __interrupt() ISR(void) {
    static uint8_t last_button = 1;

    if (RBIF) {                          // check portb change
        if (PORTBbits.RB5 == 0 && last_button == 1) {
            __delay_ms(50);             // debounce delay
            if (PORTBbits.RB5 == 0) {
                emergency_mode = !emergency_mode; // toggle emergency mode
            }
        }
        last_button = PORTBbits.RB5;    // update last state
        RBIF = 0;                        // clear interrupt flag
    }
}

void LCD_Init() {
    RS_dir = EN_dir = D4_dir = D5_dir = D6_dir = D7_dir = 0; // all lcd pins as output
    __delay_ms(50);
    LCD_Command(0x33); // lcd initialization sequence
    LCD_Command(0x32);
    LCD_Command(0x28); // 4-bit, 2 line, 5x8 font
    LCD_Command(0x0C); // display on, cursor off
    LCD_Command(0x01); // clear display
    LCD_Command(0x06); // entry mode set
}

void LCD_Command(uint8_t cmd) {
    RS = 0;                      // command mode
    D4 = (cmd >> 4) & 1; D5 = (cmd >> 5) & 1;
    D6 = (cmd >> 6) & 1; D7 = (cmd >> 7) & 1;
    LCD_PulseEnable();           // send high nibble
    D4 = (cmd >> 0) & 1; D5 = (cmd >> 1) & 1;
    D6 = (cmd >> 2) & 1; D7 = (cmd >> 3) & 1;
    LCD_PulseEnable();           // send low nibble
    if (cmd == 0x01) __delay_ms(2); // wait if clear command
}

void LCD_Data(uint8_t data) {
    RS = 1;                      // data mode
    D4 = (data >> 4) & 1; D5 = (data >> 5) & 1;
    D6 = (data >> 6) & 1; D7 = (data >> 7) & 1;
    LCD_PulseEnable();           // send high nibble
    D4 = (data >> 0) & 1; D5 = (data >> 1) & 1;
    D6 = (data >> 2) & 1; D7 = (data >> 3) & 1;
    LCD_PulseEnable();           // send low nibble
    __delay_us(50);
}

void LCD_String(uint8_t row, uint8_t col, const char *text) {
    uint8_t address = (row == 0) ? 0x80 : 0xC0; // set starting position
    LCD_Command(address + col);
    while (*text) LCD_Data(*text++);  // display each character
}

void LCD_PulseEnable() {
    EN = 1; __delay_us(1);       // pulse enable high
    EN = 0; __delay_us(100);     // pulse enable low
}

void SR04_Init() {
    TRIG_dir = 0;                // trigger is output
    ECHO_dir = 1;                // echo is input
    T1CON = 0x00;                // timer1 off
}

uint16_t SR04_GetDistance(void) {
    uint16_t echo_time = 0;
    uint16_t timeout = 60000;

    TRIG = 1; __delay_us(10); TRIG = 0; // send 10us pulse

    while (!ECHO && timeout--) __delay_us(1); // wait for echo start
    if (timeout == 0) return 999;             // timeout error

    TMR1H = TMR1L = 0;         // clear timer1
    T1CONbits.TMR1ON = 1;      // start timer

    timeout = 60000;
    while (ECHO && timeout--) __delay_us(1); // wait for echo end
    T1CONbits.TMR1ON = 0;      // stop timer

    if (timeout == 0) return 999; // timeout error

    echo_time = (TMR1H << 8) | TMR1L;  // read timer value
    return (uint16_t)((echo_time / 117)*2); // convert to cm (calibrated)
}

void PWM1_Init(uint16_t freq) {
    PR2 = (_XTAL_FREQ / (4UL * 4 * freq)) - 1; // calculate pr2 for frequency
    T2CON = 0b00000101; // timer2 on, prescaler 1:4
    CCP1CON = 0b00001100; // pwm mode
    CCPR1L = 0; // initial duty
}

void PWM1_SetDuty(uint8_t duty) {
    CCPR1L = duty >> 2; // set high bits
    CCP1CON = (CCP1CON & 0xCF) | ((duty & 0x03) << 4); // set low bits
}

void Servo_SetAngle(uint8_t angle) {
    static uint8_t current = PWM_MIN;
    uint8_t target = PWM_MIN + ((angle * (PWM_MAX - PWM_MIN)) / 180); // map angle to pwm
    if (current < target) current++;
    else if (current > target) current--;
    PWM1_SetDuty(current); // set duty
}

void Buzzer_LED_Control(uint16_t dist) {
    static uint8_t led_state = 0;
    static uint16_t buzz_counter = 0;

    if (emergency_mode) {
        EMERGENCY_LED = 1;
        LED = 0;
        buzz_counter++;
        if (buzz_counter < 200) BUZZER = 1;
        else if (buzz_counter < 400) BUZZER = 0;
        else if (buzz_counter < 600) BUZZER = 1;
        else {
            BUZZER = 0;
            if (buzz_counter >= 1000) buzz_counter = 0;
        }
    } else {
        EMERGENCY_LED = 0;
        if (dist <= ALERT_DISTANCE) {
            BUZZER = 1;
            led_state = !led_state;
            LED = led_state;
        } else {
            BUZZER = 0;
            LED = 0;
        }
    }
}

uint16_t ADC_Read(uint8_t channel) {
    ADCON0 = (channel << 3) | 0x01; // select channel and turn on adc
    __delay_ms(2);                 // wait for acquisition
    GO_nDONE = 1;                  // start conversion
    while (GO_nDONE);             // wait until done
    return ((ADRESH << 8) | ADRESL); // combine result
}
