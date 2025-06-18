#define _XTAL_FREQ 20000000
 
#include <xc.h>
#include <stdint.h>
#include <stdio.h>
 
// CONFIG
#pragma config FOSC = HS
#pragma config WDTE = OFF, PWRTE = OFF, BOREN = ON
#pragma config LVP = OFF, CPD = OFF, WRT = OFF, CP = OFF
 
// LCD Pins
#define RS      RD0
#define EN      RD3
#define D4      RD4
#define D5      RD5
#define D6      RD6
#define D7      RD7
 
#define RS_dir  TRISD0
#define EN_dir  TRISD3
#define D4_dir  TRISD4
#define D5_dir  TRISD5
#define D6_dir  TRISD6
#define D7_dir  TRISD7
 
// Buzzer and LEDs
#define BUZZER        RC0
#define LED           RC1
#define EMERGENCY_LED RC4
#define LIGHT_LED     RC5
 
#define BUZZER_dir    TRISC0
#define LED_dir       TRISC1
#define EMERGENCY_LED_dir TRISC4
#define LIGHT_LED_dir     TRISC5
 
// Ultrasonic
#define TRIG     RB0
#define ECHO     RB4
#define TRIG_dir TRISB0
#define ECHO_dir TRISB4
 
#define ALERT_DISTANCE 10
#define PWM_FREQ     50
#define PWM_MIN      5
#define PWM_MAX      25
 
volatile uint16_t distance = 0;
volatile uint8_t emergency_mode = 0;
char display_text[20];
 
// Prototypes
void LCD_Init(void);
void LCD_Command(uint8_t);
void LCD_Data(uint8_t);
void LCD_String(uint8_t row, uint8_t col, const char *text);
void LCD_PulseEnable(void);
void SR04_Init(void);
uint16_t SR04_GetDistance(void);
void PWM1_Init(uint16_t freq);
void PWM1_SetDuty(uint8_t duty);
void Servo_SetAngle(uint8_t angle);
void Buzzer_LED_Control(uint16_t dist);
uint16_t ADC_Read(uint8_t channel);
 
void main(void) {
    // Setup
    TRISB5 = 1;   // Emergency button
    TRISA0 = 1;   // LDR input (AN0)
    OPTION_REGbits.nRBPU = 0;
 
    // ADC Setup
    ADCON1 = 0x80; // Left justified, Vref = VDD, AN0 analog
    ADCON0 = 0x01; // ADC ON, AN0 selected
 
    // IO Directions
    SR04_Init();
    LCD_Init();
    PWM1_Init(PWM_FREQ);
 
    BUZZER_dir = 0;
    LED_dir = 0;
    EMERGENCY_LED_dir = 0;
    LIGHT_LED_dir = 0;
 
    INTCONbits.RBIE = 1;
    INTCONbits.RBIF = 0;
    INTCONbits.GIE = 1;
 
    LCD_String(0, 0, "System Ready");
    __delay_ms(2000);
    LCD_Command(0x01);
 
    while (1) {
        if (emergency_mode) {
            LCD_Command(0x01);
            LCD_String(0, 0, "EMERGENCY MODE!");
            PWM1_SetDuty(0);
            Buzzer_LED_Control(0);
            __delay_ms(200);
            continue;
        }
 
        // Distance
        distance = SR04_GetDistance();
 
        // LDR reading and percentage
        uint16_t ldr_value = ADC_Read(0);  // RA0/AN0
        uint8_t ldr_percent = 100 - ((ldr_value * 100) / 1023); // Inverted
 
        // Light-based LED
        if (ldr_percent < 50)
            LIGHT_LED = 1;
        else
            LIGHT_LED = 0;
 
        // LCD Display
        LCD_Command(0x01);
        sprintf(display_text, "Dist: %3u cm", distance);
        LCD_String(0, 0, display_text);
 
        sprintf(display_text, "Light: %3u%%", ldr_percent);
        LCD_String(1, 0, display_text);
 
        Servo_SetAngle(distance);
        Buzzer_LED_Control(distance);
 
        __delay_ms(200);
    }
}
 
void __interrupt() ISR(void) {
    static uint8_t last_button = 1;
 
    if (RBIF) {
        if (PORTBbits.RB5 == 0 && last_button == 1) {
            __delay_ms(50);
            if (PORTBbits.RB5 == 0) {
                emergency_mode = !emergency_mode;
            }
        }
        last_button = PORTBbits.RB5;
        RBIF = 0;
    }
}
 
void LCD_Init() {
    RS_dir = EN_dir = D4_dir = D5_dir = D6_dir = D7_dir = 0;
    __delay_ms(50);
    LCD_Command(0x33);
    LCD_Command(0x32);
    LCD_Command(0x28);
    LCD_Command(0x0C);
    LCD_Command(0x01);
    LCD_Command(0x06);
}
 
void LCD_Command(uint8_t cmd) {
    RS = 0;
    D4 = (cmd >> 4) & 1; D5 = (cmd >> 5) & 1;
    D6 = (cmd >> 6) & 1; D7 = (cmd >> 7) & 1;
    LCD_PulseEnable();
    D4 = (cmd >> 0) & 1; D5 = (cmd >> 1) & 1;
    D6 = (cmd >> 2) & 1; D7 = (cmd >> 3) & 1;
    LCD_PulseEnable();
    if (cmd == 0x01) __delay_ms(2);
}
 
void LCD_Data(uint8_t data) {
    RS = 1;
    D4 = (data >> 4) & 1; D5 = (data >> 5) & 1;
    D6 = (data >> 6) & 1; D7 = (data >> 7) & 1;
    LCD_PulseEnable();
    D4 = (data >> 0) & 1; D5 = (data >> 1) & 1;
    D6 = (data >> 2) & 1; D7 = (data >> 3) & 1;
    LCD_PulseEnable();
    __delay_us(50);
}
 
void LCD_String(uint8_t row, uint8_t col, const char *text) {
    uint8_t address = (row == 0) ? 0x80 : 0xC0;
    LCD_Command(address + col);
    while (*text) LCD_Data(*text++);
}
 
void LCD_PulseEnable() {
    EN = 1; __delay_us(1);
    EN = 0; __delay_us(100);
}
 
void SR04_Init() {
    TRIG_dir = 0;
    ECHO_dir = 1;
    T1CON = 0x00;
}
 
uint16_t SR04_GetDistance(void) {
    uint16_t echo_time = 0;
    uint16_t timeout = 60000;
 
    TRIG = 1;
    __delay_us(10);
    TRIG = 0;
 
    while (!ECHO && timeout--) __delay_us(1);
    if (timeout == 0) return 999;
 
    TMR1H = TMR1L = 0;
    T1CONbits.TMR1ON = 1;
 
    timeout = 60000;
    while (ECHO && timeout--) __delay_us(1);
    T1CONbits.TMR1ON = 0;
 
    if (timeout == 0) return 999;
 
    echo_time = (TMR1H << 8) | TMR1L;
    return (uint16_t)(echo_time / 29); // adjusted for 4 MHz and 1:2 prescaler
}
 
 
void PWM1_Init(uint16_t freq) {
    PR2 = (_XTAL_FREQ / (4UL * 4 * freq)) - 1;
    T2CON = 0b00000101;
    CCP1CON = 0b00001100;
    CCPR1L = 0;
}
 
void PWM1_SetDuty(uint8_t duty) {
    CCPR1L = duty >> 2;
    CCP1CON = (CCP1CON & 0xCF) | ((duty & 0x03) << 4);
}
 
void Servo_SetAngle(uint8_t angle) {
    static uint8_t current = PWM_MIN;
    uint8_t target = PWM_MIN + ((angle * (PWM_MAX - PWM_MIN)) / 180);
    if (current < target) current++;
    else if (current > target) current--;
    PWM1_SetDuty(current);
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
    ADCON0 = (channel << 3) | 0x01;
    __delay_ms(2);
    GO_nDONE = 1;
    while (GO_nDONE);
    return ((ADRESH << 8) | ADRESL);
}
 