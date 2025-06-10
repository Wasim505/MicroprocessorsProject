# **Distance Monitoring and Alert System**  
**PIC16F877A Microcontroller Project**  

## **Table of Contents**  
1. [Overview](#overview)  
2. [Hardware Components](#hardware-components)  
3. [Software Configuration](#software-configuration)  
4. [Pin Configuration](#pin-configuration)  
5. [Function Descriptions](#function-descriptions)  
6. [System Workflow](#system-workflow)  
7. [Emergency Mode](#emergency-mode)  
8. [Distance Measurement](#distance-measurement)  
9. [LCD Display](#lcd-display)  
10. [PWM and Servo Control](#pwm-and-servo-control)  
11. [Buzzer and LED Alerts](#buzzer-and-led-alerts)  
12. [Interrupt Handling](#interrupt-handling)  
13. [Customization](#customization)  
14. [Troubleshooting](#troubleshooting)  
15. [License](#license)  

---

## **1. Overview**  
This project implements a **distance monitoring and alert system** using a **PIC16F877A microcontroller**. The system measures distance using an **ultrasonic sensor (HC-SR04)**, displays real-time data on an **LCD**, controls a **servo motor** based on distance, and triggers **LED and buzzer alerts** when an object is too close. An **emergency button** allows immediate system shutdown.  

### **Key Features:**  
- Real-time distance measurement (0-999 cm)  
- Visual and audible alerts when distance ≤ 10 cm  
- Servo motor angle adjustment based on distance  
- Emergency mode for immediate system halt  
- 16x2 LCD display for status monitoring  

---

## **2. Hardware Components**  
- **Microcontroller:** PIC16F877A  
- **Ultrasonic Sensor:** HC-SR04 (Trig: RB0, Echo: RB4)  
- **LCD:** 16x2 (4-bit mode, RD0-RD7)  
- **Buzzer:** RC0  
- **LEDs:**  
  - Status LED (RC1)  
  - Emergency LED (RC4)  
- **Servo Motor:** PWM-controlled (CCP1, RC2)  
- **Emergency Button:** RB5 (Pull-up enabled)  

---

## **3. Software Configuration**  
### **Compiler & Dependencies**  
- **Compiler:** XC8  
- **Libraries:** `<xc.h>`, `<stdint.h>`, `<stdio.h>`  
- **Oscillator:** 20 MHz (HS mode)  

### **Configuration Bits**  
```c
#pragma config FOSC = HS       // High-speed oscillator  
#pragma config WDTE = OFF      // Watchdog disabled  
#pragma config PWRTE = OFF     // Power-up timer disabled  
#pragma config BOREN = ON      // Brown-out reset enabled  
#pragma config LVP = OFF       // Low-voltage programming disabled  
#pragma config CPD = OFF       // Data EEPROM protection off  
#pragma config WRT = OFF       // Flash memory write protection off  
#pragma config CP = OFF        // Code protection off  
```

---

## **4. Pin Configuration**  
| **Function**       | **Pin**  | **Direction** | **Description**            |  
|--------------------|---------|--------------|----------------------------|  
| **LCD RS**         | RD0     | Output       | Register Select            |  
| **LCD EN**         | RD3     | Output       | Enable                     |  
| **LCD D4-D7**      | RD4-RD7 | Output       | Data Bus (4-bit mode)      |  
| **Buzzer**         | RC0     | Output       | Alert sound                |  
| **Status LED**     | RC1     | Output       | Visual alert               |  
| **Emergency LED**  | RC4     | Output       | Emergency mode indicator   |  
| **Ultrasonic Trig**| RB0     | Output       | Trigger pulse              |  
| **Ultrasonic Echo**| RB4     | Input        | Echo pulse measurement     |  
| **Emergency Button**| RB5    | Input        | Pull-up enabled            |  

---

## **5. Function Descriptions**  

### **5.1 LCD Functions**  
- **`LCD_Init()`** – Initializes the LCD in 4-bit mode.  
- **`LCD_Command(uint8_t cmd)`** – Sends a command to the LCD.  
- **`LCD_Data(uint8_t data)`** – Sends data to the LCD.  
- **`LCD_String(uint8_t row, uint8_t col, const char *text)`** – Displays text at a specific position.  
- **`LCD_PulseEnable()`** – Toggles the EN pin to latch data.  

### **5.2 Ultrasonic Sensor (HC-SR04)**  
- **`SR04_Init()`** – Configures Trig (output) and Echo (input) pins.  
- **`SR04_GetDistance()`** – Measures distance (cm) using Timer1.  

### **5.3 PWM & Servo Control**  
- **`PWM1_Init(uint16_t freq)`** – Sets up PWM at 50 Hz (for servo).  
- **`PWM1_SetDuty(uint8_t duty)`** – Adjusts PWM duty cycle.  
- **`Servo_SetAngle(uint8_t angle)`** – Maps distance to servo angle (5%-25% duty).  

### **5.4 Buzzer & LED Control**  
- **`Buzzer_LED_Control(uint16_t dist)`** – Triggers buzzer/LED if distance ≤ 10 cm.  

### **5.5 Interrupt Service Routine (ISR)**  
- Detects emergency button press (RB5) to toggle emergency mode.  

---

## **6. System Workflow**  
1. **Initialization:**  
   - LCD, PWM, and ultrasonic sensor are configured.  
   - Welcome message is displayed.  

2. **Main Loop:**  
   - Distance is measured every 200 ms.  
   - LCD updates with distance and status.  
   - Servo angle adjusts based on distance.  
   - Buzzer/LED activate if distance ≤ 10 cm.  

3. **Emergency Mode:**  
   - Pressing RB5 halts the system.  
   - Emergency LED turns on, buzzer/LED turn off.  

---

## **7. Emergency Mode**  
- **Activation:** Press RB5 (debounced).  
- **Behavior:**  
  - Disables PWM (servo stops).  
  - Freezes distance updates.  
  - Emergency LED stays on until button is pressed again.  

---

## **8. Distance Measurement**  
- **Method:** Time-of-flight (ultrasonic echo).  
- **Formula:**  
  ```c
  distance_cm = (echo_time * 343 * 4.5) / 20000  
  ```
- **Range:** 0-999 cm (returns 999 if timeout).  

---

## **9. LCD Display**  
- **Row 0:** `Dist: XXX cm`  
- **Row 1:** `Status: OK/ALERT!`  

---

## **10. PWM and Servo Control**  
- **Frequency:** 50 Hz (standard for servos).  
- **Duty Cycle:** 5% (0°) to 25% (180°).  

---

## **11. Buzzer and LED Alerts**  
- **Condition:** Distance ≤ 10 cm.  
- **Buzzer:** Continuous tone.  
- **LED:** Blinking.  

---

## **12. Interrupt Handling**  
- **RB5 (Emergency Button):**  
  - Triggers interrupt on falling edge.  
  - Debounced with 50 ms delay.  

---

## **13. Customization**  
- **Alert Threshold:** Modify `ALERT_DISTANCE`.  
- **Servo Range:** Adjust `PWM_MIN` and `PWM_MAX`.  

---

## **14. Troubleshooting**  
| **Issue**                | **Solution**                          |  
|--------------------------|---------------------------------------|  
| LCD not displaying       | Check connections, initialization.   |  
| Incorrect distance       | Verify Timer1 settings.               |  
| Servo not moving         | Confirm PWM frequency/duty cycle.     |  

---

## **15. License**  
**MIT License** – Free for personal and educational use.  

---

### **Notes**  
- Ensure proper pull-up/down resistors for buttons.  
- Adjust `__delay_ms()` values for responsiveness.  

For questions, **ask before assuming anything!** 🚀
