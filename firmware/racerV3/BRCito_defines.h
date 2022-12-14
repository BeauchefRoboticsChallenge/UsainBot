// Pinout BRCito v0.9 agosto 2022

// Wheel pins
// Left wheel (LW)
#define PIN_LW_PWM 6
#define PIN_LW_A 10
#define PIN_LW_B 13

// Right wheel (RW)
#define PIN_RW_PWM 5
#define PIN_RW_A 12
#define PIN_RW_B 11

// Sensor array
#define S1 A2
#define S2 A3
#define S3 A4
#define S4 A5
#define S5 A6
#define S6 A7

#define NUM_ARRAY_SENSORS 6
#define NUM_SIDE_SENSORS 2

// Right sensor
#define PIN_RS A1
// Left sensor
#define PIN_LS A0

#define LED_PIN 9
#define BUTTON_PIN 7

// leds
#define LED_UP 0
#define LED_DOWN 1

// State Machine
// actions
#define BUTTON 0x22

// states
#define STDBY 0x24
#define FOLLOW_LINE 0xFA
