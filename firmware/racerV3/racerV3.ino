/* Sketch UsainBot v2.0

   by Gonzalo Olave based on v1.0 by Matias Mattamala and G.O.
   v1.0 -> https://github.com/mmattamala/racer/blob/master/arduino/robot_brc/robot_brc.ino
   last update 26/4/2022

   This code is in the public domain and it is licensed under the BEER-JUICE-WARE licence

   ------------------------------------------------------------
   "THE BEERWARE LICENSE" (Revision 42):
   The authors wrote this code. As long as you retain this
   notice, you can do whatever you want with this stuff. If we
   meet someday, and you think this stuff is worth it, you can
   buy us a beer in return (or a juice).
   ------------------------------------------------------------
*/

// debug
//#define DEBUG_SENSORS

#define CALIBRATE
// serial debug
#define DEBUG

#define LINE_WHITE

#include "DebugUtils.h"
#include <Adafruit_NeoPixel.h>
#include <QTRSensors.h>

// pinout del robot
#include "BRCito_defines.h"

// General definitions
#define LW 0  // left wheel
#define RW 1  // right wheel

#define TIMEOUT 2500

// Estos valores definen las velocidades de las ruedas
#define VEL_MIN 220
#define VEL_MAX 240        //130
#define VEL_CURVA_MAX 230  // 110
#define VEL_CURVA_MIN 0
#define MIN_ERR_THR 1000

// parámetros del controlador PID, para motores pololu de 1000rpm KP<1
float KP = 3;  // 0.3 //2
float KI = 0;
float KD = 15;  //5

// side sensor threshold
#define RS_TH 500
#define LS_TH 500

int r_sensor = 0;
int l_sensor = 0;
int r_counter = 0;
int l_counter = 0;
int st_sensor = 0;

float prop_err, int_err, der_err, last_err;
float pid_out;

bool stdby_state = 0;

// Variables will change:
int idleState = HIGH;       // the current state of the robot
int buttonState;            // the current reading from the input pin
int lastButtonState = LOW;  // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

uint8_t lw_speed;
uint8_t rw_speed;

int th = 800;
float sum1 = 0;
float sum = 0;
float err = 0;
int s_min = TIMEOUT;
int s_max = 0;
float pos = 0;

int counter = 0;
int btn = 0;

uint16_t sensorSide[NUM_SIDE_SENSORS];
uint16_t sensorLine[NUM_ARRAY_SENSORS];

Adafruit_NeoPixel pixel = Adafruit_NeoPixel(2, LED_PIN, NEO_GRB + NEO_KHZ800);
QTRSensors qtr_line, qtr_side;

uint32_t BLACK = pixel.Color(0, 0, 0);
uint32_t RED = pixel.Color(255, 0, 0);        // rojo
uint32_t GREEN = pixel.Color(0, 255, 0);      // magenta
uint32_t BLUE = pixel.Color(0, 0, 200);       // magenta
uint32_t MAGENTA = pixel.Color(255, 0, 200);  // magenta
uint32_t YELLOW = pixel.Color(255, 140, 0);   // magenta
uint32_t ORANGE = pixel.Color(255, 50, 0);    // magenta
uint32_t CYAN = pixel.Color(0, 140, 200);     // cyan

int state = STDBY;

//-------------------------------
// Setup functions
//-------------------------------
void setupWheels() {

  // configure left wheel
  pinMode(PIN_LW_PWM, OUTPUT);
  pinMode(PIN_LW_A, OUTPUT);
  pinMode(PIN_LW_B, OUTPUT);

  // configure right wheel
  pinMode(PIN_RW_PWM, OUTPUT);
  pinMode(PIN_RW_A, OUTPUT);
  pinMode(PIN_RW_B, OUTPUT);

  onWheels();
}

void setupHW() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void setupLeds() {
  pixel.begin();
}

void setupSensors() {
  qtr_side.setTypeRC();
  qtr_side.setSensorPins((const uint8_t[]){
                           PIN_LS, PIN_RS },
                         NUM_SIDE_SENSORS);
  qtr_line.setTypeAnalog();
  qtr_line.setSensorPins((const uint8_t[]){
    S1, S2, S3, S4, S5, S6}, NUM_ARRAY_SENSORS);

#ifdef CALIBRATE
  pixel.setPixelColor(LED_UP, RED);
  pixel.show();
  DEBUG_PRINT("Calibrando...");
  for (int i = 0; i < 100; i++)  // make the calibration take about 10 seconds
  {
    qtr_line.calibrate();
    qtr_side.calibrate();
  }
  pixel.setPixelColor(0, BLACK);
  pixel.show();
  DEBUG_PRINTLN("ok");
#endif
}

void setup() {

  Serial.begin(115200);  // for BT communication and debug

  setupWheels();
  setupLeds();
  setupHW();
  setupSensors();

  delay(500);

  pixel.setBrightness(100);
  pixel.setPixelColor(LED_UP, YELLOW);
  pixel.show();
  delay(500);

  while (true) {
    int boton = digitalRead(BUTTON_PIN);
    if (boton == 0) break;
  }
  delay(100);

  pixel.setPixelColor(LED_UP, GREEN);
  pixel.show();

  pixel.setPixelColor(LED_DOWN, BLUE);
  pixel.show();
  state = FOLLOW_LINE;
}

void loop() {
  // Step 1: Read Raw Sensors
  readRawSensors();

  //testBlink();
  /*
    testWheel(RW);
    delay(1000);
    testWheel(LW);
    delay(1000);
    handBrake();
  */

  btn = digitalRead(BUTTON_PIN);
  debounceButton();

  //DEBUG_PRINTLN(btn);

  if (idleState == LOW) {
    onWheels();
    lineFollowing();
    if (l_sensor < LS_TH) {
      l_counter++;
      pixel.setPixelColor(LED_UP, GREEN);
      pixel.show();
    } else if (r_sensor < RS_TH) {
      r_counter++;
      pixel.setPixelColor(LED_UP, ORANGE);
      pixel.show();
    } else {
      pixel.setPixelColor(LED_UP, BLACK);
      pixel.setPixelColor(LED_DOWN, BLUE);
      pixel.show();
    }
    if (l_sensor < LS_TH && r_sensor < RS_TH) {
      pixel.setPixelColor(LED_UP, BLUE);
      pixel.show();
    }
  } else if (idleState == HIGH) {
    pixel.setPixelColor(LED_UP, RED);
    pixel.setPixelColor(LED_DOWN, RED);
    pixel.show();
    handBrake();
  }
  lastButtonState = btn;
}

void debounceButton() {
  // If the switch changed, due to noise or pressing:
  if (btn != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (btn != buttonState) {
      buttonState = btn;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        idleState = !idleState;
      }
    }
  }
}

//-------------------------------
// Line following functions
//-------------------------------

void lineFollowing() {
  // Step 1: Read Raw Sensors
  // readRawSensors(); -> se lee en el inicio del loop
  // Step 2: Compute error
  computeError();
  // Step 3: Compute PID correction
  computePID();
  // Step 4: Map error correction to motor speeds
  computeMotorSpeeds();
  // Step 5: Apply speeds
  setMotorSpeeds();
}

void readRawSensors() {

  qtr_line.readCalibrated(sensorLine);
  qtr_side.readCalibrated(sensorSide);

#ifdef LINE_WHITE
  pos = qtr_line.readLineWhite(sensorLine);
#else
  pos = qtr_line.readLineBlack(sensorLine);
#endif

  l_sensor = sensorSide[0];
  r_sensor = sensorSide[1];

#ifdef DEBUG_SENSORS
  for (uint8_t i = 0; i < 6; i++) {
    Serial.print(sensorLine[i]);
    Serial.print('\t');
  }
  for (uint8_t i = 0; i < 2; i++) {
    Serial.print(sensorSide[i]);
    Serial.print('\t');
  }
  Serial.println();
#endif
  //DEBUG_PRINTLN(pos);
  //DEBUG_PRINTLN(l_sensor);
  DEBUG_PRINTLN(r_sensor);
}

void computeError() {
  prop_err = pos - TIMEOUT;
  der_err = prop_err - last_err;
  int_err += prop_err;

  last_err = prop_err;
  sum1 = 0;
  sum = 0;
}

void computePID() {
  pid_out = prop_err * KP + int_err * KI + der_err * KD;
  pid_out = pid_out > 2500 ? 2500 : (pid_out < (-2500) ? -2500 : pid_out);
  // DEBUG_PRINTLN(pid_out);
}

void computeMotorSpeeds() {

  if (pid_out > MIN_ERR_THR) {
    lw_speed = map(pid_out, -2500, 2500, VEL_CURVA_MIN, VEL_CURVA_MAX);
    rw_speed = VEL_CURVA_MIN;

  } else if (pid_out < -MIN_ERR_THR) {
    lw_speed = VEL_CURVA_MIN;
    rw_speed = map(pid_out, -2500, 2500, VEL_CURVA_MAX, VEL_CURVA_MIN);

  } else {
    int diff = VEL_MAX - VEL_MIN;
    rw_speed = VEL_MIN + map(pid_out, -MIN_ERR_THR, MIN_ERR_THR, -diff, diff);
    lw_speed = VEL_MIN + map(pid_out, -MIN_ERR_THR, MIN_ERR_THR, diff, -diff);
  }
}


void setMotorSpeeds() {
  analogWrite(PIN_LW_PWM, lw_speed);
  analogWrite(PIN_RW_PWM, rw_speed);
}

void handBrakeLeft() {
  analogWrite(PIN_LW_PWM, 0);
  digitalWrite(PIN_LW_A, HIGH);
  digitalWrite(PIN_LW_B, HIGH);
}

void handBrakeRight() {
  analogWrite(PIN_RW_PWM, 0);
  digitalWrite(PIN_RW_A, HIGH);
  digitalWrite(PIN_RW_B, HIGH);
}

void handBrake() {
  handBrakeLeft();
  handBrakeRight();
}

void onWheels() {
  // this depends on motor connections to driver
  digitalWrite(PIN_LW_A, HIGH);
  digitalWrite(PIN_LW_B, LOW);

  digitalWrite(PIN_RW_A, LOW);
  digitalWrite(PIN_RW_B, HIGH);
}



//-------------------------------
// Test functions
//-------------------------------
void testBlink() {
  /*
    uint32_t color = pixel.Color(255,0,255); // magenta
    pixel.setPixelColor(0,color);
    pixel.show();
  */
  PlasmaPulse(30);
}

void testWheel(int wheel) {

  int t = 20;
  if (wheel == RW) {
    for (int i = 0; i < 255; i += 5) {
      analogWrite(PIN_RW_PWM, i);
      Serial.println("Testing " + String(wheel == LW ? "left " : "right") + " wheel, power = " + String(i));
      delay(t);
    }
    for (int i = 255; i > 0; i -= 5) {
      analogWrite(PIN_RW_PWM, i);
      Serial.println("Testing " + String(wheel == LW ? "left " : "right") + " wheel, power = " + String(i));
      delay(t);
    }
  } else if (wheel == LW) {
    for (int i = 0; i < 255; i += 5) {
      analogWrite(PIN_LW_PWM, i);
      Serial.println("Testing " + String(wheel == LW ? "left " : "right") + " wheel, power = " + String(i));
      delay(t);
    }
    for (int i = 255; i > 0; i -= 5) {
      analogWrite(PIN_LW_PWM, i);
      Serial.println("Testing " + String(wheel == LW ? "left " : "right") + " wheel, power = " + String(i));
      delay(t);
    }
  }
}

void testBrake() {

  digitalWrite(PIN_RW_A, HIGH);
  digitalWrite(PIN_RW_B, LOW);
  digitalWrite(PIN_LW_A, HIGH);
  digitalWrite(PIN_LW_B, LOW);
  analogWrite(PIN_RW_PWM, 100);
  analogWrite(PIN_LW_PWM, 100);

  delay(2000);
  handBrake();

  delay(1000);
}

void PlasmaPulse(uint8_t wait) {
  uint16_t i, j;
  uint8_t brightness = 10;

  for (i = 0; i < pixel.numPixels(); i++) {
    pixel.setPixelColor(i, pixel.Color(255, 255, 255));
  }
  pixel.show();
  delay(wait);
  //Adjust 60 and 90 to the starting and ending colors you want to fade between.
  for (j = 170; j >= 135; --j) {
    for (i = 0; i < pixel.numPixels(); i++) {
      pixel.setPixelColor(i, Wheel((i + j) & 255));
    }
    pixel.show();
    brightness -= 6;
    pixel.setBrightness(brightness);
    delay(wait);
  }

  for (j = 135; j < 1170; j++) {
    for (i = 0; i < pixel.numPixels(); i++) {
      pixel.setPixelColor(i, Wheel((i + j) & 255));
    }
    pixel.show();
    brightness += 6;
    pixel.setBrightness(brightness);
    delay(wait);
  }

  for (i = 0; i < pixel.numPixels(); i++) {
    pixel.setPixelColor(i, pixel.Color(255, 255, 255));
  }
  pixel.show();
  delay(wait);
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if (WheelPos < 85) {
    return pixel.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return pixel.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
    WheelPos -= 170;
    return pixel.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
