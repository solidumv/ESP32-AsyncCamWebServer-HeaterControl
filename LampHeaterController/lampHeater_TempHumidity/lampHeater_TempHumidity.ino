#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "Adafruit_SHT4x.h"
 
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
Adafruit_SH110X display = Adafruit_SH110X(64, 128, &Wire);
 
// OLED FeatherWing buttons map to different pins depending on board:
#if defined(ESP8266)
  #define BUTTON_A  0
  #define BUTTON_B 16
  #define BUTTON_C  2
#elif defined(ESP32)
  #define BUTTON_A 15
  #define BUTTON_B 32
  #define BUTTON_C 14
#elif defined(ARDUINO_STM32_FEATHER)
  #define BUTTON_A PA15
  #define BUTTON_B PC7
  #define BUTTON_C PC5
#elif defined(TEENSYDUINO)
  #define BUTTON_A  4
  #define BUTTON_B  3
  #define BUTTON_C  8
#elif defined(ARDUINO_NRF52832_FEATHER)
  #define BUTTON_A 31
  #define BUTTON_B 30
  #define BUTTON_C 27
#else // 32u4, M0, M4, nrf52840 and 328p
  #define BUTTON_A  9
  #define BUTTON_B  6
  #define BUTTON_C  5
#endif

#define REFRESH_RATE 150 //ms

#define PWM_OUT 33
#define PWM_OFF 0
#define PWM_ON  1
#define ALL_ON  2
#define PWM_TIMER_13_BIT 13
#define PWM_BASE_FREQ 5000
#define LED_CHANNEL_0 0

struct Button {
    const uint8_t PIN;
    uint32_t numberKeyPresses;
    bool pressed;
};

Button BUTTON_INCREASE = {BUTTON_A, 0, false};
Button BUTTON_PWM_CHANGE = {BUTTON_B, 0, false};
Button BUTTON_DECREASE = {BUTTON_C, 0, false};

float innerTimeStamp;
//PID and PWM Variables//
uint32_t pwmState;
uint32_t setTemp;
float desired_temp; //degrees C
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;
int kp = 1000;   int ki = 200;   int kd = 600;
int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
float last_kp = 0;
float last_ki = 0;
float last_kd = 0;

void IRAM_ATTR fnIncreaseTemp() {
  if (pwmState = PWM_ON) setTemp += 1;
} 

void IRAM_ATTR fnDecreaseTemp() {
  if (pwmState = PWM_ON)  setTemp -= 1;
} 

void IRAM_ATTR fnPwmStateChange() {
    switch(pwmState) {
      case PWM_OFF : pwmState = PWM_ON; break;
      case PWM_ON : pwmState = ALL_ON; break;
      case ALL_ON : pwmState = PWM_OFF; break;
      default : pwmState = PWM_OFF;
    }
}

void pwmAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * min(value, valueMax);
  ledcWrite(channel, duty);
}


void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  
  ///////TEMP AND HUMIDITY INITIALIZATION//////////
  Serial.println("Adafruit SHT4x test");
  if (! sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
    while (1) delay(1);
  }
  Serial.println("Found SHT4x sensor");
  Serial.print("Serial number 0x");
  Serial.println(sht4.readSerial(), HEX);

  sht4.setPrecision(SHT4X_MED_PRECISION);
  switch (sht4.getPrecision()) {
     case SHT4X_HIGH_PRECISION: 
       Serial.println("High precision");
       break;
     case SHT4X_MED_PRECISION: 
       Serial.println("Med precision");
       break;
     case SHT4X_LOW_PRECISION: 
       Serial.println("Low precision");
       break;
  }
  // You can have 6 different heater settings
  // higher heat and longer times uses more power
  // and reads will take longer too!
  sht4.setHeater(SHT4X_NO_HEATER);
  switch (sht4.getHeater()) {
     case SHT4X_NO_HEATER: 
       Serial.println("No heater");
       break;
     case SHT4X_HIGH_HEATER_1S: 
       Serial.println("High heat for 1 second");
       break;
     case SHT4X_HIGH_HEATER_100MS: 
       Serial.println("High heat for 0.1 second");
       break;
     case SHT4X_MED_HEATER_1S: 
       Serial.println("Medium heat for 1 second");
       break;
     case SHT4X_MED_HEATER_100MS: 
       Serial.println("Medium heat for 0.1 second");
       break;
     case SHT4X_LOW_HEATER_1S: 
       Serial.println("Low heat for 1 second");
       break;
     case SHT4X_LOW_HEATER_100MS: 
       Serial.println("Low heat for 0.1 second");
       break;
  }
  setTemp = 37;
  desired_temp = 37.0; //degrees C
  
  ///////OLED INITIALIZATION//////////
  Serial.println("128x64 OLED FeatherWing test");
  display.begin(0x3C, true); // Address 0x3C default
 
  Serial.println("OLED begun");
 
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);
 
  // Clear the buffer.
  display.clearDisplay();
  display.display();
 
  display.setRotation(1);
  Serial.println("Button test");
 
  pinMode(BUTTON_INCREASE.PIN, INPUT_PULLUP);
  attachInterrupt(BUTTON_INCREASE.PIN, fnIncreaseTemp, FALLING);
  pinMode(BUTTON_DECREASE.PIN, INPUT_PULLUP);
  attachInterrupt(BUTTON_DECREASE.PIN, fnDecreaseTemp, FALLING);
  
  // text display tests
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);
  display.print("Connecting to SSID\n'adafruit':");
  display.print("connected!");
  display.println("IP: 10.0.1.23");
  display.println("Sending val #0");
  display.display(); // actually display all of the above

  ///////PWM INITIALIZATION//////////
  pinMode(BUTTON_PWM_CHANGE.PIN, INPUT_PULLUP);
  attachInterrupt(BUTTON_PWM_CHANGE.PIN, fnPwmStateChange, FALLING);
  pwmState = PWM_OFF;
  ledcSetup(LED_CHANNEL_0, PWM_BASE_FREQ, PWM_TIMER_13_BIT);
  ledcAttachPin(PWM_OUT,LED_CHANNEL_0);  
  Time = millis();
}
 
void loop() {
  sensors_event_t humidity, temp;
  innerTimeStamp = millis();
  uint32_t delayCounts;
  sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data  
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("Temperature: "); display.print(temp.temperature); display.println("C");
  display.print("Humidity: "); display.print(humidity.relative_humidity); display.println("% rH");
  display.print("Set Temp: "); display.print(desired_temp); display.println("C");
  display.print("PWM State: ");
  if (pwmState == PWM_ON) {
    display.println("PWM ON");
    //PID//
    desired_temp = (float) setTemp;
    PID_error = desired_temp - temp.temperature;
    PID_p = 0.01*kp * PID_error;
    PID_i = 0.01*PID_i + (ki * PID_error);
    timePrev = Time;
    Time = (float) millis();
    elapsedTime = (float)(Time - timePrev) / 1000;
    PID_d = 0.01*kd*((PID_error - previous_error)/elapsedTime);
    PID_value = PID_p + PID_i + PID_d;
    if(PID_value < 0)
    {    PID_value = 0;    }
    if(PID_value > 255)  
    {    PID_value = 255;  }  
    pwmAnalogWrite(LED_CHANNEL_0,PID_value);
    previous_error = PID_error; 
  }
  else if (pwmState == ALL_ON) {
    display.println("100% ON");
    pwmAnalogWrite(LED_CHANNEL_0,255);
  }
  else {
    display.println("PWM OFF");
    pwmAnalogWrite(LED_CHANNEL_0,0);
  }
  delay(10);
  yield();
  display.display();
 // innerTimeStamp = millis() - innerTimeStamp;
  delayCounts = REFRESH_RATE - (uint32_t)innerTimeStamp;
  Serial.println(temp.temperature);
  innerTimeStamp = millis() - innerTimeStamp;
  if (delayCounts > REFRESH_RATE) delayCounts = REFRESH_RATE;
  delay(delayCounts);
}
