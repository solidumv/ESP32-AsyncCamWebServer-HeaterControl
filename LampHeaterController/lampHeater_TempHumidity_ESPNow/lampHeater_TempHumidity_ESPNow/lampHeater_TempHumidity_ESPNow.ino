#include <SPI.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "Adafruit_SHT4x.h"
 
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
Adafruit_SH110X display = Adafruit_SH110X(64, 128, &Wire);

 // Set your Board ID (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc)
#define BOARD_ID 1

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
#define ESPNOW_DATA_RATE 30 //n * REFRESH_RATE

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
float measured_temp;
float measured_humid;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;
int kp = 1000;   int ki = 200;   int kd = 600;
int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
float last_kp = 0;
float last_ki = 0;
float last_kd = 0;
int connFlag = 1;
char pwmStateChar[11] = "PWM OFF";
uint8_t broadcastAddress[] = {0x3C, 0x61, 0x05, 0x3C, 0x95, 0xA4};
int dataRateCounter = 0;
//ESP Now Structs and Vars
//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int id;
    float temp;
    float hum;
    float setTemp;
    int pwmState;
    int readingId;
} struct_message;
struct_message myData;
unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 3000;        // Interval at which to publish sensor readings
unsigned int readingId = 0;
constexpr char WIFI_SSID[] = "Pale_Murphy";
int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}

void IRAM_ATTR fnIncreaseTemp() {
  if (pwmState == PWM_ON) setTemp += 1;
} 

void IRAM_ATTR fnDecreaseTemp() {
  if (pwmState == PWM_ON)  setTemp -= 1;
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

// callback when data is sent with ESPNow
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  ///////ESPNOW INITIALIZATION//////////
  WiFi.mode(WIFI_STA);
  int32_t channel = getWiFiChannel(WIFI_SSID);
  
  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after

  if (esp_now_init() != ESP_OK) {
    connFlag = 0;
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  ///////TEMP AND HUMIDITY INITIALIZATION//////////
  Serial.println("Adafruit SHT4x test");
  if (! sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
    while (1) delay(1);
  }
  Serial.println("Found SHT4x sensor");
  Serial.print("Serial number 0x");
  Serial.println(sht4.readSerial(), HEX);

  sht4.setPrecision(SHT4X_HIGH_PRECISION);
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
  desired_temp = (float) setTemp; //degrees C
  
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
  display.display(); // actually display all of the above
  Serial.print("MAC Address: "); Serial.println(WiFi.macAddress());

  
  ///////PWM INITIALIZATION//////////
  //pwmStateChar = {"PWM OFF"};
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
  measured_temp = temp.temperature;
  measured_humid = humidity.relative_humidity;
  display.clearDisplay();
  display.setCursor(0,0);
  if (connFlag == 1) {
    display.println("Connected!");
  }
  else display.println("Not Connected!");
//  display.print("Network: "); display.println(ssid);
//  display.print("IP: "); display.println(WiFi.localIP());
  display.print("Temperature: "); display.print(temp.temperature); display.println("C");
  display.print("Humidity: "); display.print(humidity.relative_humidity); display.println("% rH");
  display.print("Set Temp: "); display.print(desired_temp); display.println("C");
  display.print("PWM State: ");
  if (pwmState == PWM_ON) {
    display.println("PWM ON");
    strcpy(pwmStateChar, "PWM ON"); 
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
    display.println("ALWAYS ON");
    strcpy(pwmStateChar, "ALWAYS ON"); 
    pwmAnalogWrite(LED_CHANNEL_0,255);
  }
  else {
    display.println("PWM OFF");
    strcpy(pwmStateChar, "PWM OFF"); 
    pwmAnalogWrite(LED_CHANNEL_0,0);
  }
  delay(10);
  yield();
  display.display();
  myData.id = BOARD_ID;
  myData.temp = measured_temp;
  myData.hum  = measured_humid;
  myData.setTemp = desired_temp;
  myData.pwmState = pwmState;
  if (dataRateCounter >= ESPNOW_DATA_RATE) {
    dataRateCounter = 0;
    myData.readingId = readingId++;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  }
  //Serial.println(temp.temperature);
  dataRateCounter++;
  innerTimeStamp = millis() - innerTimeStamp;
//  Serial.println(innerTimeStamp);
  delayCounts = REFRESH_RATE - (uint32_t)innerTimeStamp;
  if (delayCounts > REFRESH_RATE || delayCounts < 0) delayCounts = 1;
  delay(delayCounts);
}
