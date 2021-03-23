#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "Adafruit_SHT4x.h"
 
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
Adafruit_SH110X display = Adafruit_SH110X(64, 128, &Wire);

const char *ssid = "Pale_Murphy";
const char *password = "6716466205";
WebServer server(80);
 
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

char pwmStateChar[11] = "PWM OFF";

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

void handleRoot() { //for webServer
  char temp[400];
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  snprintf(temp, 400,

           "<html>\
  <head>\
    <meta http-equiv='refresh' content='5'/>\
    <title>ESP32 Demo</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
    </style>\
  </head>\
  <body>\
    <h1>Hello from ESP32!</h1>\
    <p>Uptime: %02d:%02d:%02d</p>\
    <p>PWM State: %s; Set Temperature: %2.2f C</p>\
    <p>Measured Temperature: %2.2f C; Humidity: %2.2f%c rH</p>\
    <img src=\"/test.svg\" />\
  </body>\
</html>",

           hr, min % 60, sec % 60,  pwmStateChar, desired_temp, measured_temp, measured_humid,'%'
          );
  server.send(200, "text/html", temp);
}

void drawGraph() {
  String out = "";
  char temp[100];
  out += "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\"400\" height=\"150\">\n";
  out += "<rect width=\"400\" height=\"150\" fill=\"rgb(250, 230, 210)\" stroke-width=\"1\" stroke=\"rgb(0, 0, 0)\" />\n";
  out += "<g stroke=\"black\">\n";
  int y = rand() % 130;
  for (int x = 10; x < 390; x += 10) {
    int y2 = rand() % 130;
    sprintf(temp, "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke-width=\"1\" />\n", x, 140 - y, x + 10, 140 - y2);
    out += temp;
    y = y2;
  }
  out += "</g>\n</svg>\n";

  server.send(200, "image/svg+xml", out);
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }

  server.send(404, "text/plain", message);
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  ///////WIFI SERVER INITIALIZATION//////////
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  if (MDNS.begin("esp32")) {
    Serial.println("MDNS responder started");
  }
  server.on("/", handleRoot);
  server.on("/test.svg", drawGraph);
  server.on("/inline", []() {
    server.send(200, "text/plain", "this works as well");
  });
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
  
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
  display.print("Connected to: "); display.println(ssid);
  display.print("IP: "); display.println(WiFi.localIP());
  display.display(); // actually display all of the above
  Serial.println(ssid);
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
  server.handleClient();
  sensors_event_t humidity, temp;
  innerTimeStamp = millis();
  uint32_t delayCounts;
  sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data  
  measured_temp = temp.temperature;
  measured_humid = humidity.relative_humidity;
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("Network: "); display.println(ssid);
  display.print("IP: "); display.println(WiFi.localIP());
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
  Serial.println(temp.temperature);
  innerTimeStamp = millis() - innerTimeStamp;
//  Serial.println(innerTimeStamp);
  delayCounts = REFRESH_RATE - (uint32_t)innerTimeStamp;
  if (delayCounts > REFRESH_RATE || delayCounts < 0) delayCounts = 1;
  delay(delayCounts);
}
