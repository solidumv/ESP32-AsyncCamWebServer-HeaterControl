#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
//#include <WebServer.h>
#include "ESPAsyncWebServer.h"
#include <Arduino_JSON.h>
#include <ESPmDNS.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "Adafruit_SHT4x.h"
 
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
Adafruit_SH110X display = Adafruit_SH110X(64, 128, &Wire);

const char *ssid = "Pale_Murphy";
const char *password = "6716466205";
JSONVar board;
AsyncWebServer server(80);
AsyncEventSource events("/events");
 
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
uint32_t readID = 0;
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

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
  // Copies the sender mac address to a string
  char macStr[18];
  char pwmState[10];
  if (incomingReadings.pwmState == 1) strcpy(pwmState, "PWM ON"); 
  else if (incomingReadings.pwmState == 2) strcpy(pwmState, "ALWAYS ON"); 
  else strcpy(pwmState, "PWM OFF"); 
  
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  
  board["id"] = 0;
  board["MeasuredTemperature"] = measured_temp;
  board["MeasuredHumidity"] = measured_humid;
  board["DesiredTemperature"] = desired_temp;
  board["pwmState"] = pwmState;
  board["readingId"] = String(readID);
  String jsonString = JSON.stringify(board);
  events.send(jsonString.c_str(), "new_readings", millis());
}

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP-NOW DASHBOARD</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="stylesheet" href="https://use.fontawesome.com/releases/v5.7.2/css/all.css" integrity="sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr" crossorigin="anonymous">
  <link rel="icon" href="data:,">
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    p {  font-size: 1.2rem;}
    body {  margin: 0;}
    .topnav { overflow: hidden; background-color: #2f4468; color: white; font-size: 1.7rem; }
    .content { padding: 20px; }
    .card { background-color: white; box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5); }
    .cards { max-width: 700px; margin: 0 auto; display: grid; grid-gap: 2rem; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); }
    .reading { font-size: 2.8rem; }
    .packet { color: #bebebe; }
    .card.temperature { color: #fd7e14; }
    .card.humidity { color: #1b78e2; }
    .card.bolt { color: #1be285; }
  </style>
</head>
<body>
  <div class="topnav">
    <h3>ESP-NOW DASHBOARD</h3>
  </div>
  <div class="content">
    <div class="cards">
      <div class="card temperature">
        <h4><i class="fas fa-thermometer-half"></i> MEASURED TEMPERATURE</h4><p><span class="reading"><span id="t1"></span> &deg;C</span></p><p class="packet">Reading ID: <span id="rt1"></span></p>
      </div>
      <div class="card humidity">
        <h4><i class="fas fa-tint"></i> MEASURED HUMIDITY</h4><p><span class="reading"><span id="h1"></span> &percnt;</span></p><p class="packet">Reading ID: <span id="rh1"></span></p>
      </div>
      <div class="card temperature">
        <h4><i class="fas fa-thermometer-half"></i> DESIRED TEMPERATURE</h4><p><span class="reading"><span id="st1"></span> &deg;C</span></p><p class="packet">Reading ID: <span id="rst1"></span></p>
      </div>
      <div class="card bolt">
        <h4><i class="fas fa-bolt"></i> PWM STATE</h4><p><span class="reading"><span id="p1"></span> </span></p><p class="packet">Reading ID: <span id="rp1"></span></p>
      </div>
    </div>
  </div>
<script>
if (!!window.EventSource) {
 var source = new EventSource('/events');
 
 source.addEventListener('open', function(e) {
  console.log("Events Connected");
 }, false);
 source.addEventListener('error', function(e) {
  if (e.target.readyState != EventSource.OPEN) {
    console.log("Events Disconnected");
  }
 }, false);
 
 source.addEventListener('message', function(e) {
  console.log("message", e.data);
 }, false);
 
 source.addEventListener('new_readings', function(e) {
  console.log("new_readings", e.data);
  var obj = JSON.parse(e.data);
  document.getElementById("t"+obj.id).innerHTML = obj.MeasuredTemperature.toFixed(2);
  document.getElementById("h"+obj.id).innerHTML = obj.MeasuredHumidity.toFixed(2);
  document.getElementById("st"+obj.id).innerHTML = obj.DesiredTemperature.toFixed(2);
  document.getElementById("p"+obj.id).innerHTML = obj.pwmState.toString();
  document.getElementById("rt"+obj.id).innerHTML = obj.readingId;
  document.getElementById("rh"+obj.id).innerHTML = obj.readingId;
  document.getElementById("rst"+obj.id).innerHTML = obj.readingId;
  document.getElementById("rp"+obj.id).innerHTML = obj.readingId;
 }, false);
}
</script>
</body>
</html>)rawliteral";

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
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  server.addHandler(&events);
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
  static unsigned long lastEventTime = millis();
  static const unsigned long EVENT_INTERVAL_MS = 5000;
  if ((millis() - lastEventTime) > EVENT_INTERVAL_MS) {
    events.send("ping",NULL,millis());
    lastEventTime = millis();
  }
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
