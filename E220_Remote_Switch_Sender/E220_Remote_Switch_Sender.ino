//  Modification for battery voltage monitoring
//  See library downloads for each library license.

// With FIXED SENDER configuration
#define DESTINATION_ADDL 3

#include <Arduino.h>
#include "WiFi.h"
#include <WiFiUdp.h> 
#include <HTTPClient.h>
#include <time.h>
#include "LoRa_E220.h"
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"
#include <esp_sleep.h>
#include <Ticker.h>

#import "index7.h"  //Video feed HTML; do not remove

WiFiClient client;

///Are we currently connected?
boolean connected = false;

WiFiUDP udp;
// local port to listen for UDP packets
const int udpPort = 1337;
char incomingPacket[255];
char replyPacket[] = "Hi there! Got the message :-)";
//NTP Time Servers
const char * udpAddress1 = "pool.ntp.org";
const char * udpAddress2 = "time.nist.gov";

String dtStamp;

#define TZ "EST+5EDT,M3.2.0/2,M11.1.0/2"

int DOW, MONTH, DATE, YEAR, HOUR, MINUTE, SECOND;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

char strftime_buf[64];

// ---------- esp32 pins --------------
 LoRa_E220 e220ttl(&Serial2, 15, 21, 19); //  RX AUX M0 M1

//LoRa_E220 e220ttl(&Serial2, 22, 4, 18, 21, 19, UART_BPS_RATE_9600); //  esp32 RX <-- e220 TX, esp32 TX --> e220 RX AUX M0 M1
// -------------------------------------

#define AUX 15

// Replace with your network details
const char *ssid = "R2D2";
const char *password = "sissy4357";

AsyncWebServer server(80);

int data = 2;

// Struct to hold date and time components
struct DateTime {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
};

// Define the maximum length for the timestamp
const int MAX_TIMESTAMP_LENGTH = 30;

struct Message {
 int switchState;
 char timestamp[MAX_TIMESTAMP_LENGTH];
}message;

Ticker oneTick;
Ticker onceTick;

String linkAddress = "xxx.xxx.xxx.xxx:80";

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

volatile int watchdogCounter;
int totalwatchdogCounter;
int cameraPowerOff = 0;
int watchDog;

void ISRwatchdog() {

  portENTER_CRITICAL_ISR(&mux);

  watchdogCounter++;

  if (watchdogCounter >= 75) {

    watchDog = 1;
  }

  portEXIT_CRITICAL_ISR(&mux);
}

int cameraFlag;
int needAnotherCountdown = 0;

void ISRcamera() {
  batteryOff();
}

bool got_interrupt = false;
 
void interruptHandler() {
  got_interrupt = true;
}  // interruptHandler

void setup() {

  Serial.begin(9600);
  delay(500);
  
  // Startup all pins and UART
  e220ttl.begin();

  e220ttl.setMode(MODE_1_WOR_TRANSMITTER);

  Serial.println("Hi, I'm going to send WOR message!");

  // Send message
  ResponseStatus rs = e220ttl.sendFixedMessage(0, DESTINATION_ADDL, 68, "\nHello, world? WOR!");
  // Check If there is some problem of succesfully send
  Serial.println(rs.getResponseDescription());

  //  e220ttl.setMode(MODE_0_NORMAL);

  Serial.println("\n\n\nWebserver and");
  Serial.println("E220-900T30D Transceiver for turning ON Videofeed\n");

  wifi_Start();

  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  // See https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv for Timezone codes for your region
  setenv("TZ", "EST+5EDT,M3.2.0/2,M11.1.0/2", 3);   // this sets TZ to Indianapolis, Indiana

  attachInterrupt(digitalPinToInterrupt(AUX), interruptHandler, FALLING);

  server.on("/relay", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, PSTR("text/html"), HTML7, processor7);
    data = 1;
    needAnotherCountdown = 1;
    countdownTrigger();
  });

  server.begin();

  oneTick.attach(1.0, ISRwatchdog);  //watchdog  ISR triggers every 1 second
} 

void loop() {

  DateTime currentDateTime = getCurrentDateTime();	
  
  if((currentDateTime.minute % 2 == 0) && (currentDateTime.second == 0)){
    //webInterface();  //Sends URL Get request to wake up Radio and ESP32 at 1 minute interval
                      // URL = http://10.0.0.27/relay	 
    //delay(1000);
  }
  
  //udp only send data when connected
  if (connected)
  {

    //Send a packet
    udp.beginPacket(udpAddress1, udpPort);
    udp.printf("Seconds since boot: %u", millis() / 1000);
    udp.endPacket();
  }
  
  // If something available
  if (e220ttl.available() > 1) {
    // read the String message
    ResponseContainer rc = e220ttl.receiveMessage();
    // Is something goes wrong print error
    if (rc.status.code != 1) {
      Serial.println(rc.status.getResponseDescription());
    } else {
      // Print the data received
      Serial.println(rc.status.getResponseDescription());
      Serial.println(rc.data);
    }
  }
    
  if (Serial.available()) {
    message.switchState = data;
	  message.switchState = Serial.parseInt();

	  // Send message
	  ResponseStatus rs = e220ttl.sendFixedMessage(0, 2, 68, &message, sizeof(Message));

	  // Check If there is some problem of succesfully send
	  Serial.println(rs.getResponseDescription());
  }  
}

int main() {
    // Create an instance of the Message struct
    Message message;

    // Get the timestamp using the get_time function and assign it to the struct member
    String timestamp = get_time();
    timestamp.toCharArray(message.timestamp, MAX_TIMESTAMP_LENGTH);

    // Now you can use message.timestamp as needed...

    return 0;
}

String processor7(const String &var) {

  //index7.h

  if (var == F("LINK"))
    return linkAddress;

  return String();
}

void batteryOff() {
  int data = 2;
  switchOne(data);
  oneTick.detach();
}

void configTime()
{

  configTime(0, 0, udpAddress1, udpAddress2);
  setenv("TZ", "EST+5EDT,M3.2.0/2,M11.1.0/2", 3);   // this sets TZ to Indianapolis, Indiana
  tzset();

  //udp only send data when connected
  if (connected)
  {

    //Send a packet
    udp.beginPacket(udpAddress1, udpPort);
    udp.printf("Seconds since boot: %u", millis() / 1000);
    udp.endPacket();
  }

  Serial.print("wait for first valid timestamp");

  while (time(nullptr) < 100000ul)
  {
    Serial.print(".");
    delay(5000);
  }

  Serial.println("\nSystem Time set\n");

  get_time();

  Serial.println(message.timestamp);

}

void countdownTrigger() {
  // Perform countdown actions here
  Serial.println("\nCountdown timer triggered!\n");
  //getDateTime();
  // Schedule the next countdown if needed
  if (needAnotherCountdown == 1) {
    onceTick.once(30, ISRcamera);
    int data = 1;
    switchOne(data);
    needAnotherCountdown = 0;
  }
}

// Function to get current date and time
DateTime getCurrentDateTime() {
    DateTime currentDateTime;
    time_t now = time(nullptr);
    struct tm *ti = localtime(&now);

    // Extract individual components
    currentDateTime.year = ti->tm_year + 1900;
    currentDateTime.month = ti->tm_mon + 1;
    currentDateTime.day = ti->tm_mday;
    currentDateTime.hour = ti->tm_hour;
    currentDateTime.minute = ti->tm_min;
    currentDateTime.second = ti->tm_sec;

    return currentDateTime;
}

// Function to get the timestamp
String get_time() {

    time_t now;
    time(&now);
    char time_output[MAX_TIMESTAMP_LENGTH];
    strftime(time_output, MAX_TIMESTAMP_LENGTH, "%a  %m/%d/%y   %T", localtime(&now)); 
    return String(time_output); // returns timestamp in the specified format
}

void switchOne(int data) {

  if (data == 1) {
    int data = 1;
    Serial.println("\nBattery Switch is ON");
    Serial.println("ESP32 waking from Deep Sleep\n");
  }

  if (data == 2) {
    int data = 2;
    Serial.println("\nBattery power switched OFF");
    Serial.println("ESP32 going to Deep Sleep\n");
  }

 	Serial.println("Hi, I'm going to send message!");

  get_time();
 
  Message message;
 
  //initialize struct members
  message.switchState = data;
  
  // Initialize the timestamp 
  String timestamp = get_time();
  timestamp.toCharArray(message.timestamp, MAX_TIMESTAMP_LENGTH);  

  Serial.print("TimeStamp:  "); Serial.println(message.timestamp);

  // Send message
	ResponseStatus rs = e220ttl.sendFixedMessage(0, DESTINATION_ADDL, 68, &message, sizeof(Message));
	// Check If there is some problem of succesfully send
  Serial.println(rs.getResponseDescription());  
}  

void webInterface() {

  //getTimeDate();

  String data = "http://10.0.0.27/relay";

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;    // Declare object of class HTTPClient

    http.begin(data);  // Specify request destination

    // No need to add content-type header for a simple GET request

    int httpCode = http.GET();   // Send the GET request

    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();  // Get the response payload

      Serial.print("HttpCode: ");
      Serial.print(httpCode);   // Print HTTP return code
      Serial.println("\n");
      //Serial.print("  Data echoed back from Hosted website: ");
      //Serial.println(payload);  // Print payload response      

      http.end();  // Close HTTPClient
    } else {
      Serial.print("HttpCode: ");
      Serial.print(httpCode);   // Print HTTP return code
      Serial.println("  URL Request failed.");

      http.end();   // Close HTTPClient
    }
  } else {
    Serial.println("Error in WiFi connection");
  }
}


void wifi_Start() {

//Server settings
#define ip { 10, 0, 0, 27 }
#define subnet \
  { 255, 255, 255, 0 }
#define gateway \
  { 10, 0, 0, 1 }
#define dns \
  { 10, 0, 0, 1 }

  WiFi.mode(WIFI_AP_STA);

  Serial.println();
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());

  // We start by connecting to WiFi Station
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  delay(1000);

  //setting the static addresses in function "wifi_Start
  IPAddress ip;
  IPAddress gateway;
  IPAddress subnet;
  IPAddress dns;

  WiFi.config(ip, gateway, subnet, dns);

  Serial.println("Web server running. Waiting for the ESP32 IP...");

  // Printing the ESP IP address
  Serial.print("Server IP:  ");
  Serial.println(WiFi.localIP());
  Serial.print("Port:  ");
  Serial.println("80");
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Wi-Fi Channel: ");
  Serial.println(WiFi.channel());
  Serial.println("\n");

  delay(500);

  WiFi.waitForConnectResult();

  Serial.printf("Connection result: %d\n", WiFi.waitForConnectResult());

  server.begin();


  if (WiFi.waitForConnectResult() != 3) {
    delay(3000);
    wifi_Start();
  }
}

void printParameters(struct Configuration configuration) {
	DEBUG_PRINTLN("----------------------------------------");

	DEBUG_PRINT(F("HEAD : "));  DEBUG_PRINT(configuration.COMMAND, HEX);DEBUG_PRINT(" ");DEBUG_PRINT(configuration.STARTING_ADDRESS, HEX);DEBUG_PRINT(" ");DEBUG_PRINTLN(configuration.LENGHT, HEX);
	DEBUG_PRINTLN(F(" "));
	DEBUG_PRINT(F("AddH : "));  DEBUG_PRINTLN(configuration.ADDH, HEX);
	DEBUG_PRINT(F("AddL : "));  DEBUG_PRINTLN(configuration.ADDL, HEX);
	DEBUG_PRINTLN(F(" "));
	DEBUG_PRINT(F("Chan : "));  DEBUG_PRINT(configuration.CHAN, DEC); DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.getChannelDescription());
	DEBUG_PRINTLN(F(" "));
	DEBUG_PRINT(F("SpeedParityBit     : "));  DEBUG_PRINT(configuration.SPED.uartParity, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.SPED.getUARTParityDescription());
	DEBUG_PRINT(F("SpeedUARTDatte     : "));  DEBUG_PRINT(configuration.SPED.uartBaudRate, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.SPED.getUARTBaudRateDescription());
	DEBUG_PRINT(F("SpeedAirDataRate   : "));  DEBUG_PRINT(configuration.SPED.airDataRate, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.SPED.getAirDataRateDescription());
	DEBUG_PRINTLN(F(" "));
	DEBUG_PRINT(F("OptionSubPacketSett: "));  DEBUG_PRINT(configuration.OPTION.subPacketSetting, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.OPTION.getSubPacketSetting());
	DEBUG_PRINT(F("OptionTranPower    : "));  DEBUG_PRINT(configuration.OPTION.transmissionPower, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.OPTION.getTransmissionPowerDescription());
	DEBUG_PRINT(F("OptionRSSIAmbientNo: "));  DEBUG_PRINT(configuration.OPTION.RSSIAmbientNoise, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.OPTION.getRSSIAmbientNoiseEnable());
	DEBUG_PRINTLN(F(" "));
	DEBUG_PRINT(F("TransModeWORPeriod : "));  DEBUG_PRINT(configuration.TRANSMISSION_MODE.WORPeriod, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription());
	DEBUG_PRINT(F("TransModeEnableLBT : "));  DEBUG_PRINT(configuration.TRANSMISSION_MODE.enableLBT, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.TRANSMISSION_MODE.getLBTEnableByteDescription());
	DEBUG_PRINT(F("TransModeEnableRSSI: "));  DEBUG_PRINT(configuration.TRANSMISSION_MODE.enableRSSI, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription());
	DEBUG_PRINT(F("TransModeFixedTrans: "));  DEBUG_PRINT(configuration.TRANSMISSION_MODE.fixedTransmission, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());


	DEBUG_PRINTLN("----------------------------------------");
}

void printModuleInformation(struct ModuleInformation moduleInformation) {
	Serial.println("----------------------------------------");
	DEBUG_PRINT(F("HEAD: "));  DEBUG_PRINT(moduleInformation.COMMAND, HEX);DEBUG_PRINT(" ");DEBUG_PRINT(moduleInformation.STARTING_ADDRESS, HEX);DEBUG_PRINT(" ");DEBUG_PRINTLN(moduleInformation.LENGHT, DEC);

	Serial.print(F("Model no.: "));  Serial.println(moduleInformation.model, HEX);
	Serial.print(F("Version  : "));  Serial.println(moduleInformation.version, HEX);
	Serial.print(F("Features : "));  Serial.println(moduleInformation.features, HEX);
	Serial.println("----------------------------------------");
}
