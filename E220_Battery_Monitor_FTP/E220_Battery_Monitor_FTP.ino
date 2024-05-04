#include <WiFi.h>
#include <FS.h>
#include <SPIFFS.h>
#include <LittleFS.h>
#include <FTPServer.h>

// Replace with your network details
const char * ssid = "R2D2";
const char * password = "sissy4357";

WiFiClient client;

FTPServer ftpSrv(LittleFS);

void setup(){

  Serial.begin(9600);
  delay(1000);
  
  wifi_Start();

  bool fsok = LittleFS.begin(true);
  Serial.printf_P(PSTR("FS init: %s\n"), fsok ? PSTR("ok") : PSTR("fail!"));

  Serial.printf_P(PSTR("\nConnected to %s, IP address is %s\n"), ssid, WiFi.localIP().toString().c_str());

  // setup the ftp server with username and password
  // ports are defined in FTPCommon.h, default is
  //   21 for the control connection
  //   50009 for the data connection (passive mode by default)
  ftpSrv.begin(F("admin"), F("password")); //username, password for ftp.  set ports in ESP8266FtpServer.h  (default 21, 50009 for PASV)
}

void loop(){ 
  ftpSrv.handleFTP();    
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

  if (WiFi.waitForConnectResult() != 3) {
    delay(3000);
    wifi_Start();
  }
}
