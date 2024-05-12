// E220_Transceiver_Remote_Receiver.ino
// William Lucid 3/11/2024 @ 17:17 EDT    Modified 5/12/2024 @ 09:33 EST
// "C:\Users\William\Documents\■■■■   Production  E220  Project\E220 transceiver remote switch 3-11-2024\Mode 0 Rev 3\
// E220_Transceiver_Remote_Switch\E220_Transceiver_Remote_Switch\E220_Transceiver_Remote_Switch.ino"
//

/*
 * EBYTE LoRa E32
 * Stay in sleep mode and wait a wake up WOR message
 *
 * You must configure the address with 0 3 23 with WOR receiver enable
 * and pay attention that WOR period must be the same of sender
 *
 *
 * https://mischianti.org
 *
 * E32        ----- esp32
 * M0         ----- 19 (or 3.3v)
 * M1         ----- 21 (or GND)
 * RX         ----- TX2 (PullUP)
 * TX         ----- RX2 (PullUP)
 * AUX        ----- 15  (PullUP)
 * VCC        ----- 3.3v/5v
 * GND        ----- GND
 *
 */
 
// with this DESTINATION_ADDL 2 you must set
// WOR SENDER configuration to the other device and
// WOR RECEIVER to this device
#define DESTINATION_ADDL 2
 
// If you want use RSSI uncomment //#define ENABLE_RSSI true
// and use relative configuration with RSSI enabled
//#define ENABLE_RSSI true
 
#include "Arduino.h"
#include "LoRa_E220.h"
#include <WiFi.h>
#include <time.h>
#include <FS.h>
#include <LittleFS.h>
#include "esp_system.h"
#include <esp_sleep.h>
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc.h"
#include "driver/rtc_io.h"
#include <INA226_WE.h>
#include <Wire.h>

// Persisted RTC variable
RTC_DATA_ATTR bool isPowerUp = false;
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR bool switch_State = false;  // Initially switch is off

#define AUX GPIO_NUM_15 //WOR Radio/ESP32
#define TRIGGER 23  //INA226 Power Monitor
#define SWITCH 2  //KY002S  Mosfet switch

#define I2C_ADDRESS 0x40

/* There are several ways to create your INA226 object:
 * INA226_WE ina226 = INA226_WE(); -> uses I2C Address = 0x40 / Wire
 * INA226_WE ina226 = INA226_WE(I2C_ADDRESS);   
 * INA226_WE ina226 = INA226_WE(&Wire); -> uses I2C_ADDRESS = 0x40, pass any Wire Object
 * INA226_WE ina226 = INA226_WE(&Wire, I2C_ADDRESS); 
 */

INA226_WE ina226 = INA226_WE(I2C_ADDRESS);

volatile bool event = false;

void alert(){
  event = true;
  detachInterrupt(2);
}

const int pulseDuration = 1000;  // 100 milliseconds (adjust as needed)

// Define the maximum length for the timestamp
const int MAX_TIMESTAMP_LENGTH = 30;

char dtStamp[MAX_TIMESTAMP_LENGTH];

struct Message {
  int switchState;
  char timestamp[MAX_TIMESTAMP_LENGTH];
} message;

 
#define FPM_SLEEP_MAX_TIME           0xFFFFFFF
void callback() {
  Serial.println("Callback");
  Serial.flush();
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("\nWakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("\nWakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("\nWakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("\nWakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("\nWakeup caused by ULP program"); break;
    default: Serial.printf("\nWakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

int data;

bool interruptExecuted = false;

void IRAM_ATTR ISRwakeUp() {
	 // Do not use Serial on interrupt callback
	 interruptExecuted = true;
	 detachInterrupt(AUX);
}

void printParameters(struct Configuration configuration);

void enterDeepSleep() {
  gpio_hold_en(GPIO_NUM_19);
  gpio_hold_en(GPIO_NUM_21);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_15, LOW);
  gpio_deep_sleep_hold_en();
  Serial.println("Going to sleep now");
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}
 
// ---------- esp32 pins --------------
LoRa_E220 e220ttl(&Serial2, 15, 21, 19); //  RX AUX M0 M1
 
//LoRa_E32 e220ttl(&Serial2, 22, 4, 18, 21, 19, UART_BPS_RATE_9600); //  esp32 RX <-- e22 TX, esp32 TX --> e22 RX AUX M0 M1
// -------------------------------------
 
//The setup function is called once at startup of the sketch
void setup()
{
  Serial.begin(9600);
  while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB
  }
  delay(100);

  e220ttl.begin();

  e220ttl.setMode(MODE_2_WOR_RECEIVER);

  // After set configuration comment set M0 and M1 to low
  // and reboot if you directly set HIGH M0 and M1 to program
  ResponseStructContainer c;
  c = e220ttl.getConfiguration();
  Configuration configuration = *(Configuration*)c.data;
  configuration.ADDL = 0x03;
  configuration.ADDH = 0x00;

  configuration.CHAN = 68;

  configuration.SPED.uartBaudRate = UART_BPS_9600;
  configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;
  configuration.SPED.uartParity = MODE_00_8N1;

  configuration.OPTION.subPacketSetting = SPS_200_00;
  configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;
  configuration.OPTION.transmissionPower = POWER_22;

  configuration.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED;
  configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;
  configuration.TRANSMISSION_MODE.enableLBT = LBT_DISABLED;
  configuration.TRANSMISSION_MODE.WORPeriod = WOR_4000_111;

  print_wakeup_reason();
  
  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("\n\nBoot number: " + String(bootCount));

  if (isPowerUp) {
    Serial.println("\nFlag indicates this was a deep sleep wake-up");
  }

  delay(100);

  bool fsok = LittleFS.begin(true);
  Serial.printf_P(PSTR("\nFS init: %s\n"), fsok ? PSTR("ok") : PSTR("fail!"));

  pinMode(AUX, OUTPUT);  //ESP32 pin 15
  pinMode(TRIGGER, OUTPUT);  //ESP32 pin TRIGGER
  pinMode(SWITCH, OUTPUT);  //ESP32 pin 4

  delay(1000);
  Serial.println();
  Serial.println("Start deep sleep!");
  delay(100);
  attachInterrupt(AUX, ISRwakeUp, FALLING);

  e220ttl.setMode(MODE_2_POWER_SAVING);
  delay(1000);

  Serial.println();
  Serial.println("Wake and start listening!");

  Wire.begin(22, 18);

  if(!ina226.init()){
    Serial.println("Failed to init INA226. Check your wiring.");
    while(1){}
  }
  delay(1000);

  /* Set Number of measurements for shunt and bus voltage which shall be averaged
  * Mode *     * Number of samples *
  AVERAGE_1            1 (default)
  AVERAGE_4            4
  AVERAGE_16          16
  AVERAGE_64          64
  AVERAGE_128        128
  AVERAGE_256        256
  AVERAGE_512        512
  AVERAGE_1024      1024
  */
  // ina226.setAverage(AVERAGE_1024); 

  /* Set conversion time in microseconds
     One set of shunt and bus voltage conversion will take: 
     number of samples to be averaged x conversion time x 2
     
     * Mode *         * conversion time *
     CONV_TIME_140          140 µs
     CONV_TIME_204          204 µs
     CONV_TIME_332          332 µs
     CONV_TIME_588          588 µs
     CONV_TIME_1100         1.1 ms (default)
     CONV_TIME_2116       2.116 ms
     CONV_TIME_4156       4.156 ms
     CONV_TIME_8244       8.244 ms  
  */
  // ina226.setConversionTime(CONV_TIME_8244); 
  
  /* Set measure mode
  POWER_DOWN - INA226 switched off
  TRIGGERED  - measurement on demand
  CONTINUOUS  - continuous measurements (default)
  */
  ina226.setMeasureMode(TRIGGERED); // choose mode and uncomment for change of default
  
  /* If the current values delivered by the INA226 differ by a constant factor
     from values obtained with calibrated equipment you can define a correction factor.
     Correction factor = current delivered from calibrated equipment / current delivered by INA226
  */
  // ina226.setCorrectionFactor(0.95);

    /* In the default mode the limit interrupt flag will be deleted after the next measurement within limits. 
     With enableAltertLatch(), the flag will have to be deleted with readAndClearFlags(). 
  */
    ina226.enableAlertLatch();
  
  /* Set the alert type and the limit
      * Mode *        * Description *           * limit unit *
    SHUNT_OVER     Shunt Voltage over limit          mV
    SHUNT_UNDER    Shunt Voltage under limit         mV
    CURRENT_OVER   Current over limit                mA
    CURRENT_UNDER  Current under limit               mA
    BUS_OVER       Bus Voltage over limit            V
    BUS_UNDER      Bus Voltage under limit           V
    POWER_OVER     Power over limit                  mW
  */
  ina226.setAlertType(BUS_UNDER, 4.5);
 
  attachInterrupt(digitalPinToInterrupt(SWITCH), alert, FALLING);

  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  if (ESP_SLEEP_WAKEUP_EXT0 == wakeup_reason) {
    Serial.println("Waked up from external GPIO!");

    gpio_hold_dis(GPIO_NUM_21);
    gpio_hold_dis(GPIO_NUM_19);

    gpio_deep_sleep_hold_dis();

    e220ttl.setMode(MODE_0_NORMAL);

    e220ttl.sendFixedMessage(0, DESTINATION_ADDL, 68, "We have waked up from message, but we can't read It!");
  } 
}
 
// The loop function is called in an endless loop
void loop()
{
 
    if(event){
      ina226.readAndClearFlags(); // reads interrupt and overflow flags and deletes them 
      getINA226(dtStamp);  //displayResults();
      attachInterrupt(digitalPinToInterrupt(SWITCH), alert, FALLING); 
      event = false;
      digitalWrite(TRIGGER, LOW);        
      ina226.readAndClearFlags();
    }      
  
    if (e220ttl.available() > 1) {
      Serial.println("Message arrived!");
      ResponseStructContainer rsc = e220ttl.receiveMessage(sizeof(Message));
      struct Message message = *(Message*)rsc.data;

      //Serial.println(*(int*)(message.switchState));
      Serial.println(message.switchState); 
      free(rsc.data);

      // Work only with full connection
      e220ttl.setMode(MODE_0_NORMAL);

      delay(1000);

      ResponseStatus rsSend = e220ttl.sendFixedMessage(0, DESTINATION_ADDL, 23, "We have received the message!");
      // Check If there is some problem of succesfully send
      Serial.println(rsSend.getResponseDescription());

    if (interruptExecuted) {
        interruptExecuted = false;
      
      Serial.println("WakeUp Callback, AUX pin go LOW and start receive message!\n");

      if (message.switchState == 1) {
        digitalWrite(TRIGGER, LOW);
        delay(pulseDuration);
        digitalWrite(TRIGGER, HIGH);
        switch_State = digitalRead(TRIGGER);  // Read current switch state
        Serial.println("\nBattery power switched ON");
        Serial.println("ESP32 wake from Deep Sleep\n");
        char dtStamp[MAX_TIMESTAMP_LENGTH];
        strncpy(dtStamp, message.timestamp, MAX_TIMESTAMP_LENGTH);  // Copy timestamp
        getINA226(dtStamp);
      }

      if (message.switchState == 2) {
        digitalWrite(TRIGGER, LOW);
        switch_State = digitalRead(TRIGGER);  // Read current switch state
        Serial.println("\nBattery power switched OFF");
        Serial.println("ESP32 going to Deep Sleep\n");
        enterDeepSleep();
      }      
    }
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

// Function to get the timestamp
String get_time() {
  time_t now;
  time(&now);
  char time_output[MAX_TIMESTAMP_LENGTH];
  strftime(time_output, MAX_TIMESTAMP_LENGTH, "%a  %d-%m-%y %T", localtime(&now));
  return String(time_output);  // returns timestamp in the specified format
}

void getINA226(const char* dtStamp) {
  float shuntVoltage_mV = 0.0;
  float loadVoltage_V = 0.0;
  float busVoltage_V = 0.0;
  float current_mA = 0.0;
  float power_mW = 0.0;
  ina226.startSingleMeasurement();
  ina226.readAndClearFlags();
  shuntVoltage_mV = ina226.getShuntVoltage_mV();
  busVoltage_V = ina226.getBusVoltage_V();
  current_mA = ina226.getCurrent_mA();
  power_mW = ina226.getBusPower();
  loadVoltage_V = busVoltage_V + (shuntVoltage_mV / 1000);
  checkForI2cErrors();

  Serial.println(dtStamp);
  Serial.print("\nShunt Voltage [mV]: ");
  Serial.println(shuntVoltage_mV);
  Serial.print("Bus Voltage [V]: ");
  Serial.println(busVoltage_V);
  Serial.print("Load Voltage [V]: ");
  Serial.println(loadVoltage_V);
  Serial.print("Current[mA]: ");
  Serial.println(current_mA);
  Serial.print("Bus Power [mW]: ");
  Serial.println(power_mW);

  if (!ina226.overflow) {
    Serial.println("Values OK - no overflow");
  } else {
    Serial.println("Overflow! Choose higher current range");
  }
  Serial.println();

  // Open a "log.txt" for appended writing
  File log = LittleFS.open("/log.txt", "a");

  if (!log) {
    Serial.println("file 'log.txt' open failed");
  }

  log.print(dtStamp);
  log.print(" , ");
  log.print(shuntVoltage_mV, 3);
  log.print(" , ");
  log.print(busVoltage_V, 3);
  log.print(" , ");
  log.print(loadVoltage_V, 3);
  log.print(" , ");
  log.print(current_mA, 3);
  log.print(" , ");
  log.print(power_mW, 3);
  log.println("");
  log.close();
}

void checkForI2cErrors() {
  byte errorCode = ina226.getI2cErrorCode();
  if (errorCode) {
    Serial.print("I2C error: ");
    Serial.println(errorCode);
    switch (errorCode) {
      case 1:
        Serial.println("Data too long to fit in transmit buffer");
        break;
      case 2:
        Serial.println("Received NACK on transmit of address");
        break;
      case 3:
        Serial.println("Received NACK on transmit of data");
        break;
      case 4:
        Serial.println("Other error");
        break;
      case 5:
        Serial.println("Timeout");
        break;
      default:
        Serial.println("Can't identify the error");
    }
    if (errorCode) {
      while (1) {}
    }
  }
}

void printParameters(struct Configuration configuration) {
  Serial.println("----------------------------------------");

  Serial.print(F("HEAD : "));
  Serial.print(configuration.COMMAND, HEX);
  Serial.print(" ");
  Serial.print(configuration.STARTING_ADDRESS, HEX);
  Serial.print(" ");
  Serial.println(configuration.LENGHT, HEX);
  Serial.println(F(" "));
  Serial.print(F("AddH : "));
  Serial.println(configuration.ADDH, HEX);
  Serial.print(F("AddL : "));
  Serial.println(configuration.ADDL, HEX);
  Serial.println(F(" "));
  Serial.print(F("Chan : "));
  Serial.print(configuration.CHAN, DEC);
  Serial.print(" -> ");
  Serial.println(configuration.getChannelDescription());
  Serial.println(F(" "));
  Serial.print(F("SpeedParityBit     : "));
  Serial.print(configuration.SPED.uartParity, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTParityDescription());
  Serial.print(F("SpeedUARTDatte     : "));
  Serial.print(configuration.SPED.uartBaudRate, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTBaudRateDescription());
  Serial.print(F("SpeedAirDataRate   : "));
  Serial.print(configuration.SPED.airDataRate, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getAirDataRateDescription());
  Serial.println(F(" "));
  Serial.print(F("OptionSubPacketSett: "));
  Serial.print(configuration.OPTION.subPacketSetting, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getSubPacketSetting());
  Serial.print(F("OptionTranPower    : "));
  Serial.print(configuration.OPTION.transmissionPower, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getTransmissionPowerDescription());
  Serial.print(F("OptionRSSIAmbientNo: "));
  Serial.print(configuration.OPTION.RSSIAmbientNoise, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getRSSIAmbientNoiseEnable());
  Serial.println(F(" "));
  Serial.print(F("TransModeWORPeriod : "));
  Serial.print(configuration.TRANSMISSION_MODE.WORPeriod, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription());
  Serial.print(F("TransModeEnableLBT : "));
  Serial.print(configuration.TRANSMISSION_MODE.enableLBT, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.TRANSMISSION_MODE.getLBTEnableByteDescription());
  Serial.print(F("TransModeEnableRSSI: "));
  Serial.print(configuration.TRANSMISSION_MODE.enableRSSI, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription());
  Serial.print(F("TransModeFixedTrans: "));
  Serial.print(configuration.TRANSMISSION_MODE.fixedTransmission, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());


  Serial.println("----------------------------------------");
}

void printModuleInformation(struct ModuleInformation moduleInformation) {
  Serial.println("----------------------------------------");
  Serial.print(F("HEAD: "));
  Serial.print(moduleInformation.COMMAND, HEX);
  Serial.print(" ");
  Serial.print(moduleInformation.STARTING_ADDRESS, HEX);
  Serial.print(" ");
  Serial.println(moduleInformation.LENGHT, DEC);

  Serial.print(F("Model no.: "));
  Serial.println(moduleInformation.model, HEX);
  Serial.print(F("Version  : "));
  Serial.println(moduleInformation.version, HEX);
  Serial.print(F("Features : "));
  Serial.println(moduleInformation.features, HEX);
  Serial.println("----------------------------------------");
}
