Remote-Relay-with-Battery-Monitor

Project is a remote relay with battery monitoring; that uses two, EByte E220-900T30D modules that have the ability to 
tranmit a WOR (Wake On Radio) signal to wake receiver and the ESP32 from deep sleep battery monitor with NTP timestamp 
transmitted to E220 receiver. 

Project logs timestamp, adc reading, and voltage to a file. Going to find out how long battery lasts and the number 
of requests made using an automated method of generating GET requests. conditional statement determine how often request 
are made to AsycWebServer. URL request to server switches “ON” live video camera for a predetermined period using a once 
Ticker timer method.  When countdown timer expires; video camera is switched “OFF”.

Before running project sketches; E220 transeceivers need to configured by running the two configuration sketches on both 
the "Sender" and the "Receiver".  

Suggest labeling transceivers; one "Sender" and the other "Receiver".

FTP sketch can be used to retrive the logging file; for viewing data from the battery monitoring data. FTP sketch was tested
using "Filezilla Client".

Project is a work in progress...
