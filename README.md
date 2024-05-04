Remote-Relay-with-Battery-Monitor

Progress is good on project; mostly complete, want to add two-way messaging, that will come later.  Now the project uses two, EByte E220-900T30D transceivers to wake a sleeping ESP32; first a WOR preamble message.  Next when a person sends a URL request to view camera; the solar charged, battery power is switched on with a MOSFET, KY002S switch.  Happening at same time a countdown timer is started to turn off battery power when timer expires.  When battery power is turned on; battery status is checked using a INA226 battery monitor module.  Battery data is logged to a file.  Accessing log file is accomplished by a stand-alone FTP sketch to allow access for viewing battery data file.
Camera is a Wyse Cam v3 that draws 180 mA; that until now used battery 24/7 on a 10,000 mAH power bank that only lasted about day.  This improvement; should conserve the majority of lost battery power; since camera will now only be on by URL request and on-time limited.

Suggest labeling transceivers; one "Sender" and the other "Receiver".

Project is a work in progress...

[Short video of project:](https://drive.google.com/file/d/14rA51U5Aa5nzgZzr-EgNdHrJfonFd7Vr/view?usp=sharing)
