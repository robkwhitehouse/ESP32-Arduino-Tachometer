# ESP32-Arduino-Tachometer

***  In Development - this code still has serious bugs ***

Simple rev counter uses an ESP32 with arduino core and an IR proximity sensor. Output via Bluetooth Serial.

The hardware setup expected by this sketch is an ESP32 dev module loaded with the arduino core.
This has an Infrared Sensor module attached, with the output connected to GPIO PIN 15.
It uses the Bluetooth Serial library to simply send out the currently measured RPM.
This works well with the android  "Serial Bluetooth Terminal" app. (on Google Play).
The Bluetooth device name is "Rev Counter".

It is currently configured to read up to about 6000rpm but could be made to go a lot faster by
changing the SAMPLE_RATE constant.

It is intended that the sensor is pointed at a rotating disc, one half of which is painted white, and
the other half black. This could equally be a rotating shaft. The IR sensor needs to be within a few cm
of the rotating object.



