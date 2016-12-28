# ESP Motion Capture
Motion capturing and its translation to a virtual object through cloud using ESP8266 and MPU6050 sensor

## Essential changes
Download the ESP8266httpUpdate.cpp file and replace the existing file available at  
> ..\packages\esp8266\hardware\esp8266\2.3.0\libraries\ESP8266httpUpdate\src
This changes helps us to publish the status of FOTA to cloudmqtt before the ESP restarts

Replace 
```
#include <avr/pgmspace.h> 
```
with 
```
 #include <avr/pgmspace.h>
 #else
 #include <pgmspace.h>
 #endif
```
in the following files: MPU6050.h, MPU6050_6Axis_MotionApps20.h, MPU6050_9Axis_MotionApps41.h

## Working prototype video

## Project Description
Please read wiki for explanation of the project
