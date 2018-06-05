Author: Yasir Aslam Shah
IOT Embedded Firmware
Blue Gecko Bluetooth SDK 2.7
SPring 2018


# Wireless-Vehicle-Dash-Board
WIreless Vehicle DAsh Board Server Module

This project deals with wirelessly steering a vehicle and using environmental sensors
to adjust the drive in various kinds of environmental variations (Ambient
light, Humidity and Temperature).
 In this project, I am implementing two BG boards, Board A is my server with all the sensors, including
Accelerometer,Light sensor and Humidity sensor.The client on the other board recives the data from the
sensors as Bluetooth service.

 The project aims at controlling a steering wheel of a vehicle wirelessly using
accelerometer, Bluetooth and environmental sensors (Humidity, Ambient
Light* and Temperature Sensor).The LCD on both the BGs will represent Vehicle Dash
Board with Data indicating the ambient cadence, pressure and environmental
humidity.
Update on Description:
 Sensor data is represented as a state on LCD. Each sensor has two states for the client.
 BMA280 is represented as “Accelerating” and ”Non-Accelerating”
 Light Sensor is represented as “Day Light” and “Night” and is used as a service to the client
 Humidity sensor is represented as “Humid” and ”Dry” and is used in Persistent Memory Application
