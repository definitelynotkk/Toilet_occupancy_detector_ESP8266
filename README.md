# Toilet Ocuppancy Detector

**This project is still in progress and need much refinement.**  
  
My new solid wood toilet door is so thick and soundproof that nobody outside can tell if it is occupied.   
To avoid awkwardness, I decided to make this sensor.  
   
It is a smart home sensor peoject based on Arduino using 2 VL53L0X Time Of Flight sensor, detect people moving in and out. Turn on LED indicator when toilet is ocuppied. Send MQTT message to trigger other home automation actions.  
  
Only tested on esp8266 nodemcu. Other arduino compatible board should also work.  
  
### Arduino Library required:
PubSubClient  
SimpleTimer


only support one person crossing at a time.  
scenarios:  
1. enter  &radic;working  
sensor1:-----111111--------  
sensor2:---------111111----  
2. leave  &radic;working  
sensor1:--------1111111----  
sensor2:----1111111--------  
3. half way turn back  
//NOT IMPLEMENTED YET  
sensor1:----1111111-------  
sensor2:------1111--------  
4. if sensors is far from each other  
//should not work  
//but this sensor has a FOV of 25 degree, it will work in some cases.  
sensor1:----111111--------------  
sensor2:-------------1111111----  
  
### Features
- Turn on led indicator when someone enters  
- Send number of people to mqtt server for other smart home action integration  
- Button press to reset number of people to 0. Long press to reboot the board  
- Home assistant and node-red support  

### File structure
/src/main.cpp --- Arduino code
nodered_flow.json  --- Simple time dependent light control nodered flow
wiring_diagram.jpg --- wiring
  
### Installation
Connect sensors as shown. LED and button are optional.
![image](https://raw.githubusercontent.com/definitelynotkk/Toilet_occupancy_detector_ESP8266/master/schematic.png)

Modify the wifi and mqtt information in the code, and flash into your board using arduino or platform.io  

Before install onto wall, check sensor direction first.  

### Todolist
1. add half way turn back scenario   
2. add reed switch to wake up TOF sensor  
3. RGB LED support 
4. Preserve status when reboot  
5. 3D printed case  
6. sensor and board watchdog..
7. ..
