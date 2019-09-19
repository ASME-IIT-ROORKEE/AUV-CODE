# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import serial #Serial imported for Serial communication
import time #Required to use delay functions
 
ArduinoSerial = serial.Serial('com6',9600) #Create Serial port object called arduinoSerialData
time.sleep(2) #wait for 2 secounds for the communication to get established

print(ArduinoSerial.readline())#read the serial data and print it as line 
while 1: #Do this forever

    var = "0" #get input from user
    #print(var)
    ArduinoSerial.write(str.encode(var)) #send 1
    print(ArduinoSerial.readline())#read the serial data and print it as line 
    time.sleep(.1)
