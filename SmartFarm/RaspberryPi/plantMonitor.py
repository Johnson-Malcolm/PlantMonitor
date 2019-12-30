#!/usr/bin/env python
from datetime import datetime
import json
import time
import serial

#Define the serial communication
ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate = 57600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=None # waits until recieve serial
)

#convert moisture char into a string
def getmoisture(a):
    if a == 'w':
        return 'wet'
    elif a == 'm':
        return 'moist'
    else:
        return 'dry'

#writes to data.txt
def writefile(command):
    d = datetime.today()
    date = d.strftime('%d/%m/%Y') #date in day/month/year format
    moisture = getmoisture(command[1])
    str = "\n"+date+' '+moisture+' '+command[2]+' '+command[3]
    print(str)
    f = open('data.txt', 'a')
    f.write(str)

#Creates The json file
def jsonwrite(x):
    values = list()
    with open("data.txt", "r") as f:
        data = f.readlines()
        for line in data:
            words = line.split()
            d = {"date": str(words[0]), "moisture": str(words[1]), "temperature":int(words[2]), "water":int(words[3])}
            values.append(d)

    data = {}
    data ['plantId'] = int(1) #for future development multiple plants
    data ['plantData'] = values

    with open('data.js', 'w') as outfile:
        outfile.write("data =")
        json.dump(data,outfile, indent=4)

#Processes the received json data
def process(x):
    command = x.split(' ')
    print(command[0])
    if 'push' in command[0]:
        writefile(command)
        jsonwrite(x)

while 1:
    x=ser.readline() #waits until new serial receive
    y=str(x)
    process(y)
