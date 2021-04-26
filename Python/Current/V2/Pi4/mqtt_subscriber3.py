import paho.mqtt.client as mqtt
import time
import RPi.GPIO as GPIO
import numpy as np
import sys
import serial 
import threading
import struct
import queue
import csv
import numpy as np 
import datetime 
from signal import signal, SIGINT
from sys import exit
#import matplotlib.pyplot as plt 
#import matplotlib.animation as animation 
#from matplotlib.animation import FuncAnimation


fileName = datetime.datetime.now().strftime('csvOutput/oto_matic-%Y-%m-%d-%H-%M.') 
saveQueue = queue.Queue()
tempQueue = queue.Queue()
doOnce = True
currentTime = 0
prevTime = 0
running = 1
running1 = 1
count1 = 1
count2 = 1
firstTime = True
getOut = False

 
PI4_SERVER = "localhost"
PI4_PATH = [("piz1-pi4",0),("piz2-pi4-img",0),("piz2-pi4-data",0)]
#m = ''

ser = serial.Serial (
        port = '/dev/nano1',
        baudrate = 115200,
        parity = serial.PARITY_NONE,
        stopbits = serial.STOPBITS_ONE,
        bytesize = serial.EIGHTBITS,
        timeout = 1
)
 
# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
 
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(PI4_PATH)
 
# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global prevTime
    global count1 
    global count2 
    global running1
    
    if (msg.topic == "piz1-pi4"):
        fileName1 = "img1." + str(count1)
        count1 = count1 + 1
        f = open(fileName1,"wb")
        f.write(msg.payload)
        print("Image1 received")
        f.close()
    elif (msg.topic == "piz2-pi4-img"):
        fileName2 = "img2." + str(count2)
        count2 = count2 + 1
        f2 = open(fileName2,"wb")
        f2.write(msg.payload)
        print("Image2 received")
        f2.close()
    elif (msg.topic == "piz2-pi4-data"):
        vals = msg.payload
        #print(vals,end = ',')
        val1 = chr(vals[0]) # D or M or T or X or C
        if (val1 == 'X'):
            ser.write('s'.encode())
        elif (val1 == 'D'):
            val2 = chr(vals[1]) #0 or 1 for motor dir
            if (val2 == '0'):
                ser.write('f'.encode())
            elif (val2 == '1'):
                ser.write('r'.encode())
        elif (val1 == 'M'):
            saveQueue.put(vals.decode())
        elif (val1 == 'T'):
            tempQueue.put(vals.decode())
        elif (val1 == 'C'):
            #csv1.CloseFile()
            #firstTime = True
            #running1 = 0
            print('done transfer')
        
        
        
    

def InitializeMQTT():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
     
    client.connect_async(PI4_SERVER, 1883, 60)
    client.loop_start()

def ReadData(data,numBytes,appendTrue,display):
    if numBytes == 1:
        readVal = ser.read(1).decode()
        if display == True:
            print(readVal,end='')
    if numBytes == 2:
        readVal = ser.read(2)
        readVal = struct.unpack(">H",readVal)[0]
        if display == True:
            print(readVal,end='')
    if appendTrue == 1:
        data = np.append(data,[readVal])
        
    return data
    
class serialListener(threading.Thread):
  
    def run(self):
        global newData
        global dataSend
        print('Starting Serial Listener')
 
        printTrue = True
        
        while(running):
            time.sleep(0.01)
            #print(ser.in_waiting)

            if ser.in_waiting > 2 and ser.isOpen():
                dataRow = np.array([])
                dataRow = ReadData(dataRow,2,1,printTrue) #f
               
                   
                if printTrue == True:
                    print('')
                    
                # Clear the buffer
                #readByte = ser.readline()

                newData=True

class csvWriter(threading.Thread):
    
    #def __init__(self): 
        #threading.Thread.__init__(self)
        #print('Starting CSV Writer')  
        #with open(fileName + 'csv', 'a') as self.csvfile:
            #self.csvwriter = csv.writer(self.csvfile, dialect='excel',delimiter =',')
        
    def run(self):
        print('Starting CSV Writer')  
        global running1
        global getOut
        firstTymp = True
        
        with open(fileName + 'csv', 'a') as csvfile:
            csvwriter = csv.writer(csvfile, dialect='excel',delimiter =',')

            while (running1):
                if (getOut is True):
                    print("leaving")
                    return
                
                if saveQueue.qsize() > 0:
                    if firstTymp is True:
                        csvwriter.writerow(['Type','Mic','Pressure'])
                        firstTymp = False
                    
                    data = saveQueue.get()
                    data = data.split(",")
                    csvwriter.writerow(data)
                else:
                    time.sleep(0.01)
                if tempQueue.qsize() > 0:
                    data2 = tempQueue.get()                
                    csvwriter.writerow(['Type','Temp'])                                       
                    data2 = data2.split(",")
                    csvwriter.writerow(data2)
                    firstTymp = True
                                
  
def main():
    global getOut
    

    InitializeMQTT()
    time.sleep(0.5)
    csv1 = csvWriter()
    csv1.start()
    serialListener().start()
    
    while(running):
        value = input("Press S to save\n")
        if value is 'S' or value is 's':
            getOut = True 
   
    print('Closing Main')
    client.loop_stop()


if __name__ == '__main__':
    main()
