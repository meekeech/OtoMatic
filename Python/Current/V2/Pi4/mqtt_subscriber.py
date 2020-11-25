import paho.mqtt.client as mqtt
import time
import RPi.GPIO as GPIO
import numpy as np
import sys
import serial 
import threading
import struct

GPIO.setmode(GPIO.BCM)
GPIO.setup(2,GPIO.IN,pull_up_down=GPIO.PUD_UP)
doOnce = True
currentTime = 0
prevTime = 0
running = 1
 
PI4_SERVER = "localhost"
PI4_PATH = [("piz1-pi4",0),("piz2-pi4-img",0),("piz2-pi4-data",0)]
#m = ''

ser = serial.Serial (
        port = '/dev/ttyUSB2',
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
    if (msg.topic == "piz1-pi4"):
        f = open('output1.jpg',"wb")
        f.write(msg.payload)
        print("Image1 received")
        f.close()
    elif (msg.topic == "piz2-pi4-img"):
        f2 = open('output2.jpg',"wb")
        f2.write(msg.payload)
        print("Image2 received")
        f2.close()
    elif (msg.topic == "piz2-pi4-data"):
        #currentTime = int(round(time.time()*1000))
        #print(currentTime-prevTime)
        #print(currentTime-prevTime)
        #prevTime = currentTime
        #bufferedMsg = np.frombuffer(msg.payload,dtype='b')
        #print("Pi: ")
        #print(msg.payload)
        ser.write('f'.encode())
        
        
    

def InitializeMQTT():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
     
    client.connect_async(PI4_SERVER, 1883, 60)
    client.loop_start()
 
# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.



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



def main():
    InitializeMQTT()
    
    serialListener().start()
    
    while(running):
        #if GPIO.input(2) == 0 and doOnce == True:
            #print('0')
            #doOnce = False
            #prevTime = int(round(time.time()*1000))
            pass

   
    print('Closing Main')
    client.loop_stop()





if __name__ == '__main__':
    main()
