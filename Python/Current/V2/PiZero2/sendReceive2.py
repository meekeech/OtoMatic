import time 
import serial #import serial and time libraries
import numpy as np 
import datetime as dt 
import threading
import sys
import struct
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import pygame
import pygame.camera
from pygame.locals import *
import os
import RPi.GPIO as GPIO  

GPIO.setmode(GPIO.BCM)  
GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_UP)
 
pygame.init()
pygame.camera.init()
#cam = pygame.camera.Camera("/dev/video0",(400,380))
#cam.start()
 
count = 0
printTrue = True
dataRow = np.array([])
newData = False
running = 1

sendPhoto = False

current_milli_time = lambda: int(round(time.time() * 1000))

ser = serial.Serial (
        port = '/dev/ttyS0',
        baudrate = 115200,
        parity = serial.PARITY_NONE,
        stopbits = serial.STOPBITS_ONE,
        bytesize = serial.EIGHTBITS,
        timeout = 1
)



#PI4_SERVER = "192.168.0.139"
PI4_SERVER = "192.168.2.100" #home
PI4_PATH = "piz2-pi4-img"
PI4_PATH2 = "piz2-pi4-data"

    
def ReadData(data,numBytes,appendTrue,display):
    if numBytes == 1:
        readVal = ser.read(1)
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
    

class Capture(object):
    
    def __init__(self):
        #self.size = (352,288)
        self.size = (400,380)
        self.cam = pygame.camera.Camera("/dev/video0",self.size)
        self.cam.start()

        
    def get_snapshot(self):
        #Must call three times or the photo won't update
        self.snapshot = self.cam.get_image()
        self.snapshot = self.cam.get_image()
        self.snapshot = self.cam.get_image() 
        #timestr = time.strftime("%Y%m%d-%H%M%S")
        timestr = str(current_milli_time())
        fileName = "img-"+timestr+".jpg"
        pygame.image.save(self.snapshot,fileName)
        f = open(fileName,"rb")
        fileContent = f.read()
        byteArr = bytearray(fileContent)
        publish.single(PI4_PATH,byteArr,hostname = PI4_SERVER)

class serialListener(threading.Thread):
  
    def run(self):
        global newData
        global dataSend
        print('Starting Serial Listener')
        #ser.open() #begins serial communication
        printTrue = True
        
        while(running):
            time.sleep(0.01)
            #print(ser.in_waiting)

            if ser.in_waiting > 4 and ser.isOpen():
                dataRow = np.array([])
                dataRow = ReadData(dataRow,1,0,printTrue) #B
                dataRow = ReadData(dataRow,1,1,printTrue) #Second Button State
                dataRow = ReadData(dataRow,1,1,printTrue) #Third Button State
                dataRow = ReadData(dataRow,1,0,printTrue) #M
                dataRow = ReadData(dataRow,1,1,printTrue) # Motor Direction

                
                
                if (dataRow[0].decode() == '1'):
                    sendData = str(dataRow[0].decode())+str(dataRow[1].decode())+str(dataRow[2].decode())
                    #sendArray = bytearray([dataRow[2],dataRow[3]])
                    #read pressure and send to Rpi4
                    publish.single(PI4_PATH2,sendData,hostname = PI4_SERVER)
                    #print("a")
                    
                if printTrue == True:
                    print('')
                # Clear the buffer
                #readByte = ser.readline()
                
                
                newData=True


def handle_close(evt):
    global running
    print('Closed')
    running = 0

def my_callback(channel): 
    global sendPhoto
    sendPhoto = True

def main():
    
    GPIO.add_event_detect(26, GPIO.FALLING, callback=my_callback,bouncetime = 150) 
    global running
    global sendPhoto
    capture = Capture()
    
    
    serialListener().start()
    
    while running:
        if sendPhoto is True:
            capture.get_snapshot()
            sendPhoto = False
            
        
    cam.stop()
    GPIO.cleanup()
        
    client.loop_stop()
    print('Closing Main')

if __name__ == '__main__':
    main()
