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



PI4_SERVER = "192.168.0.139"
#PI4_SERVER = "192.168.2.100" #home
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
        self.display = pygame.display.set_mode(self.size, 0)
        self.cam = pygame.camera.Camera("/dev/video0",self.size)
        self.cam.start()
        self.snapshot = pygame.surface.Surface(self.size, 0, self.display)
        GPIO.add_event_detect(26, GPIO.FALLING, callback=self.get_snapshot,bouncetime = 150) 
        self.sendTrue = False
        
    def get_and_flip(self):
        if self.cam.query_image():
            self.snapshot = self.cam.get_image()
        self.display.blit(self.snapshot, (0,0))
        pygame.display.flip()
        
    def get_snapshot(self,channel):
        self.snapcopy = self.snapshot
        self.timestr = str(current_milli_time())
        self.sendTrue = True
            
    def send_snapshot(self):
        if self.sendTrue is True:
            fileName = "img-"+self.timestr+".jpg"
            pygame.image.save(self.snapcopy,fileName)
            f = open(fileName,"rb")
            fileContent = f.read()
            byteArr = bytearray(fileContent)
            publish.single(PI4_PATH,byteArr,hostname = PI4_SERVER)
            self.sendTrue = False

class serialListener(threading.Thread):
  
    def run(self):
        global newData
        global dataSend
        print('Starting Serial Listener')
        #ser.open() #begins serial communication
        printTrue = False
        doOnce = True
        
        while(running):
            time.sleep(0.01)
            #print(ser.in_waiting)

            if ser.in_waiting > 0 and ser.isOpen():
                dataRow = np.array([])
                dataRow = ReadData(dataRow,1,1,printTrue) #D or M or T (data mic or temp)
                
                if (dataRow[0].decode() == 'D'):
                    dataRow = ReadData(dataRow,1,1,printTrue) #Motor Dir State
                    sendData = str(dataRow[0].decode())+str(dataRow[1].decode())
                    
                elif (dataRow[0].decode() == 'M'):   
                    dataRow = ReadData(dataRow,1,1,printTrue) #comma 
                    dataRow = ReadData(dataRow,2,1,printTrue) #Microhpone data
                    dataRow = ReadData(dataRow,1,1,printTrue) #comma
                    dataRow = ReadData(dataRow,2,1,printTrue) #Pressure data
                    sendData = str(dataRow[0].decode())+str(dataRow[1].decode())+str(dataRow[2].decode())+str(dataRow[3].decode())+str(dataRow[4].decode())
                    
                    
                elif (dataRow[0].decode() == 'T'):
                    dataRow = ReadData(dataRow,1,1,printTrue) #comma 
                    dataRow = ReadData(dataRow,2,1,printTrue) # temp
                    sendData = str(dataRow[0].decode())+str(dataRow[1].decode())+str(dataRow[2].decode())
                    
                elif (dataRow[0].decode() == 'X'):
                    sendData = str(dataRow[0].decode())
                    print("done tymp")
                    
                elif (dataRow[0].decode() == 'C'):
                    sendData = str(dataRow[0].decode())
                    print("done sending")
                    
                
                publish.single(PI4_PATH2,sendData,hostname = PI4_SERVER)

                    
                if printTrue == True:
                    print('')
                # Clear the buffer
                #readByte = ser.readline()
                newData=True
            

#def my_callback(channel): 
    #global sendPhoto
    #sendPhoto = True

def main():
    
    
    global running
    global sendPhoto
    capture = Capture()
    
    
    serialListener().start()
    
    while running:
        
        capture.get_and_flip()
        capture.send_snapshot()
        
        events = pygame.event.get()
        for e in events:
            if e.type == QUIT or (e.type == KEYDOWN and e.key == K_ESCAPE):
                capture.cam.stop()
                going = False
                GPIO.cleanup()
                

            
        
    cam.stop()
    
    GPIO.cleanup()
        
    client.loop_stop()
    print('Closing Main')

if __name__ == '__main__':
    main()
