from __future__ import print_function

import threading
from threading import Thread
import cv2
from imutils.video import FPS
import imutils
import time
import numpy as np
# for Messaging
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
#import signal 
#import atexit

# for Teensy
import serial

# for thread messaging
import queue
import sys
import signal

messageQueue = queue.Queue()
imageQueue = queue.Queue()


## for buttons
#import RPi.GPIO as GPIO  
#GPIO.setmode(GPIO.BCM)  
#GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_UP)


## for colour Correction
currH = 1
currS = 1
currV = 1
changeType = 'hsv'
currR = 1
currG = 1
currB = 1

# Globals BAD
running = 1
printTrue = 0

def current_milli_time():
    return round(time.time() * 1000)

#Thread listens for messages from the teensy and relays them to the Pi4
class TeensyListener(Thread):
    def __init__(self, src=0):
        # initialize the variable used to indicate if the thread should
        # be stopped
        self.stopped = False
        self.connected = False
   
    def start(self):
        self.ser = serial.Serial (
            port = '/dev/ttyAMA0',
            baudrate = 115600,
            parity = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            bytesize = serial.EIGHTBITS,
            timeout = 0)
             
        self.ser.reset_input_buffer()
        
        if self.connect() is True:
            self.ser.reset_input_buffer()
            self.thread = Thread(target=self.update, args=())
            self.thread.daemon = True
            self.thread.start()
            print('Starting Serial Listener')
            return self
        else:
            return
        
        
    def connect(self):
        
        try:
            self.ser.write('A'.encode('utf-8'))
        except SerialTimeoutException:
            print("Timeout Exception")
            return False
            
        time.sleep(0.5)
     
        
        # while self.connected is False:
        if self.ser.in_waiting > 0:
            connect_msg = self.ser.readline().decode()
            if connect_msg[0] == 'B':
                self.connected = True
                return True
            else: 
                print("No Return Message")
                return False
        else:
            print("Nothing in Buffer")
                # return False

    def update(self):        
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return

            # if data available, read and put in message queue
            if self.ser.in_waiting > 0 and self.ser.isOpen():
                msg = self.ser.readline().decode()
                msg1 = msg
                messageQueue.put(msg1)
                print(msg1)
                    
                #if printTrue == True:
                    #print('BLA')

                newData=True
            else:
                pass
                #print('ackkk')
                #msg = "D\r"
                #self.ser.write(msg.encode())
            time.sleep(0.01)
    def stop(self):
        # indicate that the thread should be stopped
        try:
            self.ser.write('C'.encode('utf-8'))
        except SerialTimeoutException:
            print("Timeout Exception in Stop Serial")
        self.ser.close()
        print('Closed Serial')
        self.stopped = True	

def HChange(new_val):
    global currH, changeType
    currH = new_val/100
    changeType = 'hsv'
    
def SChange(new_val):
    global currS, changeType
    currS = new_val/100
    changeType = 'hsv'
    
def VChange(new_val):
    global currV, changeType
    currV = new_val/100
    changeType = 'hsv'

def RChange(new_val):
    global currR, changeType
    currR = new_val/100
    changeType = 'bgr'
    
def GChange(new_val):
    global currG, changeType
    currG = new_val/100
    changeType = 'bgr'    

def BChange(new_val):
    global currB, changeType
    currB = new_val/100
    changeType = 'bgr'
        
#Thread polls the camera at set frame rate and updates the screen
class VideoStream(threading.Thread):
    def __init__(self, src=0):
       #super(Job, self).__init__(*args, **kwargs)
        # ~ super(VideoStream, self).__init__(src)
        super().__init__()
        self.flag = threading.Event()
        self.flag.set()
        self.running = threading.Event()
        self.running.set()
        
        # initialize the video camera stream and read the first frame
        # from the stream
        self.stream = cv2.VideoCapture(0)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE,2)
        (self.grabbed, self.frame) = self.stream.read()
        cv2.namedWindow("PIC", cv2.WND_PROP_FULLSCREEN)
 #       cv2.setWindowProperty("PIC", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        # create Colour Adjusters
        # ~ cv2.createTrackbar("H","PIC", 100, 200, HChange)
        # ~ cv2.createTrackbar("S","PIC", 100, 200, SChange)
        # ~ cv2.createTrackbar("V","PIC", 100, 200, VChange)
        # ~ cv2.createTrackbar("R","PIC", 100, 200, RChange)
        # ~ cv2.createTrackbar("G","PIC", 100, 200, GChange)
        # ~ cv2.createTrackbar("B","PIC", 100, 200, BChange)

        # initialize the variable used to indicate if the thread should
        # be stopped
        self.stopped = False
        self.paused = False
        self.FPS = 1/30
        self.FPS_MS = int(self.FPS * 1000)
        
    # sends the current frame to the message queue
    def send_snapshot(self):
        fileName = "img-"+ str(current_milli_time()) +".jpg"
        cv2.imwrite(fileName,self.frame)
        f = open(fileName,"rb")
        data = f.read()
        byteArr = bytearray(data)
        imageQueue.put(byteArr) 
        #print('Sending Image')      

    def start(self):
        # start the thread to read frames from the video stream
        self.thread = Thread(target=self.run, args=())
        self.thread.daemon = True
        self.thread.start()
        return self
        
    def run(self):
        # keep looping infinitely until the thread is stopped
        while self.running.isSet():
            self.flag.wait()
            # if the thread indicator variable is set, stop the thread
            if self.stopped: 
                return
            # otherwise, read the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()
            #self.newPic = True
            time.sleep(self.FPS)
    
    def read(self):
        # return the frame most recently read
        return self.frame
    def show_frame(self):
        #if self.stopped is False and self.newPic is True:
        
        # ~ if changeType == 'hsv':
            # ~ pic_temp = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
            # ~ dim1,dim2,dim3 = cv2.split(pic_temp)
            # ~ dim1 = np.array(dim1*currH,dtype = np.float64) 
            # ~ dim1 = np.clip(dim1,0,180) #use 180 for hue
            # ~ dim2 = np.array(dim2*currS,dtype = np.float64) 
            # ~ dim2 = np.clip(dim2,0,255)
            # ~ dim3 = np.array(dim3*currV,dtype = np.float64) 
            # ~ dim3 = np.clip(dim3,0,255)
            # ~ pic2 = cv2.merge([np.uint8(dim1) , np.uint8(dim2) , np.uint8(dim3)])
            # ~ pic2 = cv2.cvtColor(pic2, cv2.COLOR_HSV2BGR)
            
        # ~ if changeType == 'bgr':
            # ~ dim1,dim2,dim3 = cv2.split(self.frame)
            # ~ dim1 = np.array(dim1*currB,dtype = np.float64) 
            # ~ dim1 = np.clip(dim1,0,255)
            # ~ dim2 = np.array(dim2*currG,dtype = np.float64) 
            # ~ dim2 = np.clip(dim2,0,255)
            # ~ dim3 = np.array(dim3*currR,dtype = np.float64) 
            # ~ dim3 = np.clip(dim3,0,255)
            # ~ pic2 = cv2.merge([np.uint8(dim1) , np.uint8(dim2) , np.uint8(dim3)])
        
        # ~ cv2.imshow('PIC',pic2)
        cv2.imshow('PIC',self.frame)
        #self.newPic = False
        cv2.waitkey(self.FPS_MS)
    
    def pause(self):
        print("pausing")
        self.flag.clear()
        
    def resume(self):
        print("resuming")
        self.flag.set()    
        
    def stop(self):
        # indicate that the thread should be stopped
        print('Closing Video Stream')
        self.stopped = True	
        self.flag.set() # Resume the thread from the suspended state, if it is already suspended
        self.running.clear() # Set to False
        
        
        
class MessageStream:
    def __init__(self):
        self.PI4_SERVER = "192.168.0.139"
        self.dataMSG = "piz2-pi4-data"
        self.imgMSG = "piz2-pi4-img"
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()
        print('Starting MQTT')
        return self
 
    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return

            # ~ if messageQueue.qsize() > 0:
                # ~ print(messageQueue.qsize())
            while (messageQueue.qsize() > 0):
                msg = messageQueue.get()
                publish.single(self.dataMSG,msg,hostname = self.PI4_SERVER)
                messageQueue.task_done()
                #print(messageQueue.qsize())

            # ~ while (imageQueue.qsize() > 0):
                # ~ msg = imageQueue.get()
                # ~ publish.single(self.imgMSG,msg,hostname = self.PI4_SERVER)
                # ~ imageQueue.task_done()

            time.sleep(0.1)

    def stop(self):
        # indicate that the thread should be stopped
        print("Stopping MQTT")
        self.stopped = True	    
        

signal.signal(signal.SIGINT, signal.default_int_handler)


def main():  
    #start Camera grabbing     
    vs = VideoStream(src=0).start()
    #fps = FPS().start()
    
    #start arduino listener
    # ~ ts = TeensyListener().start()

    # start message Stream
    # ~ ms = MessageStream().start()
    t0 = time.time()
    show = True

    while(running):
        pass
        try:
            pass
            if show is True:
                vs.show_frame()
        except AttributeError: 
            pass
            
        inputVal = cv2.waitKey(1) & 0xFF    
            
        if inputVal == ord('p'):
            #vs.pause()
            print("fps")
            show = False
            
        
        elif inputVal == ord('r'):
            #vs.resume()
            show = True
            
        elif inputVal == ord('q'):
            break
        #fps.update()
        try:
            t1 = time.time()
            if (t1-t0) >= 1.0:
                t0 = t1
        except KeyboardInterrupt:
            ts.stop()
            ms.stop()
            sys.exit()
            
        
            #vs.send_snapshot()
    #fps.stop()
    #print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
    #print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
    cv2.destroyAllWindows()
    vs.stop()
    # ~ ts.stop()
    # ~ ms.stop()
    #GPIO.cleanup()
    print('Closing Main')

if __name__ == '__main__':
    main()
