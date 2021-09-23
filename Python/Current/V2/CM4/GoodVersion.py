from __future__ import print_function
from threading import Thread
import cv2
from imutils.video import FPS
import imutils
import time

# for Messaging
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish


# for Teensy
import serial

# for data reading
import numpy as np

# for thread messaging
import queue

messageQueue = queue.Queue()


## for buttons
import RPi.GPIO as GPIO  
GPIO.setmode(GPIO.BCM)  
GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_UP)




# Globals BAD
running = 1
printTrue = 0

#Thread listens for messages from the teensy and relays them to the Pi4
class TeensyListener(Thread):
    def __init__(self, src=0):
        # initialize the variable used to indicate if the thread should
        # be stopped
        self.stopped = False
   
    def start(self):
        self.ser = serial.Serial (
            port = '/dev/ttyACM0',
            baudrate = 57600,
            parity = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            bytesize = serial.EIGHTBITS,
            timeout = 1)
             
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()
        return self
    
    def run(self):
        global newData
        global dataSend
        print('Starting Serial Listener')
 
        printTrue = True

    def update(self):        
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return

            # if data available, read and put in message queue
            if self.ser.in_waiting > 2 and self.ser.isOpen():
                msg = self.ser.readline().decode()
                
                messageQueue.put(msg)
                
                if printTrue == True:
                    print('BLA')

                newData=True
            else:
#                print('ackkk')
                msg = "D\r"
                self.ser.write(msg.encode())
                time.sleep(0.01)

#Thread polls the camera at set frame rate and updates the screen
class VideoStream:
    def __init__(self, src=0):
        # initialize the video camera stream and read the first frame
        # from the stream
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE,2)
        (self.grabbed, self.frame) = self.stream.read()
        # initialize the variable used to indicate if the thread should
        # be stopped
        self.stopped = False
        self.FPS = 1/30
        self.FPS_MS = int(self.FPS * 1000)
        GPIO.add_event_detect(26, GPIO.FALLING, callback=self.send_snapshot,bouncetime = 150)
        
    # sends the current frame to the message queue
    def send_snapshot(self):
        fileName = "img-"+ str(current_milli_time()) +".jpg"
        cv2.imwrite(fileName,self.frame)
        f = open(fileName,"rb")
        data = f.read()
        byteArr = bytearray(data)
        messageQueue.put(byteArr)       

    def start(self):
        # start the thread to read frames from the video stream
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()
        return self
        
    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
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
        cv2.imshow('frame',self.frame)
        #self.newPic = False
        cv2.waitkey(self.FPS_MS)
    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True	
        
        
class MessageStream:
    def __init__(self):
        self.PI4_SERVER = "192.168.0.139"
        self.PI4_PATH2 = "piz2-pi4-data"
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()
        return self
 
    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return

            print(messageQueue.qsize())
            if messageQueue.qsize() > 0:
                msg = messageQueue.get()
                publish.single(self.PI4_PATH2,msg,hostname = self.PI4_SERVER)

            time.sleep(0.1)

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True	        
        


def main():  
    #start Camera grabbing     
    vs = VideoStream(src=0).start()
    fps = FPS().start()

    #start arduino listener
    ts = TeensyListener().start()

    # start message Stream
    ms = MessageStream().start()


    while(running):
        try:
            vs.show_frame()
        except AttributeError: 
            pass
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        fps.update()
    fps.stop()
    print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
    cv2.destroyAllWindows()
    vs.stop()
    GPIO.cleanup()
    print('Closing Main')

if __name__ == '__main__':
    main()
