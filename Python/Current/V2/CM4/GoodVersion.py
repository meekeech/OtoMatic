from __future__ import print_function
from threading import Thread
import cv2
from imutils.video import FPS
import imutils
import time
import numpy as np
# for Messaging
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish


# for Teensy
import serial

# for thread messaging
import queue

messageQueue = queue.Queue()
imageQueue = queue.Queue()


## for buttons
import RPi.GPIO as GPIO  
GPIO.setmode(GPIO.BCM)  
GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_UP)




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
                msg = 'Test' + msg
                messageQueue.put(msg)
                #print(msg)
                
                if printTrue == True:
                    print('BLA')

                newData=True
            else:
#                print('ackkk')
                msg = "D\r"
                self.ser.write(msg.encode())
                time.sleep(0.1)

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
        imageQueue.put(byteArr) 
        print('Sending Image')      

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
        self.dataMSG = "piz2-pi4-data"
        self.imgMSG = "piz2-pi4-img"
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
            while (messageQueue.qsize() > 0):
                msg = messageQueue.get()
                publish.single(self.dataMSG,msg,hostname = self.PI4_SERVER)
                messageQueue.task_done()
                #print(messageQueue.qsize())

            while (imageQueue.qsize() > 0):
                msg = imageQueue.get()
                publish.single(self.imgMSG,msg,hostname = self.PI4_SERVER)
                imageQueue.task_done()

            time.sleep(0.1)

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True	        


def main():  
    #start Camera grabbing     
    vs = VideoStream(src=-1).start()
    fps = FPS().start()

    #start arduino listener
    ts = TeensyListener().start()

    # start message Stream
    ms = MessageStream().start()
    t0 = time.time()

    while(running):
        try:
            vs.show_frame()
        except AttributeError: 
            pass
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        fps.update()
        t1 = time.time()
        if (t1-t0) >= 1.0:
            t0 = t1
            vs.send_snapshot()
    fps.stop()
    print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
    cv2.destroyAllWindows()
    vs.stop()
    GPIO.cleanup()
    print('Closing Main')

if __name__ == '__main__':
    main()
