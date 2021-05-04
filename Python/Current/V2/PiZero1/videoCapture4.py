import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import time
import pygame
import pygame.camera
import math
import RPi.GPIO as GPIO  
from pygame.locals import *
import paho.mqtt.publish as publish

PI4_SERVER = "192.168.0.139"
#PI4_SERVER = "192.168.2.100" #home
PI4_PATH = "piz1-pi4"


GPIO.setmode(GPIO.BCM)  
GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_UP)

pygame.init()

pygame.camera.init()

sendPhoto = False

current_milli_time = lambda: int(round(time.time() * 1000))

class Capture(object):
    
    def __init__(self):
    #400,380
        #self.size = (352,288)
        self.size = (400,380)
        # create a display surface. standard pygame stuff
        self.display = pygame.display.set_mode(self.size, pygame.FULLSCREEN)
        #self.display = pygame.display.set_mode(self.size, 0) 
        self.cam = pygame.camera.Camera("/dev/video0",self.size)
        self.cam.start()
        self.snapshot = pygame.surface.Surface(self.size, 0, self.display)
        GPIO.add_event_detect(26, GPIO.FALLING, callback=self.get_snapshot,bouncetime = 150) 
        self.sendTrue = False
        
    def get_and_flip(self):
        if self.cam.query_image():
            self.snapshot = self.cam.get_image(self.snapshot)
        self.snapCopy = pygame.surfarray.array3d(self.snapshot)
        self.snapCopy = self.snapCopy.transpose([1,0,2])
        img_bgr = cv2.cvtColor(view,cv2,COLOR_RGB2BGR)
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

    
 
def main():
    #global sendPhoto  
    capture = Capture()
    going = True
    
    while going:
        capture.get_and_flip()
        capture.send_snapshot()
        
        events = pygame.event.get()
        for e in events:
            if e.type == QUIT or (e.type == KEYDOWN and e.key == K_ESCAPE):
                capture.cam.stop()
                going = False
                GPIO.cleanup()
     
if __name__ == '__main__':
    main()
     
