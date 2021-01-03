import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

import pygame
import pygame.camera
import math
import RPi.GPIO as GPIO  


from pygame.locals import*


import paho.mqtt.publish as publish
PI4_SERVER = "192.168.0.75"
PI4_PATH = "piz1-pi4"


GPIO.setmode(GPIO.BCM)  
GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_UP)

pygame.init()

pygame.camera.init()


class Capture(object):
    
    def __init__(self):
	#352,288
    #400,380
        self.size = (352,288)
        # create a display surface. standard pygame stuff
        #self.display = pygame.display.set_mode(self.size, pygame.FULLSCREEN)
        self.display = pygame.display.set_mode(self.size, 0)


        # this is the same as what we saw before
        #self.clist = pygame.camera.list_cameras()
        #if not self.clist:
            #raise ValueError("Sorry, no cameras detected.")
        #self.cam = pygame.camera.Camera(self.clist[0], self.size)
        #self.cam.start()
        
        self.cam = pygame.camera.Camera("/dev/video0",self.size)
        self.cam.start()
        self.snapshot = pygame.surface.Surface(self.size, 0, self.display)
        
        #self.btn = pygame.Surface((100,100),pygame.SRCALPHA, self.display)
        #self.btn.set_alpha(128)
        #self.btnRect = pygame.Rect(100,100,50,50)
        
        
        

    def get_and_flip(self,sendTrue):
        if self.cam.query_image():
            self.snapshot = self.cam.get_image(self.snapshot)
	   # self.snapshot = pygame.transform.scale(self.snapshot,(640,480))
 
        
        

        self.display.blit(self.snapshot, (0,0))
        pygame.display.flip()
        
        
        
        if sendTrue == True:
            #rawImage = self.cam.get_raw() #attempted to avoid saving to file first
            self.snapshot = self.cam.get_image(self.snapshot)
            pygame.image.save(self.snapshot,"sentImage.jpg")
            f = open("sentImage.jpg","rb")
            fileContent = f.read()
            byteArr = bytearray(fileContent)
            
            publish.single(PI4_PATH,byteArr,hostname = PI4_SERVER)
            
            
        return False
        
        
def my_callback(channel): 
    sendPhoto = True
 
def main():
    
    
    capture = Capture()
    going = True
    sendPhoto = False
    GPIO.add_event_detect(26, GPIO.FALLING, callback=my_callback,bouncetime = 100) 
    
    
    
    while going:
        #pygame.draw.rect(s2,[255,0,0,],btn)
        #apture.buttonBlit()
        events = pygame.event.get()
        for e in events:
            if e.type == QUIT or (e.type == KEYDOWN and e.key == K_ESCAPE):
                # close the camera safely
                capture.cam.stop()
                going = False
                GPIO.cleanup()
            

        sendPhoto = capture.get_and_flip(sendPhoto)
    
    
     
if __name__ == '__main__':
    main()
     
