from __future__ import print_function
from threading import Thread
import cv2
from imutils.video import FPS
import imutils
import time




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
        

def main():       
    
    
    
    vs = VideoStream(src=0).start()
    fps = FPS().start()
    running = 1

    while(running):
        #frame = vs.read()
        #frame = imutils.resize(frame,width = 400)
        
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
    
    print('Closing Main')

   


if __name__ == '__main__':
    main()
