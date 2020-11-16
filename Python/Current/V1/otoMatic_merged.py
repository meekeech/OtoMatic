import time 
import serial #import serial and time libraries
import struct
import numpy as np 
import datetime as dt 
import matplotlib.pyplot as plt 
import matplotlib.animation as animation 
import cv2
from matplotlib.animation import FuncAnimation
import threading
import sys
import imutils
import datetime
import queue
import csv

fileName = datetime.datetime.now().strftime('csvOutput/oto_matic-%Y-%m-%d-%H-%M.') 
saveQueue = queue.Queue()
#plotQueue = queue.Queue()
videoQueue = queue.Queue()

#Serial
ser = serial.Serial() #create serial instance to allow serial communication
ser.close() #close any previously open serial ports
ser.baudrate = 9600  #set the baud rate to match that of the microcontroller
ser.port = '/dev/nano1' #device name in Linux
ser.timeout = 1 #set atimeout of 1 second if no data received	
msg = "r"

count = 0
printTrue = True
dataRow = np.array([])
dataPlot = np.array([])

xs = []
bLeft = []
bRight = []
solenoid = []
pump = []
carbon = []
oxy = []
humid = []
pressure = []
flow = []
upperX = 50;
upperY = 1023;
sensorLabels = ['CO2 (%)','O2 (%)','H (%)','P (daPa)','F (mL/min)']
sensorTitles = ['CO2 Content','O2 Content','Humidity','Pressure','Flow Rate']
binaryLabels = ['BL', 'BR','S','P']
binaryTitles = ['Button Left', 'Button Right','Solenoid','Pump']
#fig, ax = plt.subplots(5,2)

fig, ax = plt.subplots(5,2)
xdata, ydata = [], []
#ln0, = ax[0].plot([], [], 'r')
#ln1, = ax[1].plot([], [], 'b')

newData = False
running = 1
cameraRunning = 0
frameNum = 0
videoSave = False





ln0, = ax[0,0].plot([], [], 'b')
ln1, = ax[1,0].plot([], [], 'r')
ln2, = ax[2,0].plot([], [], 'g')
ln3, = ax[3,0].plot([], [], 'c')
ln4, = ax[4,0].plot([], [], 'm')
ln5, = ax[0,1].plot([], [], 'r')
ln6, = ax[1,1].plot([], [], 'g')
ln7, = ax[2,1].plot([], [], 'b')
ln8, = ax[3,1].plot([], [], 'm')
ln9, = ax[4,1].plot([], [], 'b')
ln10, = ax[4,1].plot([], [], 'g')
ln11, = ax[4,1].plot([], [], 'r')

histTrue = False
histSize = np.arange(256)


def handle_close(evt):
    global running
    print('Closed')
    running = 0
    #sys.exit()

        
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


def SetPlot():
    
    global upperX
    global upperY
    
    for y in range (2):
        for x in range(5):
            if x == 4 & y == 1:
                pass
            else:
                ax[x,y].clear()
    for x in range(5):
        ax[x,0].set_xlim(0,upperX)
        ax[x,0].set_ylim(0,upperY)
        ax[x,0].set_ylabel(sensorLabels[x])
        #ax[x,0].set_title(sensorTitles[x])
        if x != 4: #skip since there's only 4 binary values
            ax[x,1].set_xlim(0,upperX)
            ax[x,1].set_ylim(0,1.1)
            ax[x,1].set_ylabel(binaryLabels[x])
        ax[4,1].set_xlim(0,256)    
        ax[4,1].set_ylim(0,50000)  
            
    return ln0,ln1,ln2,ln3,ln4,ln5,ln6,ln7,ln8,ln9,ln10,ln11,
        
        
def update(count):
    
    #print('count',end='')
    #print(count)
    #xdata.append(count)
    #ydata.append(np.sin(count))
    #ln.set_data(xdata, ydata)
    #ln2.set_data(xdata,ydata)
   
    global upperX,printTrue, msg
    global xs
    global bLeft
    global bRight 
    global newData
    global solenoid 
    global pump
    global carbon
    global oxy
    global humid
    global pressure
    global flow
    global dataPlot
    global histTrue
    global histSize

    

    if newData == True:
        
        
        
        xs.append(count)
        bLeft.append(dataPlot[0])
        bRight.append(dataPlot[1])
        solenoid.append(dataPlot[2])
        pump.append(dataPlot[3])
        carbon.append(dataPlot[4])
        oxy.append(dataPlot[5])
        humid.append(dataPlot[6])
        pressure.append(dataPlot[7])
        flow.append(dataPlot[8])
             
        ln0.set_data(xs, carbon)
        ln1.set_data(xs, oxy)
        ln2.set_data(xs, humid)
        ln3.set_data(xs, pressure)
        ln4.set_data(xs, flow)
        ln5.set_data(xs, bLeft)
        ln6.set_data(xs, bRight)
        ln7.set_data(xs, solenoid)
        ln8.set_data(xs, pump)
        
        if histTrue == True:
            frm = videoQueue.get()
            histrB = cv2.calcHist([frm],[0],None,[256],[0,256]) 
            histrG = cv2.calcHist([frm],[1],None,[256],[0,256])
            histrR = cv2.calcHist([frm],[2],None,[256],[0,256])        

            ln9.set_data(histSize,histrB)
            ln10.set_data(histSize,histrG)
            ln11.set_data(histSize,histrR)

            histTrue = False
        
        newData = False
    
        if count == (upperX):
            bLeft = []
            bRight = []
            solenoid = []
            pump = []
            carbon = []
            oxy = []
            humid = []
            pressure = []
            flow = []
            xs = []

    return ln0,ln1,ln2,ln3,ln4,ln5,ln6,ln7,ln8,ln9,ln10,ln11,        
        
        
class camReader(threading.Thread):
    
    def run(self):
        global cameraRunning
        global frameNum
        videoSave = True
        
        print('Starting Cam Reader')
        capture = cv2.VideoCapture(0)
        time.sleep(1)

        #preCapture for sizing...
        ret, frame = capture.read()
        frame = imutils.resize(frame, width=400)
        
        if videoSave is True:
            fshape = frame.shape
            fheight = fshape[0]
            fwidth = fshape[1]
            vid_cod = cv2.VideoWriter_fourcc(*'avc1')
            vidOutput = cv2.VideoWriter(fileName +'mp4', vid_cod, 20.0, (fwidth, fheight)) 

        while(cameraRunning):
            ret, frame = capture.read()
            if videoSave is True:
                vidOutput.write(frame)
            videoQueue.put(frame)
            frameNum += 1
            time.sleep(0.01)
            
        capture.release()
        if videoSave is True:
            vidOutput.release()
        cv2.destroyAllWindows()
        print('Ending camWindow')


class vidDisplay(threading.Thread):
    
    def run(self):
        global cameraRunning
        global histTrue
        videoShow = True
        
        print('Starting Vid Display')

        while (cameraRunning):
            if videoShow == True:
                if videoQueue.qsize() > 0:
                    #print('showing frame')
                    frame = videoQueue.get()
                    cv2.imshow('Video', frame)
                time.sleep(0.01)
                if cv2.waitKey(1) == 27:
                    cameraRunning = 0
                    histTrue = True
                    break
        
        cv2.destroyAllWindows()
        
class serialListener(threading.Thread):
  
    def run(self):
        global newData
        global dataSend
        global cameraRunning
        global dataPlot
        print('Starting Serial Listener')
        ser.open() #begins serial communication
        msg = "r"
        printTrue = False
        
        while(running):
            time.sleep(0.01)
            #print(ser.in_waiting)
                
            if ser.in_waiting == 0 and ser.isOpen():
                ser.write(msg.encode())

            elif ser.in_waiting > 22 and ser.isOpen():
                dataRow = np.array([])
                dataRow = ReadData(dataRow,1,0,printTrue) #B
                dataRow = ReadData(dataRow,1,1,printTrue) #Left Button State
                dataRow = ReadData(dataRow,1,1,printTrue) #Right Button State
                if dataRow[1] == '1' and cameraRunning == 0:
                    #plotTrue = False
                    #time.sleep(1)
                    print('video')
                    cameraRunning = 1
                    camReader().start()
                    vidDisplay().start()
                    
                dataRow = ReadData(dataRow,1,0,printTrue) #S
                dataRow = ReadData(dataRow,1,1,printTrue) # Solenoid State
                dataRow = ReadData(dataRow,1,1,printTrue) # Pump State
                dataRow = ReadData(dataRow,1,0,printTrue) # C
                dataRow = ReadData(dataRow,2,1,printTrue) # CO2 reading
                dataRow = ReadData(dataRow,1,0,printTrue) # 0
                dataRow = ReadData(dataRow,2,1,printTrue) # O2 reading
                dataRow = ReadData(dataRow,1,0,printTrue) # H
                dataRow = ReadData(dataRow,2,1,printTrue) # Humidity reading
                dataRow = ReadData(dataRow,1,0,printTrue) # P
                dataRow = ReadData(dataRow,2,1,printTrue) # Pressure reading
                dataRow = ReadData(dataRow,1,0,printTrue) # F
                dataRow = ReadData(dataRow,2,1,printTrue) # Flow reading
                if printTrue == True:
                    print('')
                # Clear the buffer
                readByte = ser.readline()
                
 #               print(datetime.datetime.now().time())
  #              print(dataRow)
                saveQueue.put(dataRow)
                dataPlot = dataRow.astype(int)
                newData=True


   
class csvWriter(threading.Thread):
    def run(self):
        print('Starting CSV Writer')  
        #global running

        with open(fileName + 'csv', 'w+') as csvfile:
            csvwriter = csv.writer(csvfile, dialect='excel')
            csvwriter.writerow(['Frame #','Left Button', 'Right Button', 'Soleonid','Pump','CO2','O2','Humidity','Pressure','Flow'])
            startTime = time.perf_counter_ns()
            while (running):
                if saveQueue.qsize() > 0:
                    data = saveQueue.get()
#                    print(data)
                    csvwriter.writerow([frameNum,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8]])
                else:
                    time.sleep(0.01)
            print('Closing File')
            csvfile.close()    


    


def main():
    global upperX
    fig.canvas.mpl_connect('close_event',handle_close)
    
    frames = np.arange(upperX+1)
    frames = frames.astype(int)
    #frames = np.linspace(0, 2*np.pi, 128)
  
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    
    
    csvWriter().start()
    serialListener().start()
    #running = 1
    #print('hi')
    ani = FuncAnimation(fig, update,frames,init_func=SetPlot, blit=True, interval=400)
    plt.show()
    
    while(running):
        pass
    print('Closing Main')
    time.sleep(1)
   


if __name__ == '__main__':
    main()
    
    

