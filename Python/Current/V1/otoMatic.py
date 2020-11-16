import time 
import serial #import serial and time libraries
import struct
import numpy as np 
import datetime as dt 
import matplotlib.pyplot as plt 
import matplotlib.animation as animation 
import cv2
import threading

import datetime
import queue
import csv


running = 1
frameNum = 0
fileName = datetime.datetime.now().strftime('oto_matic-%Y-%m-%d-%H-%M.')    
 
q = queue.Queue()
vq = queue.Queue()
    
#Serial
ser = serial.Serial() #create serial instance to allow serial communication
ser.close() #close any previously open serial ports
ser.baudrate = 9600  #set the baud rate to match that of the microcontroller
# ser.port = '/dev/nano1' #device name in Linux
ser.port = '/dev/uno1' # brians Arduino
ser.timeout = 1 #set atimeout of 1 second if no data received	

#Plotting 
#fig = plt.figure()
#ax = fig.add_subplot(1,1,1)
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
fig, ax = plt.subplots(5,2)

#Camera


#capture = cv2.VideoCapture(0)
#ret, frame = capture.read()

#cv2.imshow('video', frame)
#capture.release()


#Historgram
histSize = 256
histRange = (0, 256) # the upper boundary is exclusive
accumulate = False
refreshCount = 0;



def SetPlot():
    global upperX
    global upperY
    global sensorLabels
    global sensorTitles
    global binaryLabels
    global binaryTitles
    
    for x in range(5):
        ax[x,0].set_xlim(0,upperX)
        ax[x,0].set_ylim(0,upperY)
        ax[x,0].set_ylabel(sensorLabels[x])
        #ax[x,0].set_title(sensorTitles[x])
        if x != 4: #skip since there's only 4 binary values
            ax[x,1].set_xlim(0,upperX)
            ax[x,1].set_ylim(0,1.1)
            ax[x,1].set_ylabel(binaryLabels[x])
            #ax[x,1].set_title(binaryTitles[x])

def ResetPlot():
    for y in range (2):
        for x in range(5):
            ax[x,y].clear()
    SetPlot()
     

def AnimateMe(x1,data):
    global bLeft
    global bRight
    global solenoid
    global pump
    global carbon
    global oxy
    global humid
    global pressure
    global flow
    global xs
    global upperX

    xs.append(x1)
    bLeft.append(data[0])
    bRight.append(data[1])
    solenoid.append(data[2])
    pump.append(data[3])
    carbon.append(data[4])
    oxy.append(data[5])
    humid.append(data[6])
    pressure.append(data[7])
    flow.append(data[8])
 
    ax[0,0].plot(xs,carbon,'b')
    ax[1,0].plot(xs,oxy,'r')
    ax[2,0].plot(xs,humid,'g')
    ax[3,0].plot(xs,pressure,'c')
    ax[4,0].plot(xs,flow,'m')
    
    ax[0,1].plot(xs,bLeft,'r')
    ax[1,1].plot(xs,bRight,'g')
    ax[2,1].plot(xs,solenoid,'b')
    ax[3,1].plot(xs,pump,'m')

    plt.pause(0.001)
    
    if x1 == upperX:
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
        #ax.get_legend().remove()
        ResetPlot()
        
        

    
def ReadData(data,numBytes,appendTrue,printTrue):
    if numBytes == 1:
        readVal = ser.read(1).decode()
#        if printTrue == 1:
 #           print(readVal,end='')
    if numBytes == 2:
        readVal = ser.read(2)
        readVal = struct.unpack(">H",readVal)[0]
  #      if printTrue == 1:
   #         print(readVal,end='')
    if appendTrue == 1:
        data = np.append(data,[readVal])
        
    return data
    
def handle_close(evt):
    print('Closed')
    ser.close()
    #exit()

def StartVideo():
    global capture 
    global ret
    global frame
    
    print('Starting Video')
    
    while(1):
        print('Capturing frame')
        ret, frame = capture.read()
        cv2.imshow('video', frame)
        if cv2.waitKey(1) == 27:
            break
        
    capture.release()
    cv2.destroyAllWindows()
    cv2.imwrite('image1.png',frame) #make sure this updates numbers in the future
    CreateHistogram(frame)
    return True
    
def CreateHistogram(frm):
    color = ("b","g","r")
    for i,col in enumerate(color):
        histr = cv2.calcHist([frm],[i],None,[256],[0,256])
        #print(histr)
        ax[4,1].plot(histr,color=col)
        ax[4,1].set_xlim(0,256)    
        ax[4,1].set_ylim(0,50000)

class vidDisplay(threading.Thread):
    def run(self):
        global running
        print('Starting Vid Display')

        while (running):
            if vq.qsize() > 0:
                print('showing frame')
                frame = vq.get()
                cv2.imshow('Video', frame)
            time.sleep(0.01)
            if cv2.waitKey(1) == 27:
                running = 0
                break
        
        
class camReader(threading.Thread):
    def run(self):
        global running
        global frameNum
        print('Starting Cam Reader')
        capture = cv2.VideoCapture(0)
        

        #preCapture for sizing...
        ret, frame = capture.read()
        fshape = frame.shape
        fheight = fshape[0]
        fwidth = fshape[1]

        print('A')
        # Create output write
        #vid_cod = cv2.VideoWriter_fourcc(*'H264')
        #vid_cod = -1
        #vidOutput = cv2.VideoWriter(fileName +'mp4', vid_cod, 20.0, (fwidth, fheight)) 

        while(1):
            print('Capturing frame')
            startT = time.perf_counter_ns()
            ret, frame = capture.read()
            #readT =  time.perf_counter_ns()
            #cv2.imshow('video', frame)
            #showT =  time.perf_counter_ns()
            #vidOutput.write(frame)
            vq.put(frame)
            writeT =  time.perf_counter_ns()
            
            #print( 'Read :', readT - startT, '  Show:', showT-readT, '  Write:', writeT-showT)
            print ('tic toc:', (writeT- startT)/1000000)
            frameNum += 1

            time.sleep(0.01)         
        capture.release()
        vidOutput.release()
        cv2.destroyAllWindows()
        print('Ending camWindow')
         
class csvWriter(threading.Thread):
    def run(self):
        print('Starting CSV Writer')  
        with open(fileName + 'csv', 'w+') as csvfile:
            csvwriter = csv.writer(csvfile, dialect='excel')
            csvwriter.writerow(['Frame #','Time','Left Button', 'Right Button', 'Soleonid','Pump','CO2','O2','Humidity','Pressure','Flow'])
            startTime = time.perf_counter_ns()
            while (running):
                if q.qsize() > 0:
                    data = q.get()
#                    print(data)
                    csvwriter.writerow([frameNum,time.perf_counter_ns() - startTime, data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8]])
                else:
                    time.sleep(0.01)
                    #csvwriter.writerow(data)
            print('Closing File')
            csvfile.close()


class serialListener(threading.Thread):
    def run(self):
        print('Starting Serial Listener')
        ser.open() #begins serial communication
        msg = "r"
        printTrue = 0
        
        while(running):
            time.sleep(0.01)
            #print(ser.in_waiting)
                
            if ser.in_waiting == 0:
                ser.write(msg.encode())

            elif ser.in_waiting > 22 :
                dataRow = np.array([])
                dataRow = ReadData(dataRow,1,0,printTrue) #B
                dataRow = ReadData(dataRow,1,1,printTrue) #Left Button State
                dataRow = ReadData(dataRow,1,1,printTrue) #Right Button State
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
                
                # Clear the buffer
                readByte = ser.readline()
                
 #               print(datetime.datetime.now().time())
  #              print(dataRow)
                q.put(dataRow)
   #             print(q.qsize())
#            else :
#                print('Serial in waiting :', ser.in_waiting)
        print('Closing serialListener')

   
def main():
    print('Main')
    global running
    running = 1
 
    csvWriter().start()
    serialListener().start()
    vidDisplay().start()
    time.sleep(2)
    camReader().start()
 
    while(running):
# Collect events until released
               
        time.sleep(2)
    print('Closing Main')
    time.sleep(2)
'''
    global upperX
    global upperY
    
    plt.ion()
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    plt.show()
    SetPlot()

    
    fig.canvas.mpl_connect('close_event',handle_close)
    printTrue = 0
    plotTrue = True
 
    count = 0;
    xAxis = np.arange(0,upperX+1)
    
    RightButtonPrev = '0'
    

    #Numpy Array
    dataRow = np.array([])
    dataMatrix = np.array([[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0]]) #need two rows of zeros with current code
    
    place = True;

    for i in range (100): #infinite loop created

        #B11S00C111O222H333P444F555
        
 
            if printTrue == 1:
                print('')
            
            
            #dataMatrix = np.append(dataMatrix,[dataRow],axis=0)
            
            if plotTrue == True:
                AnimateMe(xAxis[count],dataRow.astype(int))
                #pass

            dataRow = np.array([])
            
        
            # Clear the buffer
            readByte = ser.readline()
            count = count +1
            if count == (upperX+1):
                count = 0
'''
            

if __name__ == '__main__':
    main()
    
    cv2.destroyAllWindows()
