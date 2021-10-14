import threading
import paho.mqtt.client as mqtt
import time
import datetime
import queue
import csv
import serial
import numpy as np
import struct
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import cv2

fileName = datetime.datetime.now().strftime('csvOutput/oto_matic-%Y-%m-%d-%H-%M.') 
running = 1
running1 = 1
getOut = False
saveQueue = queue.Queue()
tempQueue = queue.Queue()
count1 = 1
count2 = 1
imageReady = 0
imagePath = 'photos/startup.jpg'

PI4_SERVER = "localhost"
PI4_PATH = [("piz1-pi4",0),("piz2-pi4-img",0),("piz2-pi4-data",0)]

ser = serial.Serial (
        port = '/dev/nano1',
        baudrate = 115200,
        parity = serial.PARITY_NONE,
        stopbits = serial.STOPBITS_ONE,
        bytesize = serial.EIGHTBITS,
        timeout = 1
)
 
 
# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
 
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(PI4_PATH)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global prevTime
    global count1 
    global count2 
    global running1
    global imagePath
    global imageReady
    
    if (msg.topic == "piz1-pi4"):
        fileName1 = "photos/img1." + str(count1)
        count1 = count1 + 1
        f = open(fileName1,"wb")
        f.write(msg.payload)
        print("Image1 received")
        f.close()
    elif (msg.topic == "piz2-pi4-img"):
        fileName2 = "photos/img2." + str(count2)
        count2 = count2 + 1
        f2 = open(fileName2,"wb")
        f2.write(msg.payload)
        f2.close()
        print("Image2 received")
        print("")
        imagePath = fileName2
        imageReady = 1


    elif (msg.topic == "piz2-pi4-data"):
        vals = msg.payload
        #print(vals,end = ',')
        val1 = chr(vals[0]) # D or M or T or X or C
      #  print(vals.decode())
        if (val1 == 'X'):
            ser.write('s'.encode())
        elif (val1 == 'D'):
            val2 = chr(vals[1]) #0 or 1 for motor dir
            if (val2 == '0'):
                ser.write('f'.encode())
            elif (val2 == '1'):
                ser.write('r'.encode())
        elif (val1 == 'M'):
            saveQueue.put(vals.decode())
        elif (val1 == 'T'):
            tempQueue.put(vals.decode())
        elif (val1 == 'C'):
            #csv1.CloseFile()
            #firstTime = True
            #running1 = 0
            print('done transfer')
        else:
            print(vals)
        
        
        
    


    
def InitializeMQTT():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
     
    client.connect_async(PI4_SERVER, 1883, 60)
    client.loop_start()
    
class csvWriter(threading.Thread):
    
    #def __init__(self): 
        #threading.Thread.__init__(self)
        #print('Starting CSV Writer')  
        #with open(fileName + 'csv', 'a') as self.csvfile:
            #self.csvwriter = csv.writer(self.csvfile, dialect='excel',delimiter =',')
        
    def run(self):
        print('Starting CSV Writer')  
        global running1
        global getOut
        firstTymp = True
        
        with open(fileName + 'csv', 'a') as csvfile:
            csvwriter = csv.writer(csvfile, dialect='excel',delimiter =',')

            while (running1):
                if (getOut is True):
                    print("leaving")
                    return
                
                if saveQueue.qsize() > 0:
                    if firstTymp is True:
                        csvwriter.writerow(['Type','Mic','Pressure'])
                        firstTymp = False
                    
                    data = saveQueue.get()
                    data = data.split(",")
                    csvwriter.writerow(data)
                else:
                    time.sleep(0.01)
                if tempQueue.qsize() > 0:
                    data2 = tempQueue.get()                
                    csvwriter.writerow(['Type','Temp'])                                       
                    data2 = data2.split(",")
                    csvwriter.writerow(data2)
                    firstTymp = True
   
                                
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
        data = numpy.append(data,[readVal])
        
    return data
    
    
class serialListener(threading.Thread):
  
    def run(self):
        global newData
        global dataSend
        print('Starting Serial Listener')
 
        printTrue = True
        
        while(running):
            time.sleep(0.01)
            #print(ser.in_waiting)

            if ser.in_waiting > 2 and ser.isOpen():
                dataRow = numpy.array([])
                dataRow = ReadData(dataRow,2,1,printTrue) #f
               
                   
                if printTrue == True:
                    print('')
                    
                # Clear the buffer
                #readByte = ser.readline()

                newData=True
                
   
def handle_close(evt):
    global running
    print('Closed')
    running = 0
    #sys.exit()
   
def hexencode(rgb):
    r=rgb[0]
    g=rgb[1]
    b=rgb[2]
    return '#%02x%02x%02x' % (r,g,b)
    
        
def main():
    global getOut
    global imageReady
    global imagePath

    InitializeMQTT()
    time.sleep(0.5)
    csv1 = csvWriter()
    csv1.start()
    serialListener().start()

#setup display stuff
    plt.ion()
    x = np.linspace(0,255, 256)
    y = 5000*np.sin(x)
    fig = plt.figure()
    # start histogram plot
    ax1 = fig.add_subplot(121)
    line1, = ax1.plot(x,y,'b-')
    line2, = ax1.plot(x,y,'g-')
    line3, = ax1.plot(x,y,'r-')
    fig.canvas.mpl_connect('close_event',handle_close)
    #start image plot
    ax2 = fig.add_subplot(122)
    cv_img = cv2.imread(imagePath)
    cv_img = cv2.cvtColor(cv_img,cv2.COLOR_BGR2RGB)
    img_obj = plt.imshow(cv_img)



    while(running):
        # see if there is a new image to play
        if (imageReady > 0) :
            cv_img = cv2.imread(imagePath)
            cv_img = cv2.cvtColor(cv_img,cv2.COLOR_BGR2RGB)
            img_obj.set_data(cv_img)
            (B, G, R) = cv2.split(cv_img)
            
            histr = cv2.calcHist([B],[0],None,[256],[0,256])
            print(x.shape)
            print(histr.shape)
            line1.set_ydata(histr)
            histr = cv2.calcHist([G],[0],None,[256],[0,256])
            line2.set_ydata(histr)
            histr = cv2.calcHist([R],[0],None,[256],[0,256])
            line3.set_ydata(histr)
            ax1.relim()
            ax1.autoscale_view()
            
            fig.canvas.draw()

            imageReady = 0
            fig.canvas.flush_events()
   
    print('Closing Main')
    client.loop_stop()


if __name__ == '__main__':
    main()
