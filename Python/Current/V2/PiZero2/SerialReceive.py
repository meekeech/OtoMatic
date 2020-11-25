import time 
import serial #import serial and time libraries
import numpy as np 
import datetime as dt 
import threading
import sys
import struct

count = 0
printTrue = True
dataRow = np.array([])
#dataPlot = np.array([])

newData = False
running = 1

ser = serial.Serial (
        port = '/dev/ttyS0',
        baudrate = 9600,
        parity = serial.PARITY_NONE,
        stopbits = serial.STOPBITS_ONE,
        bytesize = serial.EIGHTBITS,
        timeout = 1
)

#while 1:
    #ser.write('Counter %d \n'.encode() %(counter))
    #ser.write("r".encode())
    #time.sleep(0.1)
    #counter +=1

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



class serialListener(threading.Thread):
  
    def run(self):
        global newData
        global dataSend
        print('Starting Serial Listener')
        #ser.open() #begins serial communication
        msg = "r"
        printTrue = False
        
        
        while(running):
            time.sleep(0.01)
            #print(ser.in_waiting)
                
           

            if ser.in_waiting > 10 and ser.isOpen():
                dataRow = np.array([])
                dataRow = ReadData(dataRow,1,0,printTrue) #B
                dataRow = ReadData(dataRow,1,1,printTrue) #Top Button State
                dataRow = ReadData(dataRow,1,1,printTrue) #Bottom Button State
                dataRow = ReadData(dataRow,1,0,printTrue) #S
                dataRow = ReadData(dataRow,1,1,printTrue) # Solenoid State
                dataRow = ReadData(dataRow,1,1,printTrue) # Motor Direction
                dataRow = ReadData(dataRow,1,0,printTrue) # P
                dataRow = ReadData(dataRow,2,1,printTrue) # Pressure reading
                
                
                #if (dataRow[0] == 1):
                    #read pressure and send to Rpi4
                    
                if printTrue == True:
                    print('')
                # Clear the buffer
                #readByte = ser.readline()
                
                newData=True
                
                

def handle_close(evt):
    global running
    print('Closed')
    running = 0

def main():
    serialListener().start()
    
    while(running):
        pass
        
    print('Closing Main')

if __name__ == '__main__':
    main()
    
