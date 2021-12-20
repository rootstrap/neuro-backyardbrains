# Backyard Brains Sep. 2019
# Made for python 3
# First install serial library
# Install numpy, pyserial, matplotlib
# pip3 install pyserial
#
# Code will read, parse and display data from BackyardBrains' serial devices
#
# Written by Stanislav Mircic
# stanislav@backyardbrains.com

import threading
import serial
import time
import matplotlib.pyplot as plt 
import numpy as np
from numpy import savetxt


global databuff
global connected
global input_buffer
global sample_buffer
global cBufTail
global stop_threads
global time_start
global last_time_recorded

########Initialize variables########

#change name of the port here
port = '/dev/cu.usbserial-DM02IZXW'
baud = 230400
connected = False
cBufTail = 0
input_buffer = []
sample_rate = 10000
display_size = 30000 #3 seconds
sample_buffer = np.linspace(0,0,display_size)
serial_port = serial.Serial(port, baud, timeout=0)
plot = False
stop_threads = False
databuff = np.array([0,0])
last_time_recorded = 0


def checkIfNextByteExist():
        global cBufTail
        global input_buffer
        tempTail = cBufTail + 1
        
        if tempTail==len(input_buffer): 
            return False
        return True
    

def checkIfHaveWholeFrame():
        global cBufTail
        global input_buffer
        tempTail = cBufTail + 1
        print('tempTail=', tempTail, 'len(input_buffer)=', len(input_buffer))
        while tempTail!=len(input_buffer): 
            nextByte  = input_buffer[tempTail] & 0xFF
            print('nextByte=', nextByte)
            if nextByte > 127:
                return True
            tempTail = tempTail +1
        return False;
    
def areWeAtTheEndOfFrame():
        global cBufTail
        global input_buffer
        tempTail = cBufTail + 1
        nextByte  = input_buffer[tempTail] & 0xFF
        if nextByte > 127:
            return True
        return False

def numberOfChannels():
    return 1

def handle_data(data):
    global input_buffer
    global cBufTail
    global sample_buffer   
    global databuff
    global time_start 
    global last_time_recorded
    print('handle_data')
    if len(data)>0:
        cBufTail = 0
        haveData = True
        weAlreadyProcessedBeginingOfTheFrame = False
        numberOfParsedChannels = 0
        
        while haveData:
            MSB  = input_buffer[cBufTail] & 0xFF
            
            if(MSB > 127): # Most Significant Bit
                weAlreadyProcessedBeginingOfTheFrame = False
                numberOfParsedChannels = 0
                
                if checkIfHaveWholeFrame():
                    
                    while True:
                        
                        MSB  = input_buffer[cBufTail] & 0xFF 
                        if(weAlreadyProcessedBeginingOfTheFrame and (MSB>127)):
                            #we have begining of the frame inside frame
                            #something is wrong
                            break #continue as if we have new frame
            
                        weAlreadyProcessedBeginingOfTheFrame = True
                        cBufTail = cBufTail +1
                        LSB  = input_buffer[cBufTail] & 0xFF 

                        if LSB>127:
                            break #continue as if we have new frame

                        LSB  = input_buffer[cBufTail] & 0x7F # Least Significant Bit 
                        MSB = MSB<<7
                        writeInteger = LSB | MSB
                        numberOfParsedChannels = numberOfParsedChannels+1
                        if numberOfParsedChannels>numberOfChannels():
            
                            #we have more data in frame than we need
                            #something is wrong with this frame
                            break #continue as if we have new frame
            
                        print('Writing sample buffer')    
                        this_sample = writeInteger-512
                        sample_buffer = np.append(sample_buffer, this_sample)
                        time_now = time.time()
                        time_elapsed = float("{:.6f}".format(time_now-time_start))
                        
                        if (time_elapsed > 0.002):
                            databuff = np.vstack((databuff, [time_now, this_sample]))
                            time_start = time_now


                        if areWeAtTheEndOfFrame():
                            break
                        else:
                            cBufTail = cBufTail +1
                else:
                    haveData = False
                    break
            if(not haveData):
                break
            cBufTail = cBufTail +1
            if cBufTail==len(input_buffer):
                haveData = False
                break


def read_from_port(ser):
    global connected
    global input_buffer
    while not connected:
        #serin = ser.read()
        connected = True
        print('Connected')
        while True:
            print('stop_threads', stop_threads)
            if stop_threads:
                break;
            print('reading..')    
            reading = ser.read(1024)
            print('len(reading)=',len(reading))
            if(len(reading)>0):
                reading = list(reading)
#here we overwrite if we left some parts of the frame from previous processing 
#should be changed             
                input_buffer = reading.copy()
                print("len(reading)",len(reading))
                handle_data(reading)
            time.sleep(0.001)

print('Start thread')
thread = threading.Thread(target=read_from_port, args=(serial_port,))
time_start = time.time()
thread.start()



if (plot):
    xi = np.linspace(-display_size/sample_rate, 0, num=display_size)
    while True:
        plt.ion()
        plt.show(block=False)
        if(len(sample_buffer)>0):
            #i = len(sample_buffer)
            print(len(sample_buffer))
            yi = sample_buffer.copy()
            yi = yi[-display_size:]
            sample_buffer = sample_buffer[-display_size:]
            plt.clf()      

            plt.ylim(-550, 550)
            plt.plot(xi, yi, linewidth=1, color='royalblue')
            plt.pause(0.001)
            time.sleep(0.08)
            
def press_enter():
    global stop_threads
    input("Press enter to finish")
    stop_threads = True
    print('Saving data')
    ts = time.time()
    savetxt(f'data/data_{ts}.csv', databuff, delimiter=',')
           
tread_input = threading.Thread(target=press_enter)
tread_input.start()
