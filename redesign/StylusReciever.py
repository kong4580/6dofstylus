import serial
import sys
import numpy as np
from math import pi


import time
from Stylus import Stylus
# ser = serial.Serial('/dev/pts/1')
# while(1):
#     data = ser.read()
#     print(int.from_bytes(data,byteorder='big'))
    
# ser.close()

class StylusReciever():

    def __init__(self,port,baudrate=9600,timeout=None):
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(baudrate=self.baudrate,port=self.port,timeout = timeout,write_timeout = 1)
    
    def isActivate(self):
        return self.ser.is_open
    
    def getDevice(self):
        return self.ser.name
    
    def connect(self):
        self.ser.open()

    def disconnect(self):
        self.ser.close()

    def send(self,data):
        self.ser.write(data)
    
    def readRaw(self):
        rawData = self.ser.read()
        if not len(rawData):
            return None
        data = int.from_bytes(rawData, byteorder=sys.byteorder)
        return data
    
    def recieve(self):
        # self.ser.reset_input_buffer()
        # time.sleep(0.0000001)
        dataLen = 0
        
        start = [None] * 2
        stop = [None] * 2
        # check start bytes
        start[0] =  self.readRaw()
        if start[0] == None:
            # print("start0")
            return -1,-1
        elif  start[0]== 0xFE:
            # print("start0 here")
            start[1] =  self.readRaw()
            if start[1] == 0xFF:
                
        # get command
                command = self.readRaw()
        # get data length
                dataLen = self.readRaw()
        
        # get data
                if command == None or dataLen == None:
                    return -1,-1
                rawData = [None] * dataLen

                for i in range(dataLen+2):

                    if i < dataLen:
                        rawData[i] = self.readRaw()

                    else:
                        # get stop bytes
                        stop[i-dataLen] = self.readRaw()

                # check stop bytes
                if stop[0] == 0xAF and stop[1] == 0xFF: # all data is correct
                    
                    return command,rawData
                else: # data is wrong
                    return -1,-1
            else:
                # print("start1 wrong",start[1])
                return -1,-1
        else:
            return -1,-1
        
                
    def encDataCvt(self,data):
        
        def hexCheck(dummy):
            # convert alphabet to int
            if dummy == 'a':
                dummy = 10
            elif dummy == 'b':
                dummy = 11
            elif dummy == 'c':
                dummy = 12
            elif dummy == 'd':
                dummy = 13
            elif dummy == 'e':
                dummy = 14
            elif dummy == 'f':
                dummy = 15
            return dummy
        
        convertedData = [None]*(len(data)//2)
        # print(len(data))
        for i in range(0,len(data),2):
            
            # High byte
            if len(hex(data[i]))==3:
                dummy = hexCheck(hex(data[i])[2])
                highByte = int(dummy)*16**2
            elif len(hex(data[i]))==4:
                dummy1 = hexCheck(hex(data[i])[2])
                dummy2 = hexCheck(hex(data[i])[3])
                highByte = int(dummy1)*16**3+int(dummy2)*16**2
                
            # Low byte
            if len(hex(data[i+1]))==3:
                dummy = hexCheck(hex(data[i+1])[2])
                lowByte = int(dummy)
            elif len(hex(data[i+1]))==4:
                dummy1 = hexCheck(hex(data[i+1])[2])
                dummy2 = hexCheck(hex(data[i+1])[3])
                lowByte = int(dummy1)*16+int(dummy2)
                
            # convert data
            convertedData[i//2] = highByte + lowByte
            maxInt = 2**(8*2-1)-1
            convertedData[i//2] = convertedData[i//2] - (2*(maxInt+1) if convertedData[i//2] > maxInt else 0)
        return convertedData
    
    def getButtonState(self,data):
        binData = bin(data)
        
        if len(binData)==3:
            b1State = False
            b2State = bool(int(binData[2]))
            
        elif len(binData)== 4 :
            b1State = bool(int(binData[2]))
            b2State = bool(int(binData[3]))
        else:
            print("Invalid data")
        return b1State, b2State
    def readCommand(self, command, rawData):
        
        if command == 0xFF:# jointStates from stylus
            jointRawData = rawData[:-1]
            buttonStateRawData = rawData[-1]
            
            jointStates = self.encDataCvt(jointRawData)
            buttonStates = self.getButtonState(buttonStateRawData)
            
            for idx,j in enumerate(jointStates):
                jointStates[idx] = j*(2*pi)/4096
                
            return jointStates,buttonStates
        if command == 0xFE:
            jointRawData = rawData[:6]
            imuRawData = rawData[6:-2]
            calibrateRawData = rawData[-2]
            buttonStateRawData = rawData[-1]
            
            jointStates = self.encDataCvt(jointRawData)
            for idx,j in enumerate(jointStates):
                jointStates[idx] = j*(2*pi)/4096
            imuData = self.encDataCvt(imuRawData)
            buttonStates = self.getButtonState(buttonStateRawData)
            
            imuNpData = np.array(imuData)/1000
            state = [False, False, False]
            
            
            
            if calibrateRawData >= 4:
                state[0] = True
                calibrateRawData -= 4
                
            if calibrateRawData >= 2:
                state[1] = True
                calibrateRawData -= 2
                
            if calibrateRawData >= 1:
                state[2] = True
                calibrateRawData -= 1
            
            if state[0] and state[1] and state[2]:
                calibrateState = True
            else:
                
                print( "MagnetState:",state[0],"AccState:",state[1],"GyroState:",state[2])
                print("Please Calibrate")
                calibrateState = False
            return jointStates,imuNpData,calibrateState,buttonStates
            
        

if __name__ == '__main__':
    port = '/dev/pts/4' # ubuntu port
    # port = '/dev/ttyUSB0' # arduino port
    
    stylus = Stylus() # init stylus
    srec = StylusReciever(baudrate = 9600,port = port) # init serial
    
    # init GUI window
    ###########
    print("Start Program..\n")
    # while serial is activate
    while(srec.isActivate()):
        # read raw data
        command,rawData = srec.recieve()
        
        if command == 0xFF:
            # get joint states
            q,buttonStates = srec.readCommand(command,rawData)
            pose = stylus.getEndTransforms(q)
            
            print(buttonStates)
        if command == 0xFE:
            # get button states
            pass
        
        # update frame gui
        #   someFunction(pose)
        ############
        
        
