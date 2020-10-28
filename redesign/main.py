
import fltk
import sys
import serial
import numpy as np
from math import pi,cos,sin
from functools import partial
import multiprocessing as mp
from StylusReciever import StylusReciever
from Stylus import Stylus,Stylus2

from GUI import Gui
import drawFunc
from Model import Model,OBJ,ArticulateModel,Joint
from Controller import MainController,StylusController,MouseController,StylusController2

import time
def readSerial(conn):
    pose = None
    imuData = None
    posed=np.array([0,0,0])
    while True:
        try:
                srec.send("a".encode())
            
            # if gui.openglWindow.shown():
            
                if controllerMode != 'mouse' and srec.isActivate():
                    s = time.time()
                    
                    command,rawData = srec.recieve()
                    # print(rawData)
                    # update joint state command
                    if command == 0xFF and controllerMode == 'stylus':
                        # print(mode)
                        # get joint states and button states
                        jointStates,buttonStates = srec.readCommand(command,rawData)
                        # print(jointStates)
                        pose = stylus.getEndTransforms(jointStates)
                        
                        device.setTransform(pose)
                        gui.openglWindow.ctl.readEvent(999)
                        
                        if buttonStates[0] == True and buttonStates[1] == False:
                            gui.openglWindow.ctl.readEvent(1001)
                        elif buttonStates[0] == False and buttonStates[1] == True:
                            gui.openglWindow.ctl.readEvent(1002)
                        else:
                            gui.openglWindow.ctl.readEvent(1000)
                        
                        
                        
                        
                    # update button state command    
                    if command == 0xFE and controllerMode == 'stylus2':
                        # get button states
                        jointStates,imuData,calibrateState,buttonStates = srec.readCommand(command,rawData)
                        pose = stylus.getEndTransforms(jointStates)
                        posed = np.vstack((posed,np.array(pose[0:3,3])))
                        
                        # if calibrateState and not device.isImuInit:
                        #     print("IMU calibrate finish")
                        #     print("Set home position then press k ")

                        # device.setTransform(pose,imuData)
                        # gui.openglWindow.ctl.readEvent(999)
                        
                        # if buttonStates[0] == True and buttonStates[1] == False:
                        #     gui.openglWindow.ctl.readEvent(1001)
                        # elif buttonStates[0] == False and buttonStates[1] == True:
                        #     gui.openglWindow.ctl.readEvent(1002)
                        # else:
                        #     gui.openglWindow.ctl.readEvent(1000)
                    # print(time.time() - s)
            
        except:
            pass
        # print(conn.recv())
        if conn.poll(0.0000000000000000005):
            recvMsg = conn.recv()
            if recvMsg == 1 and pose is not None:
                # print("call")
            
                scale = np.array([0.3,0.2,0.3])
                scalePosed = posed*scale
                scalePosed = np.sum(scalePosed,axis=0)/scalePosed.shape[0]
                # print(scalePosed)
                try:
                    pose[0,3] = scalePosed[0].copy()/scale[0]
                    pose[1,3] = scalePosed[1].copy()/scale[1]
                    pose[2,3] = scalePosed[2].copy()/scale[2]
                    posed = np.array([scalePosed[0].copy()/scale[0],scalePosed[1].copy()/scale[1],scalePosed[2].copy()/scale[2]])
                except:
                    print(posed)
                    print(posed.shape)
                    print(scalePosed)
                
                conn.send([1,pose,imuData,buttonStates])
        
    # time.sleep(1)
def callback(samplingRate,gui,conn):
    # print(conn)
    while gui.openglWindow.shown():
        recvMsg = []
        # if recieve serial messages
        
        conn.send(1)
        
        if conn.poll(0.00000005):
            recvMsg = conn.recv()
            # conn.send(1)
        s = time.time()
        
        if len(recvMsg)>0 and recvMsg[0] == 1:
            pose,imuData,buttonStates = recvMsg[1],recvMsg[2],recvMsg[3]
            # print(pose)
            device.setTransform(pose,imuData)
            gui.openglWindow.ctl.readEvent(999)
            
            if buttonStates[0] == True and buttonStates[1] == False:
                gui.openglWindow.ctl.readEvent(1001)
            elif buttonStates[0] == False and buttonStates[1] == True:
                gui.openglWindow.ctl.readEvent(1002)
            else:
                gui.openglWindow.ctl.readEvent(1000)
        # print(time.time()-s)   
        
        
        try:
            
            gui.updateUI()
            
        except:
            pass
        
        fltk.Fl_check() 
    
    # uiCallback = partial(callback,samplingRate,gui,conn)
    # fltk.Fl_repeat_timeout(samplingRate,uiCallback)
    

def openGUI(conn,samplingRate = 0.005):
    
    
    gui.openglWindow.ctl = mainController
    # add model
    if modelType == "articulate":
        jointA = Joint('A',1,drawFunc.drawPyramid)
        jointB = Joint('B',2,drawFunc.drawPyramid)
        
        jointC = Joint('C',3,drawFunc.drawPyramid)
        if modelMode =='fk':
            gui.addModel('Arti1',6,objType = 'joint',listOfJoint=[jointA,jointB,jointC],showTarget = False,showPole = False)
        elif modelMode == 'ik':
            gui.addModel('Arti1',6,objType = 'joint',listOfJoint=[jointA,jointB,jointC],showTarget = True,showPole = True)
        jointB.moveModel(np.array([[1,0,0,0],
                            [0,1,0,3],
                            [0,0,1,0],
                            [0,0,0,1]]))
        # jointB.moveModel(np.array([[1,0,0,0],
        #                     [0,1,0,0],
        #                     [0,0,1,0],
        #                     [0,0,0,1]]),mode='relative')
        jointC.moveModel(np.array([[1,0,0,0],
                            [0,1,0,3],
                            [0,0,1,0],
                            [0,0,0,1]]),mode='relative')
        # jointA.moveModel(np.array([[1,0,0,0],
        #                     [0,0.7071,-0.7071,0],
        #                     [0,0.7071,0.7071,0],
        #                     [0,0,0,1]]),mode='relative')
    elif modelType == 'rig':
        teapot = OBJ('./teapot.obj',scale=1)
        gui.addModel('teapot',1,teapot.initOBJ,obj=teapot)
        bunny = OBJ('./bunny.obj',scale=20)
        gui.addModel('bunny',2,bunny.initOBJ,obj=bunny)
    
    else:
        print("wrong model type")
        sys.exit()
    
    
    # open GUI window
    gui.window.show()
    
    # run callback function
    uiCallback = partial(callback,samplingRate,gui,conn)
    fltk.Fl_add_timeout(samplingRate,uiCallback)
    # fltk.Fl_check()
    # print description
    print("Start Program ...")
    print("""
            Key\t\tDescription
             Q\t\tShow and hide model
             1\t\tShow model wireframe
             2\t\tTurn model to opacity 50%
             M\t\tReset model position
             L\t\tAdd User profie
             P\t\tStart test mode (start timer)
             D\t\tCalculate IoU\n\t\t\tWhile press D in test mode backdrop and model will switch to next one\n\t\t\tAfter finish test mode data will update in testLog.csv""")
    
    # open application
    return fltk.Fl_run()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Please enter device name !!")
        print("### device name = mouse, stylus, stylus2 ###")
        sys.exit()
    elif len(sys.argv) < 3:
        print("Please enter model type !!")
        print("### device name = rig,articulate ###")
        sys.exit()
    elif len(sys.argv) < 4:
        print("Please enter model move mode type !!")
        print("### device name = fk,ik ###")
        sys.exit()
    # get input mode from terminal
    controllerMode = sys.argv[1]
    modelType = sys.argv[2]
    if modelType == "rig":
        modelMode = 'fk'
    else:
        modelMode = sys.argv[3]
    print("\n Start Program with:",controllerMode,"\n")
    # declare ui samplingRate
    samplingRate = 0.0000000005
    
    # init GUI
    gui = Gui()
    
    # init controller
    mainController = MainController()
    
    # get controller mode
    gui.controllerMode = controllerMode
    
    if controllerMode == 'mouse':
        packData = {'flags':gui.openglWindow.flags,
                'modelDicts':gui.openglWindow.modelDicts,
                'log':gui.log,
                'height':gui.openglWindow.h(),
                'width':gui.openglWindow.w(),
                'camera':gui.openglWindow.cameravalue,
                'modelMode':modelMode
                }
        deviceController = MouseController(packData)

    elif controllerMode == 'stylus':
         # declare port
        # port = '/dev/pts/2' # ubuntu port
        port = '/dev/ttyUSB0' # arduino port
        
        # declare constants
        serialTimeOut = 0.0005
        
        # init stylus
        print("Init Stylus ... ",end="")
        stylus = Stylus() 
        print("Done !")
        
        # init serial
        print("Init Stylus Serial ... ",end="")
        srec = StylusReciever(baudrate = 115200,port = port,timeout = serialTimeOut) 
        print("Done !")
    
        packData = {'flags':gui.openglWindow.flags,
                'modelDicts':gui.openglWindow.modelDicts,
                'log':gui.log,
                'cursor':gui.openglWindow.cursor,
                'height':gui.openglWindow.h(),
                'width':gui.openglWindow.w(),
                'camera':gui.openglWindow.cameravalue,
                'modelMode':modelMode}
        deviceController = StylusController(packData)
        
    elif controllerMode == 'stylus2':
         # declare port
        # port = '/dev/pts/2' # ubuntu port
        port = '/dev/ttyACM0' # arduino port
        
        # declare constants
        
        serialTimeOut = 0.0005
        
        # init stylus
        print("Init Stylus ... ",end="")
        stylus = Stylus2() 

        print("Done !")
        
        # init serial
        print("Init Stylus Serial ... ",end="")
        srec = StylusReciever(baudrate = 115200,port = port,timeout = serialTimeOut) 
        print("Done !")
    
        packData = {'flags':gui.openglWindow.flags,
                'modelDicts':gui.openglWindow.modelDicts,
                'log':gui.log,
                'cursor':gui.openglWindow.cursor,
                'height':gui.openglWindow.h(),
                'width':gui.openglWindow.w(),
                'camera':gui.openglWindow.cameravalue,
                'modelMode':modelMode}
        deviceController = StylusController2(packData)
    mainController.registerController(deviceController)
    device = mainController.getController()
    conn1,conn2 = mp.Pipe()
    p = mp.Process(target = readSerial,args=(conn2,))
    
    # run GUI
    p.start()
    # p.join()
    
    openGUI(conn = conn1,samplingRate = samplingRate)
    
    print("Close program..")
    
    p.terminate()