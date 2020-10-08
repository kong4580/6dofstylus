
import fltk
import sys
import serial
import numpy as np
from math import pi
from functools import partial

from StylusReciever import StylusReciever
from Stylus import Stylus,Stylus2

from GUI import Gui
import drawFunc
from Model import Model,OBJ,ArticulateModel,Joint
from Controller import MainController,StylusController,MouseController,StylusController2

def callback(samplingRate,gui):

    while gui.openglWindow.shown():
        # if recieve serial messages
        
        
        if mode != 'mouse' and srec.isActivate():
            command,rawData = srec.recieve()
            # print(rawData)
            # update joint state command
            if command == 0xFF and mode == 'stylus':
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
            if command == 0xFE and mode == 'stylus2':
                # get button states
                jointStates,imuData,calibrateState,buttonStates = srec.readCommand(command,rawData)
                pose = stylus.getEndTransforms(jointStates)
                if calibrateState and not device.isImuInit:
                    print("IMU calibrate finish")
                    print("Set home position then press k ")
                device.setTransform(pose,imuData)
                gui.openglWindow.ctl.readEvent(999)
                
                if buttonStates[0] == True and buttonStates[1] == False:
                    gui.openglWindow.ctl.readEvent(1001)
                elif buttonStates[0] == False and buttonStates[1] == True:
                    gui.openglWindow.ctl.readEvent(1002)
                else:
                    gui.openglWindow.ctl.readEvent(1000)
        try:
            gui.updateUI()
        except:
            pass
        
        fltk.Fl_check()    
    uiCallback = partial(callback,samplingRate,gui)
    fltk.Fl_repeat_timeout(samplingRate,uiCallback)
    

def openGUI(samplingRate = 0.005):
    
    
    gui.openglWindow.ctl = mainController
    # add model
    jointA = Joint('A',1,drawFunc.drawPyramid)
    jointB = Joint('B',2,drawFunc.drawPyramid)
    jointB.moveModel(np.array([[0,1,0,-2],
                        [-1,0,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]))
    jointC = Joint('C',3,drawFunc.drawPyramid)
    jointC.moveModel(np.array([[1,0,0,-2],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]))
    # teapot = OBJ('./teapot.obj',scale=1)
    # gui.addModel('teapot',1,teapot.initOBJ,obj=teapot)
    # bunny = OBJ('./bunny.obj',scale=20)
    # gui.addModel('bunny',2,bunny.initOBJ,obj=bunny)
    gui.addModel('Arti1',6,objType = 'joint',listOfJoint=[jointA,jointB,jointC])
    
    
    # open GUI window
    gui.window.show()
    
    # run callback function
    uiCallback = partial(callback,samplingRate,gui)
    fltk.Fl_add_timeout(samplingRate,uiCallback)
    
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
        
    # get input mode from terminal
    mode = sys.argv[1]
    print("\n Start Program with:",mode,"\n")
    # declare ui samplingRate
    samplingRate = 0.005
    
    # init GUI
    gui = Gui()
    
    # init controller
    mainController = MainController()
    
    
    if mode == 'mouse':
        packData = {'flags':gui.openglWindow.flags,
                'modelDicts':gui.openglWindow.modelDicts,
                'log':gui.log,
                'height':gui.openglWindow.h(),
                'width':gui.openglWindow.w(),
                'camera':gui.openglWindow.cameravalue
                }
        deviceController = MouseController(packData)

    elif mode == 'stylus':
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
                'camera':gui.openglWindow.cameravalue}
        deviceController = StylusController(packData)
        
    elif mode == 'stylus2':
         # declare port
        # port = '/dev/pts/2' # ubuntu port
        port = '/dev/ttyUSB0' # arduino port
        
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
                'camera':gui.openglWindow.cameravalue}
        deviceController = StylusController2(packData)
    mainController.registerController(deviceController)
    device = mainController.getController()
    # run GUI
    openGUI(samplingRate)