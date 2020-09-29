
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
from Model import Model,OBJ
from Controller import MainController,StylusController,MouseController,StylusController2

def callback(samplingRate,gui):

    while gui.openglWindow.shown():
        # if recieve serial messages
        
        
        if mode != 'mouse' and srec.isActivate():
            command,rawData = srec.recieve()
            # print(rawData)
            # update joint state command
            if command == 0xFF:
                print(mode)
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
            if command == 0xFE:
                # get button states
                jointStates,imuData,calibrateState,buttonStates = srec.readCommand(command,rawData)
                pose = stylus.getEndTransforms(jointStates)
                if calibrateState:
                    print("imu calibrate finish")
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
    teapot = OBJ('./teapot.obj',scale=1)
    gui.addModel('teapot',teapot.initOBJ,obj=teapot)
    bunny = OBJ('./bunny.obj',scale=20)
    gui.addModel('bunny',bunny.initOBJ,obj=bunny)
    
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
    
    samplingRate = 0.005
    
    # init GUI
    gui = Gui()
    mainController = MainController()
    
    
    mode = 'stylus'
    if mode == 'mouse':
        packData = {'flags':gui.openglWindow.flags,
                'modelDicts':gui.openglWindow.modelDicts,
                'log':gui.log,
                'height':gui.openglWindow.h(),
                'width':gui.openglWindow.w(),
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
                'cursor':gui.openglWindow.cursor}
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
                'cursor':gui.openglWindow.cursor}
        deviceController = StylusController2(packData)
    mainController.registerController(deviceController)
    device = mainController.getController()
    # run GUI
    openGUI(samplingRate)