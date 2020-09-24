
import fltk
import sys
import serial
import numpy as np
from math import pi
from functools import partial

from StylusReciever import StylusReciever
from Stylus import Stylus

from GUI import Gui
import drawFunc
from Model import Model,OBJ
from Controller import MainController,StylusController

def callback(samplingRate,gui):

    while gui.openglWindow.shown():
        # if recieve serial messages
        if srec.isActivate():
            command,rawData = srec.recieve()
            
            # update joint state command
            if command == 0xFF:
                # get joint states and button states
                jointStates,buttonStates = srec.readCommand(command,rawData)
                # print(jointStates)
                pose = stylus.getEndTransforms(jointStates)
                # update gui
                gui.updateUI(pose[1],buttonStates,scale=20,cursorTransform=pose[0])
                
                
                
            # update button state command    
            if command == 0xFE:
                # get button states
                pass
        # gui.updateUI(None,None)
        gui.updateFltk() 
        fltk.Fl_check()    
    uiCallback = partial(callback,samplingRate,gui)
    fltk.Fl_repeat_timeout(samplingRate,uiCallback)
    

def openGUI(samplingRate = 0.005):
    mainController = MainController()
    
    packData = {'flags':gui.openglWindow.flags,
                'modelDicts':gui.openglWindow.modelDicts,
                'log':gui.log}
    
    stylusController = StylusController(packData)
    mainController.registerController(stylusController)
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
    
    # declare port
    port = '/dev/pts/4' # ubuntu port
    # port = '/dev/ttyUSB0' # arduino port
    
    # declare constants
    samplingRate = 0.005
    serialTimeOut = 0.005
    
    # init stylus
    print("Init Stylus ... ",end="")
    stylus = Stylus() 
    print("Done !")
    
    # init serial
    print("Init Stylus Serial ... ",end="")
    srec = StylusReciever(baudrate = 9600,port = port,timeout = serialTimeOut) 
    print("Done !")
    
    # init GUI
    gui = Gui()
    
    # run GUI
    openGUI(samplingRate)