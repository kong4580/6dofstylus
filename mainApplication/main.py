
import fltk
import sys
import serial
import numpy as np
from math import pi
from functools import partial

from StylusReciever import StylusReciever
from Stylus import Stylus
# from Guifunc import updateUI, OpenGLWindow, createOutputWidget
from GUI import Gui
import drawFunc
from Model import Model,OBJ

def callback(samplingRate,gui):

    
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
        
    uiCallback = partial(callback,samplingRate,gui)
    fltk.Fl_repeat_timeout(samplingRate,uiCallback)
    

def openGUI(samplingRate = 0.005):
    
    # add model
    teapot = OBJ('./teapot.obj',scale=1)
    gui.addModel('teapot',teapot.initOBJ,obj=teapot)
    bunny = OBJ('./bunny.obj',scale=10)
    gui.addModel('bunny',bunny.initOBJ,obj=bunny)
    # gui.moveModel('teapot',position = (5,0,0),rotation = (90,0,0))
    
    # open GUI window
    gui.window.show()
    
    # run callback function
    uiCallback = partial(callback,samplingRate,gui)
    fltk.Fl_add_timeout(samplingRate,uiCallback)
    
    # open application
    print("Start Program ...")
    return fltk.Fl_run()

if __name__ == '__main__':
    
    # declare port
    port = '/dev/pts/3' # ubuntu port
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