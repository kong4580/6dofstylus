from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from PIL.Image import *
from fltk import *
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
            pose = stylus.getEndTransforms(jointStates)
            # update gui
            print("pose",pose)
            gui.moveModel('teapot',[0,0,0,-10,0,0])
            gui.updateUI(pose,buttonStates,scale=20)
            
        # update button state command    
        if command == 0xFE:
            # get button states
            pass
        
    uiCallback = partial(callback,samplingRate,gui)
    Fl_repeat_timeout(samplingRate,uiCallback)
    

def openGUI(samplingRate = 0.005):
    
    # add model
    bunny = OBJ('./teapot.obj')
    gui.addModel('teapot',bunny.initOBJ,obj=bunny)
    # open GUI window
    gui.window.show()
    
    # run callback function
    uiCallback = partial(callback,samplingRate,gui)
    Fl_add_timeout(samplingRate,uiCallback)
    
    # open application
    print("Start Program ...")
    return Fl_run()

if __name__ == '__main__':
    
    # declare port
    port = '/dev/pts/4' # ubuntu port
    # port = '/dev/ttyUSB0' # arduino port
    
    # declare constants
    samplingRate = 0.005
    serialTimeOut = samplingRate
    
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