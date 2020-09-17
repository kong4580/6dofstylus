
import sys
from functools import partial

import fltk
import math
from scipy.spatial.transform import Rotation as R
import numpy as np

from drawFunc import *
from OpenGLWindow import OpenGLWindow
from Model import Model


class Gui():
    
    # init class GUI
    def __init__(self):
        
        # init fltk window
        self.__initWindow(size=(800,600),name="UI")
        
        # init opengl window class
        self.__initOpenglWindow(size=(0,0,600,600),name="opengl")
        
        # init side output
        self.__initOutputWidgetStorage()
        
        # set config
        self.cfg={"homeCfg":(5.23747141, -0.01606842, -0.3270202)}

        # set cursor scale
        self.cursorSpeed = 1
        
        # variable for revolute cursor mode
        self.cursorIsHold = False
        self.initHoldCursor = None
        self.offsetCursor = np.eye(4)
        
    # init fltk window
    def __initWindow(self,size=(800,600),name="UI"):
        print("Init GUI window ... ",end="")
        self.window = fltk.Fl_Window(size[0],size[1],name)
        print("Done !")
        
    # init opengl window
    def __initOpenglWindow(self,size=(0,0,600,600),name="opengl"):
        print("Init openGl window ... ",end="")
        self.openglWindow = OpenGLWindow(size[0],size[1],size[2],size[3],name)
        print("Done !")
        
    # init side output
    def __initOutputWidgetStorage(self):
        self.outputWidgetStorage = self.__createOutputWidget()
    
    # draw side output
    def __createOutputWidget(self):
        
        # side output area
        storage_area = []
        
        # create text box
        name=["TransX", "TransY", "TransZ", "RotX", "RotY", "RotZ","OriX","OriY","OriZ"]
        y=0
        for n in range(6):
            output = fltk.Fl_Output(660,y,60,25,name[n])
            y = y + 25
            output.align(fltk.FL_ALIGN_LEFT)
            output.value("0")
            storage_area.append(output)
        
        # create camera control slider
        y=150
        name_Scale = ["ScaleX","ScaleY","ScaleZ","RotX","RotY","RotZ"]
        for n in range(6):
            slider = fltk.Fl_Hor_Value_Slider(655, y, 145,25,name_Scale[n])
            y = y + 25
            slider.minimum(-50)
            slider.maximum(100)
            slider.value(0)
            slider.align(fltk.FL_ALIGN_LEFT)
            sldCallback = partial(Gui.__sliderScaleCB,self.openglWindow)
            slider.callback(sldCallback, n)
        
        # create mouse speed control slider
        slider = fltk.Fl_Hor_Value_Slider(655, y, 145,25,'mouseSpeed')
        slider.minimum(0)
        slider.maximum(10)
        slider.value(10)
        slider.align(fltk.FL_ALIGN_LEFT)
        sldCallback = partial(Gui.__sliderMouseCB,self)
        slider.callback(sldCallback, 4)
        
        # add value to opengl window class
        self.openglWindow.readScaleX(-10)
        self.openglWindow.readScaleY(0)
        self.openglWindow.readScaleZ(0)
        
        # return side output position and value
        return storage_area
           
           
    
    # main call back update ui
    def updateUI(self,cursorPose,buttonStates,scale=20,cursorTransform=None):
        
        # convert pose format
        ### now not use ###
        cvtedPose = self.__cvtPose(cursorPose,self.cursorSpeed)
        
        # add cursor pose to openglWindow class
        ### now not use ###
        self.openglWindow.readPose(cvtedPose)
        
        # read cursor transform from stylus and scaling
        cursorTransform[0:3,3] = cursorTransform[0:3,3].copy() * 0.05
        
        # offset home position
        cursorTransform[0,3] -= self.cfg["homeCfg"][0]
        cursorTransform[1,3] -= self.cfg["homeCfg"][1]
        cursorTransform[2,3] -= self.cfg["homeCfg"][2]
        
        # cursorTransform = np.dot(np.linalg.inv(self.openglWindow.cursorTransform.copy()),cursorTransform.copy())
        # cursorTransform[0:3,3] = cursorTransform[0:3,3].copy() * self.cursorSpeed
        # scale = self.cursorSpeed*np.eye(4)
        # cursorTransform[0:3,3] = np.dot(self.cursorSpeed,cursorTransform[0:3,3])
        self.openglWindow.cursorSpeed = self.cursorSpeed
        
        # offset mouse mode
        if self.openglWindow.flags['offsetMode']:
            
            # find delta transform between first transform that trigger mode and last transform that turn off this mode
            self.offsetCursor = np.dot(cursorTransform,np.linalg.inv(self.openglWindow.cursorTransform.copy()))
            # cursorTransform = np.dot(np.linalg.inv(self.openglWindow.cursorTransform.copy()),cursorTransform.copy())
            
            
            # offset only translation
            # self.offsetCursor[0:3,0:3] = np.eye(3)
            
        else:
            
            # new cursor transform is
            # inv(offsetCursor) * current cursor transform from stylus
            self.openglWindow.cursorTransform = np.dot(np.linalg.inv(self.offsetCursor),cursorTransform)
            # self.openglWindow.cursorTransform = np.dot(self.openglWindow.cursorTransform.copy(),cursorTransform)
            
            # update opengl window
            self.openglWindow.redraw()
            
            # update side output value
            self.__updateOutput(cvtedPose)
            
        # check left mouse is clicked
        if buttonStates[0] == 1 and buttonStates[1] == 0:
            
            # pass
            
            # check what model is clicked
            print("left click!")
            selectedModel = self.selectModel()
            
            # show all model that is selected
            for model in selectedModel:
                print("selectModel = ",model.name)
        
        # check right mouse is clicked
        elif buttonStates[0] == 0 and buttonStates[1] == 1: 
            
            # pass     
            # realease model
            print("releaseModel")
            self.releaseModel()
            
        return
    
    
    
    # update output function
    def __updateOutput(self,pos):
        for i in range(6):
            a=float("{:.2f}".format(pos[i]))
            self.outputWidgetStorage[i].value(str(a))
    
    # convert pose from stylus to opengl
    # from [rotX, rotY, rotZ, tranX, tranY, tranZ]
    # to [tranX, tranY, tranZ, rotX, rotY, rotZ]
    # scaling with cursor speed 
    ### now not use ###
    def __cvtPose(self,pose,scale,offset=True):
        
        # init buffer
        newPose = pose.copy()
        real = [0]*9
        
        # switch position and convert radians to degrees
        for i in range(len(newPose)):
            
            if i <3:
                real[i+3] = ((newPose[i])*180)/(math.pi)
            else:
                real[i-3] = newPose[i]*scale
        
        # offset home postion
        if offset:
            real[0] -= self.cfg["homeCfg"][0]
            real[1] -= self.cfg["homeCfg"][1]
            real[2] -= self.cfg["homeCfg"][2]
                  
        # return convert pose
        return real
    
    # add model to opengl window class
    def addModel(self,name,drawFunction = None,position=(0,0,0),rotation=(0,0,0),obj=None):
        print("Add model name",name)
        self.openglWindow.addModel(name,drawFunction,position,rotation,obj=obj)
        print("Done!")
    
    # move model to specified position and rotation
    ### now not use ###
    def moveModel(self,name,pose=None,position=None,rotation=None):
        
        # if input is not pose massage
        if pose!=None:
            cvtedPose = self.__cvtPose(pose,scale=1,offset=False)
            position = (cvtedPose[0],cvtedPose[1],cvtedPose[2])
            rotation = (cvtedPose[3],cvtedPose[4],cvtedPose[5])
            self.openglWindow.moveModel(name,position,rotation)
        
        # if input is position and rotation
        elif position != None and rotation != None:
            self.openglWindow.moveModel(name,position,rotation)
            
    # select model
    def selectModel(self):
        
        # create selected model buffer
        selectModel = []
        
        # run through all models in opengl window class
        for model in self.openglWindow.modelDicts['model']:
            
            # if cursor position is in model
            if model.isPointInsideConvexHull(self.openglWindow.cursor.centerPosition):
                
                # model is selected
                model.isSelected = True
                
                # add model to selected model buffer
                selectModel.append(model)
        
        # return selected model buffer
        return selectModel
    
    # release model
    def releaseModel(self):
        
        # run through all models in opengl window class
        for model in self.openglWindow.modelDicts['model']:
            
            # change model to not selected
            model.isSelected = False
    
    # set cursor speed
    def setMouseSpeed(self,speed):
        self.cursorSpeed = speed
    
    
    
    # Callback
    # camera control slider call back
    @staticmethod
    def __sliderScaleCB(opengl,widget,v):
        a = widget.value()
        size = (a/10)
        if v == 0:
            size = size -10
            opengl.readScaleX(size)
        elif v == 1:
            opengl.readScaleY(size)
        elif v==2:
            opengl.readScaleZ(size)
        elif v == 3:
            # size = size -10
            opengl.readRotX(size)
        elif v == 4:
            opengl.readRotY(size)
        elif v==5:
            opengl.readRotZ(size)
        opengl.redraw()
    
    # mouse speed slider callback
    @staticmethod
    def __sliderMouseCB(GUI,widget,v):
        mouseSpeed = widget.value()/10
        GUI.setMouseSpeed(mouseSpeed)
    
    