
# from PIL.Image import *
# from fltk import *
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
    def __init__(self):
        self.__initWindow(size=(800,600),name="UI")
        self.__initOpenglWindow(size=(0,0,600,600),name="opengl")
        self.__initOutputWidgetStorage()
        
        self.cfg={"homeCfg":(5.23747141, -0.01606842, -0.3270202)}

        self.cursorSpeed = 0.05
        self.cursorIsHold = False
        
        self.initHoldCursor = None
        self.offsetCursor = np.eye(4)
    def __initWindow(self,size=(800,600),name="UI"):
        print("Init GUI window ... ",end="")
        self.window = fltk.Fl_Window(size[0],size[1],name)
        print("Done !")
        
    def __initOpenglWindow(self,size=(0,0,600,600),name="opengl"):
        print("Init openGl window ... ",end="")
        self.openglWindow = OpenGLWindow(size[0],size[1],size[2],size[3],name)
        print("Done !")
        
    def __initOutputWidgetStorage(self):
        self.outputWidgetStorage = self.__createOutputWidget()
        
    def __createOutputWidget(self):
        storage_area = []
        name=["TransX", "TransY", "TransZ", "RotX", "RotY", "RotZ","OriX","OriY","OriZ"]
        y=0
        for n in range(6):
            output = fltk.Fl_Output(660,y,60,25,name[n])
            y = y + 25
            output.align(fltk.FL_ALIGN_LEFT)
            output.value("0")
            storage_area.append(output)
            
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
        
        slider = fltk.Fl_Hor_Value_Slider(655, y, 145,25,'mouseSpeed')
        slider.minimum(0)
        slider.maximum(10)
        slider.value(5)
        slider.align(fltk.FL_ALIGN_LEFT)
        sldCallback = partial(Gui.__sliderMouseCB,self)
        slider.callback(sldCallback, 4)
        
        self.openglWindow.readScaleX(-10)
        self.openglWindow.readScaleY(0)
        self.openglWindow.readScaleZ(0)
        return storage_area
           
    def __updateOutput(self,pos):
        for i in range(6):
            a=float("{:.2f}".format(pos[i]))
            self.outputWidgetStorage[i].value(str(a))
    
    def __cvtPose(self,pose,scale,offset=True):
        newPose = pose.copy()
        real = [0]*9
        
        for i in range(len(newPose)):
            
            if i <3:
                real[i+3] = ((newPose[i])*180)/(math.pi)
            else:
                real[i-3] = newPose[i]*scale
        # print(real)
        if offset:
            real[0] -= self.cfg["homeCfg"][0]
            real[1] -= self.cfg["homeCfg"][1]
            real[2] -= self.cfg["homeCfg"][2]
            # real[4] -= 90
        
        
        return real
    
    def updateUI(self,cursorPose,buttonStates,scale=20,cursorTransform=None):
        cvtedPose = self.__cvtPose(cursorPose,self.cursorSpeed)
        self.openglWindow.readPose(cvtedPose)
        cursorTransform[0:3,3] = cursorTransform[0:3,3].copy() * 0.05
        
        
        cursorTransform[0,3] -= self.cfg["homeCfg"][0]
        cursorTransform[1,3] -= self.cfg["homeCfg"][1]
        
        cursorTransform[2,3] -= self.cfg["homeCfg"][2]
        
        
        if self.openglWindow.flags['offsetMode']:
            self.offsetCursor = np.dot(cursorTransform,np.linalg.inv(self.openglWindow.cursorTransform.copy()))
            # self.offsetCursor[0:3,0:3] = np.eye(3)
            # self.offsetCursor = cursorTransform
            print(self.offsetCursor)
        else:
        
            self.openglWindow.cursorTransform = np.dot(np.linalg.inv(self.offsetCursor),cursorTransform)
            
            self.openglWindow.redraw()
            self.__updateOutput(cvtedPose)
            
        # check any model is selected when mouse clicked
        if buttonStates[0] == 1 and buttonStates[1] == 0:
            
            # pass
            print("left click!")
            selectedModel = self.selectModel()
            for model in selectedModel:
                print("selectModel = ",model.name)
                
        elif buttonStates[0] == 0 and buttonStates[1] == 1: 
            # pass     
            print("releaseModel")
            self.releaseModel()
        # elif buttonStates[0] == 1 and buttonStates[1] == 1:      
        #     print("Hold cursor")
        #     # self.cursorIsHold = not self.cursorIsHold
        #     self.offsetCursor = np.dot(cursorTransform,np.linalg.inv(self.openglWindow.cursorTransform.copy()))
        #     self.offsetCursor[0:3,0:3] = np.eye(3)
        #     print(self.offsetCursor)
        
    
        
    def addModel(self,name,drawFunction = None,position=(0,0,0),rotation=(0,0,0),obj=None):
        print("Add model name",name)
        self.openglWindow.addModel(name,drawFunction,position,rotation,obj=obj)
        print("Done!")
        
    def moveModel(self,name,pose=None,position=None,rotation=None):
        if pose!=None:
            cvtedPose = self.__cvtPose(pose,scale=1,offset=False)
            position = (cvtedPose[0],cvtedPose[1],cvtedPose[2])
            rotation = (cvtedPose[3],cvtedPose[4],cvtedPose[5])
            self.openglWindow.moveModel(name,position,rotation)
        elif position != None and rotation != None:
            self.openglWindow.moveModel(name,position,rotation)
            
    
    def selectModel(self):
        selectModel = []
        for model in self.openglWindow.modelDicts['model']:
            if model.isPointInsideConvexHull(self.openglWindow.cursor.centerPosition):
                model.isSelected = True
                selectModel.append(model)
        return selectModel
    
    def releaseModel(self):
        
        for model in self.openglWindow.modelDicts['model']:
            model.isSelected = False
                
    def setMouseSpeed(self,speed):
        self.cursorSpeed = speed
    
    
    # Callback
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
    
    @staticmethod
    def __sliderMouseCB(GUI,widget,v):
        mouseSpeed = widget.value()/10
        GUI.setMouseSpeed(mouseSpeed)
    
    