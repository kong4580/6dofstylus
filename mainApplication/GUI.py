from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
# from PIL.Image import *
from fltk import *
import sys
import math
from drawFunc import *
from functools import partial
from OpenGLWindow import OpenGLWindow

class Gui():
    def __init__(self):
        self.__initWindow(size=(800,600),name="UI")
        self.__initOpenglWindow(size=(0,0,600,600),name="opengl")
        self.__initOutputWidgetStorage()
        self.cfg={"homeCfg":(5.231761051519499,-0.2290998697402224,-1.8696414028770085)}
        self.model=None
        
    def __initWindow(self,size=(800,600),name="UI"):
        print("Init GUI window ... ",end="")
        self.window = Fl_Window(size[0],size[1],name)
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
            output = Fl_Output(660,y,60,25,name[n])
            y = y + 25
            output.align(FL_ALIGN_LEFT)
            output.value("0")
            storage_area.append(output)
        y=150
        name_Scale = ["ScaleX","ScaleY","ScaleZ"]
        for n in range(3):
            slider = Fl_Hor_Value_Slider(655, y, 145,25,name_Scale[n])
            y = y + 25
            slider.minimum(-100)
            slider.maximum(100)
            slider.value(0)
            slider.align(FL_ALIGN_LEFT)
            sldCallback = partial(Gui.__sliderScaleCB,self.openglWindow)
            slider.callback(sldCallback, n)
        self.openglWindow.readScaleX(-10)
        self.openglWindow.readScaleY(0)
        self.openglWindow.readScaleZ(0)
        return storage_area
           
    def __updateSlider(self,pos):
        for i in range(6):
            a=float("{:.2f}".format(pos[i]))
            self.outputWidgetStorage[i].value(str(a))
    
    def __cvtPose(self,pose,scale):
        newPose = pose.copy()
        real = [0]*9
        
        for i in range(len(newPose)):
            
            if i <3:
                real[i+3] = ((newPose[i])*360)/(2*math.pi)
            else:
                real[i-3] = newPose[i]/scale
                
        real[0] -= self.cfg["homeCfg"][0]
        real[1] -= self.cfg["homeCfg"][1]
        real[2] -= self.cfg["homeCfg"][2]
        return real
    
    def updateUI(self,pose,buttonStates,scale=20):
        cvtedPose = self.__cvtPose(pose,scale)
        self.openglWindow.readPose(cvtedPose)
        self.openglWindow.redraw()
        self.__updateSlider(cvtedPose)
    
    def addModel(self):
        # add 3D model
        pass
    
    def addBackdrop(self):
        # add addBackdrop
        pass
    
    
    
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
        else:
            opengl.readScaleZ(size)
        opengl.redraw()