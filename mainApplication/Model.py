from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
# from PIL.Image import *
from fltk import *
import sys
import math
from drawFunc import *

class Model():
    def __init__(self,name,drawFunction = None,position=(0,0,0),rotation=(0,0,0)):
        self.name = name
        self.centerPosition = position
        self.rotation = rotation
        self.drawFunction = drawFunction
    
        
    def createModel(self,position=(0,0,0),rotation=(0,0,0),showFrame=False):
        
        if self.drawFunction != None:
            self.__updatePosition(position,rotation)
            glPushMatrix()
            glTranslatef(self.centerPosition[0],self.centerPosition[1],self.centerPosition[2])
            glRotatef(self.rotation[0],0,1,0) #y#z
            glRotatef(self.rotation[1],0,0,1) #z#x
            glRotatef(self.rotation[2],1,0,0) #x#y
            glTranslatef(-self.centerPosition[0],-self.centerPosition[1],-self.centerPosition[2])
            glTranslatef(self.centerPosition[0],self.centerPosition[1],self.centerPosition[2])
            self.drawFunction()
            if showFrame:
                coordinate()
            # glLoadIdentity()
            glPopMatrix()
        else:
            print("No model draw function")
            
    def __updatePosition(self,position=(0,0,0),rotation=(0,0,0)):
        self.centerPosition = position
        self.rotation = rotation

    def getName(self):
        return self.name