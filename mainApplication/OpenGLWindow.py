from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
# from PIL.Image import *
from fltk import *
import sys
import math
import drawFunc
from functools import partial
from Model import Model
class OpenGLWindow(Fl_Gl_Window):
    def __init__(self, xpos, ypos, width, height, label):
        Fl_Gl_Window.__init__(self, xpos, ypos, width, height, label)
        glColor3f(1.0, 1.0, 1.0)
        self.pose = [0,0,0,0,0,0,0,0,0]
        self.modelDicts = {'model':[],
                           'movepose':[],
                           'modelNum':0}
        self.models = []
        
        self.cursor = Model("cursor",drawFunc.DrawCube)
        self.grid = Model("grid",drawFunc.Grid)
        self.origin = Model("origin",drawFunc.point)
        
    def __initGL(self): 
        glutInit(sys.argv) #add
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0) 
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)   
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        # gluPerspective(45.0, float(Width)/float(Height), 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        # initialize texture mapping
        glEnable(GL_TEXTURE_2D)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)
        
    def readPose(self,pose):
        self.pose = pose
    def readScaleX(self,scaleX):
        self.scaleX = scaleX
    def readScaleY(self,scaleY):
        self.scaleY = scaleY
    def readScaleZ(self,scaleZ):
        self.scaleZ = scaleZ
    def draw(self):
        firstTime = True
        if (firstTime):
            self.__initGL()
            firstTime = False # ondraw is called all the time
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glFrustum(-1, 1, -1, 1, 1, 100)
        glTranslatef(self.scaleY,self.scaleZ,self.scaleX)
        
        # DrawCube()
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        # glTranslatef(0,0,0)
        gluLookAt(0, 0, 10, 0, 0, 0, 0, 1, 0)
        
        self.grid.createModel()
        
        self.cursor.createModel(position=(self.pose[1],self.pose[2],self.pose[0]),rotation=(self.pose[5],self.pose[3],self.pose[4]),showFrame=True)
        
        self.origin.createModel(position=(self.pose[6],self.pose[7],self.pose[8]))
        
        if self.modelDicts['modelNum'] > 0:
            for idx in range(self.modelDicts['modelNum']):
                model = self.modelDicts['model'][idx]
                movePose = self.modelDicts['movepose'][idx]
                print(movePose)
                model.createModel(position = movePose[0],rotation = movePose[1], showFrame=True)
                
    def addModel(self,name,drawFunction=None,position=(0,0,0),rotation=(0,0,0)):
        model = Model(name,drawFunction,position,rotation)
        self.modelDicts['model'].append(model)
        self.modelDicts['movepose'].append([(position[0],position[1],position[2]),(rotation[0],rotation[1],rotation[2])])
        self.modelDicts['modelNum'] = len(self.modelDicts['model'])
        
    def moveModel(self,name,position,rotation):
        for idx in range(self.modelDicts['modelNum']):
            model = self.modelDicts['model'][idx]
            if model.getName() == name:
                self.modelDicts['movepose'][idx][0] = (position[0],position[1],position[2])
                self.modelDicts['movepose'][idx][1] = (rotation[0],rotation[1],rotation[2])