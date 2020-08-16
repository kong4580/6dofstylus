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
import numpy as np
class OpenGLWindow(Fl_Gl_Window):
    def __init__(self, xpos, ypos, width, height, label):
        Fl_Gl_Window.__init__(self, xpos, ypos, width, height, label)
        glColor3f(1.0, 1.0, 1.0)
        self.pose = [0,0,0,0,0,0,0,0,0]
        self.modelDicts = {'model':[],
                           'movepose':[],
                           'isModelInit':[],
                           'modelNum':0}
        self.models = []
        
        self.cursor = Model("cursor",drawFunc.point)
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
            firstTime = False 
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        
        # set camera view
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glFrustum(-1, 1, -1, 1, 1, 100)
        glTranslatef(self.scaleY,self.scaleZ,self.scaleX)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(0, 0, 10, 0, 0, 0, 0, 1, 0)
        
        # draw grid
        self.grid.drawModel()
        # draw cursor
        self.cursor.drawModel(position=(self.pose[1],self.pose[2],self.pose[0]),rotation=(self.pose[5],self.pose[3],self.pose[4]),showFrame=True)
        # draw origin
        self.origin.drawModel(position=(self.pose[6],self.pose[7],self.pose[8]),showFrame=True)
        # draw model
        if self.modelDicts['modelNum'] > 0:
            for idx in range(self.modelDicts['modelNum']):
                model = self.modelDicts['model'][idx]
                movePose = self.modelDicts['movepose'][idx]
                if self.modelDicts['isModelInit'][idx] == 0:
                    model.obj.initOBJ()
                    model.obj.current_vertices = model.obj.vertices.copy()
                    model.createOBB(showOBB=False)
                    model.obb.current_point = model.obb.points
                    model.obb.current_centroid = model.obb.centroid
                    model.obb.current_homo = np.dot(glGetFloatv(GL_MODELVIEW_MATRIX).T,model.obb.homo)
                    
                    self.modelDicts['isModelInit'][idx] = 1
                model.drawModel(position = movePose[0],rotation = movePose[1], showFrame=False)
                
                print("Is cursor in model OBB :",model.isPointInsideOBB(self.cursor.centerPosition))
                print("Is cursor in model convex :",model.isPointInsideConvexHull(self.cursor.centerPosition))
                
    def addModel(self,name,drawFunction=None,position=(0,0,0),rotation=(0,0,0),obj=None):
        model = Model(name,drawFunction,position,rotation,obj=obj)
        self.modelDicts['model'].append(model)
        self.modelDicts['movepose'].append([(position[0],position[1],position[2]),(rotation[0],rotation[1],rotation[2])])
        self.modelDicts['isModelInit'].append(0)
        self.modelDicts['modelNum'] = len(self.modelDicts['model'])
        
    def moveModel(self,name,position,rotation):
        for idx in range(self.modelDicts['modelNum']):
            model = self.modelDicts['model'][idx]
            if model.getName() == name:
                self.modelDicts['movepose'][idx][0] = (position[0],position[1],position[2])
                self.modelDicts['movepose'][idx][1] = (rotation[0],rotation[1],rotation[2])