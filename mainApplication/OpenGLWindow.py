from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
# from PIL.Image import *
from fltk import *
import sys
import math
from drawFunc import *
from functools import partial
class OpenGLWindow(Fl_Gl_Window):
    def __init__(self, xpos, ypos, width, height, label):
        Fl_Gl_Window.__init__(self, xpos, ypos, width, height, label)
        glColor3f(1.0, 1.0, 1.0)
        self.pose = [0,0,0,0,0,0,0,0,0]
        
    def __initGL(self): 
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
        glPushMatrix()
        Grid()
        glPopMatrix()
        glPushMatrix()
        glTranslatef(self.pose[1],self.pose[2],self.pose[0])
        glRotatef(self.pose[5],0,1,0) #y#z
        glRotatef(self.pose[3],0,0,1) #z#x
        glRotatef(self.pose[4],1,0,0) #x#y
        glTranslatef(-self.pose[1],-self.pose[2],-self.pose[0])
        glTranslatef(self.pose[1],self.pose[2],self.pose[0])
        DrawCube()
        coordinate()
        # glLoadIdentity()
        glPopMatrix()
        glPushMatrix()
        glTranslatef(self.pose[6],self.pose[7],self.pose[8])
        coordinate()
        point()
        glPopMatrix()