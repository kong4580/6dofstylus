from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL import GL
# from PIL.Image import *

from PIL import Image
from PIL import ImageOps
from fltk import *
import sys
import math
import drawFunc
from functools import partial
from Model import Model,OBJ
import numpy as np
from scipy.spatial.transform import Rotation as R
class OpenGLWindow(Fl_Gl_Window):
    def __init__(self, xpos, ypos, width, height, label):
        Fl_Gl_Window.__init__(self, xpos, ypos, width, height, label)
        # glColor3f(1.0, 1.0, 1.0)
        self.pose = [0,0,0,0,0,0,0,0,0]
        self.modelDicts = {'model':[],
                           'movepose':[],
                           'isModelInit':[],
                           'modelNum':0}
        self.models = []
        # cursor = OBJ('./teapot.obj',scale=0.1)
        # # gui.addModel('cursor',cursor.initOBJ,obj=cursor)
        # self.cursor = Model('cursor',cursor.initOBJ,obj=cursor)
        self.initCursor=False
        self.cursor = Model('cursor',drawFunc.point)
        
        self.grid = Model("grid",drawFunc.Grid)
        self.origin = Model("origin",drawFunc.point)
        
        self.imageFile = Image.open( "1.jpg" )
        self.imageObj  = self.imageFile.tobytes("raw", "RGBX", 0, -1)
        
        self.texid = None
        
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
        glPixelStorei(GL_UNPACK_ALIGNMENT,1)
        
        glEnable(GL_TEXTURE_2D)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)
        # glDisable(GL_TEXTURE_2D)
        
        
    def readPose(self,pose):
        self.pose = pose
        
    def readScaleX(self,scaleX):
        self.scaleX = scaleX
        
    def readScaleY(self,scaleY):
        self.scaleY = scaleY
        
    def readScaleZ(self,scaleZ):
        self.scaleZ = scaleZ

    # main opengl callback 
    def draw(self):
        
        # firstTime = True
        # if (firstTime):
        #     self.__initGL()
        #     firstTime = False 
        
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0) 
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        
        
        # set camera view
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        GL.glOrtho( -1, 1, -1, 1, -1, 1 )
        # glFrustum(-1, 1, -1, 1, 1, 1000)
        # glTranslatef(self.scaleY,self.scaleZ,self.scaleX)
        
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        # gluLookAt(0, 0, 10, 0, 0, 0, 0, 1, 0)
        
    
        
        imageW = self.imageFile.size[0]
        imageH = self.imageFile.size[1]
        
        if( self.texid == None ):
        
            self.texid = GL.glGenTextures( 1 )
            GL.glEnable( GL.GL_TEXTURE_2D )
            GL.glBindTexture( GL.GL_TEXTURE_2D, self.texid )
            GL.glPixelStorei( GL.GL_UNPACK_ALIGNMENT,1 )
            GL.glTexImage2D( GL.GL_TEXTURE_2D, 0, 3, imageW, imageH, 0, 
                             GL.GL_RGBA, GL.GL_UNSIGNED_BYTE, self.imageObj )
        
            GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_S, GL.GL_CLAMP)
            GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_T, GL.GL_CLAMP)
            GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_S, GL.GL_REPEAT)
            GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_T, GL.GL_REPEAT)
            GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_NEAREST)
            GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_NEAREST)
            # # GL.glTexEnvf(GL.GL_TEXTURE_ENV, GL.GL_TEXTURE_ENV_MODE, GL.GL_DECAL)
        
        # print( self.texid )
        
        GL.glBindTexture( GL.GL_TEXTURE_2D, self.texid )
        GL.glEnable( GL.GL_TEXTURE_2D )
        
        GL.glColor4f(1,1,1,1)
        GL.glBegin(GL.GL_QUADS)

        GL.glTexCoord2f(0.0, 0.0)
        GL.glVertex3f( -0.5, -0.5, 0 )
        
        GL.glTexCoord2f(1.0, 0.0)
        GL.glVertex3f( 0.5, -0.5, 0 )
        
        GL.glTexCoord2f(1.0, 1.0)
        GL.glVertex3f( 0.5, 0.5, 0 )
        
        GL.glTexCoord2f(0.0, 1.0)
        GL.glVertex3f( -0.5, 0.5, 0 )
        
        GL.glEnd()
        
        GL.glDisable( GL.GL_TEXTURE_2D )
        
        
        
        
        # glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        
        
        # set camera view
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        # GL.glOrtho( -1, 1, -1, 1, -1, 1 )
        glFrustum(-1, 1, -1, 1, 1, 1000)
        glTranslatef(self.scaleY,self.scaleZ,self.scaleX)
        
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(0, 0, 10, 0, 0, 0, 0, 1, 0)
        
        # try:
        # self.DrawGLScene('1.jpg')
        # return
        # except Exception as e:
            # print(e)
        # draw grid
        self.grid.drawModel()
        # draw cursor
        # if not self.initCursor:
        #     self.cursor.obj.initOBJ()
        #     self.cursor.obj.current_vertices = self.cursor.obj.vertices.copy()
        #     self.cursor.createOBB(showOBB=False)
        #     self.cursor.obb.current_point = self.cursor.obb.points
        #     self.cursor.obb.current_centroid = self.cursor.obb.centroid
        #     self.cursor.obb.current_homo = np.dot(glGetFloatv(GL_MODELVIEW_MATRIX).T,self.cursor.obb.homo)
        #     self.initCursor = True
        self.cursor.drawModel(position=(self.pose[1],self.pose[2],self.pose[0]),rotation=(self.pose[5],self.pose[3],self.pose[4]),showFrame=True)
        # draw origin
        self.origin.drawModel(position=(self.pose[6],self.pose[7],self.pose[8]),showFrame=True)
        # draw model
        if self.modelDicts['modelNum'] > 0:
            for idx in range(self.modelDicts['modelNum']):
                model = self.modelDicts['model'][idx]
                movePose = self.modelDicts['movepose'][idx]
                if self.modelDicts['isModelInit'][idx] == 0:
                    model.initModel(matrixView = glGetFloatv(GL_MODELVIEW_MATRIX).T)
                    # model.obj.initOBJ()
                    # model.obj.current_vertices = model.obj.vertices.copy()
                    # model.createOBB(showOBB=False)
                    # model.obb.current_point = model.obb.points
                    # model.obb.current_centroid = model.obb.centroid
                    # model.obb.current_homo = np.dot(glGetFloatv(GL_MODELVIEW_MATRIX).T,model.obb.homo)
                    
                    self.modelDicts['isModelInit'][idx] = 1
                if model.isSelected:
                    
                    # if model.cursorPose == None:
                    #     model.cursorPose = self.cursor.rotation
                    #     model.currentRotation = model.rotation
                        
                    # deltaRot = np.asarray(list(self.cursor.rotation)) - np.asarray(list(model.cursorPose))
                    # newRotation = model.currentRotation
                    
                    # if deltaRot[0]**2+deltaRot[1]**2+deltaRot[2]**2 >=0.1: 
                    #     newRotation = model.currentRotation + deltaRot
                    
                    
                    targetPosition,targetRotation = model.followCursor(self.cursor)
                    print(model.centerPosition)
                    # model.drawModel(position = targetPosition,rotation = targetRotation,showFrame=True)
                    # model.cursorPose = tuple(np.asarray(list(self.cursor.rotation))+ deltaRot)
                    
                else:
                    
                    model.cursorPose = None
                    targetPosition,targetRotation = model.centerPosition,model.rotation
                    # model.drawModel(position = model.centerPosition,rotation =model.rotation, showFrame=True)
                model.drawModel(position = targetPosition,rotation = targetRotation,showFrame=True)
                
                # print("Is cursor in model OBB :",model.isPointInsideOBB(self.cursor.centerPosition))
                # print("Is cursor in model convex :",model.isPointInsideConvexHull(self.cursor.centerPosition))
                
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
    
    def loadImage(self,Filename):
        image = open(Filename)
        
        ix = image.size[0]
        iy = image.size[1]
        image = image.tobytes("raw", "RGBX", 0, -1)
        
        self.texid = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, self.texid)
        glPixelStorei(GL_UNPACK_ALIGNMENT,1)
        glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, image)
        # glBindTexture(GL_TEXTURE_2D, 0)
        
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        # glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)
        return [ix,iy]
    def DrawGLScene(self,Filename):
        # glPushMatrix()
        # pass
        size = [600,600]
        # if self.texid == None:
        #     size = self.loadImage(Filename)

        #image size
        # size[0] = 600
        # size[1] = 600
        # glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        
        # glLoadIdentity()

        #how far the image is
        # glTranslatef(0.0,0.0,-600.0)
        
        glEnable(GL_TEXTURE_2D)
        # glBindTexture(GL_TEXTURE_2D, self.texid)
        
        
        glColor4f(1,1,1,1)
        glBegin(GL_QUADS)

        
        glTexCoord2f(0.0, 0.0)
        glVertex3f(-size[0]/2,-size[1]/2,-1)
        # glTexCoord2f(0.0, 0.0)
        glTexCoord2f(1.0, 0.0)
        glVertex3f(size[0]/2,-size[1]/2,-1)
        # glTexCoord2f(1.0, 0.0)
        glTexCoord2f(1.0, 1.0)
        glVertex3f(size[0]/2,size[1]/2,-1)
        # glTexCoord2f(1.0, 1.0)
        glTexCoord2f(0.0, 1.0)
        glVertex3f(-size[0]/2,size[1]/2,-1)
        # glTexCoord2f(0.0, 1.0)
        # glutSwapBuffers()
        print("e")
        print(self.texid)
        glEnd()
        # glBindTexture(GL_TEXTURE_2D, 0)
        
        print('jj')
        glDisable(GL_TEXTURE_2D)
        
        print('aaa')
        
        # glPopMatrix()
        print('eee')

# if __name__ == "__main__":
    
#     import fltk
    
#     tmp = OpenGLWindow(0,0,480,480, "")
#     tmp.show()
    
#     fltk.Fl_run()