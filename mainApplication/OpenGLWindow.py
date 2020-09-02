# from OpenGL.GL import *
# from OpenGL.GLUT import *
# from OpenGL.GLU import *
from OpenGL import GL, GLUT, GLU
# from PIL.Image import *

from PIL import Image
from PIL import ImageOps

import fltk
import sys
import math
import drawFunc
from functools import partial
from Model import Model,OBJ
import numpy as np
from scipy.spatial.transform import Rotation as R

class OpenGLWindow(fltk.Fl_Gl_Window):
    def __init__(self, xpos, ypos, width, height, label):
        fltk.Fl_Gl_Window.__init__(self, xpos, ypos, width, height, label)
        # glColor3f(1.0, 1.0, 1.0)
        self.cursorTransform = np.eye(4)
        self.pose = [0,0,0,0,0,0,0,0,0]
        self.modelDicts = {'model':[],
                           'movepose':[],
                           'isModelInit':[],
                           'modelNum':0}
        self.nameNumber = 0
        
        
        self.cursor = Model('cursor',drawFunc.point)
        
        self.grid = Model("grid",drawFunc.Grid)
        self.origin = Model("origin",drawFunc.point)
        
        self.backdropImageFile = Image.open( "backdrop_0.jpg" )
        
        self.showModel= True
        
        self.imageObj  = self.backdropImageFile.tobytes("raw", "RGBX", 0, -1)
        
        self.texid = None
        self.snapMode = False
        
    def __initGL(self): 
        GLUT.glutInit(sys.argv) #add
        GL.glClearColor(0.0, 0.0, 0.0, 0.0)
        GL.glClearDepth(1.0) 
        GL.glDepthFunc(GL.GL_LESS)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glShadeModel(GL.GL_SMOOTH)   
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        # GL.gluPerspective(45.0, float(Width)/float(Height), 0.1, 100.0)
        GL.glMatrixMode(GL.GL_MODELVIEW)
        # initialize texture mapping
        GL.glPixelStorei(GL.GL_UNPACK_ALIGNMENT,1)
        
        GL.glEnable(GL.GL_TEXTURE_2D)
        GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_NEAREST)
        GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_NEAREST)
        GL.glTexEnvf(GL.GL_TEXTURE_ENV, GL.GL_TEXTURE_ENV_MODE, GL.GL_DECAL)
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
        
        self.lighting()
        GL.glClearColor(0.0, 0.0, 0.0, 0.0)
        GL.glClearDepth(1.0) 
        GL.glDepthFunc(GL.GL_LESS)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        
        GL.glDisable(GL.GL_DEPTH_TEST)
        self.drawBackdrop()
        GL.glEnable(GL.GL_DEPTH_TEST)

        # set camera view
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        
        GL.glFrustum(-1, 1, -1, 1, 1, 100)
        GL.glTranslatef(self.scaleY,self.scaleZ,self.scaleX)
        
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()
        GLU.gluLookAt(0, 0, 10, 0, 0, 0, 0, 1, 0)
        
        
        self.cursor.show = not self.snapMode
        
        # self.cursor.drawModel(position=(self.pose[1],self.pose[2],self.pose[0]),rotation=(self.pose[5],self.pose[3],self.pose[4]),showFrame=not self.snapMode)
        
        self.cursor.drawMatrixModel(self.cursorTransform,showFrame=not self.snapMode)
        
        
        # draw origin
        # self.origin.drawModel(position=(self.pose[6],self.pose[7],self.pose[8]),showFrame=True)
        
        # draw model
        if self.modelDicts['modelNum'] > 0:
            for idx in range(self.modelDicts['modelNum']):
                model = self.modelDicts['model'][idx]
                movePose = self.modelDicts['movepose'][idx]
                model.show = self.showModel
                
                if self.modelDicts['isModelInit'][idx] == 0:
                    model.initModel(matrixView = GL.glGetFloatv(GL.GL_MODELVIEW_MATRIX).T)
                    self.modelDicts['isModelInit'][idx] = 1
                    
                if model.isSelected:
                    targetPosition,targetRotation,newM = model.followCursor(self.cursor)
                    
                else:
                    model.cursorM = None
                    model.cursorPose = None
                    
                    targetPosition,targetRotation,newM = model.centerPosition,model.rotation,model.currentM
                    
                # model.drawModel(position = targetPosition,rotation = targetRotation,showFrame=False)
                
                model.drawMatrixModel(newM,showFrame=False)
                
                
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
                
    def handle(self,event):
        xMousePosition = fltk.Fl.event_x()
        yMousePosition = fltk.Fl.event_y()
        
        if event == fltk.FL_PUSH and fltk.Fl.event_button() == 1: #push left mouse button handle
            print(xMousePosition,yMousePosition)
            return 1
        
        elif event == fltk. FL_KEYUP: #keyboard handle
            print("key press : ",chr(fltk.Fl.event_key())) #check key #type:int 
            
            if fltk.Fl.event_key() == ord('s'):
                self.snapMode = not self.snapMode
                self.redraw()
                print("set Snapmode",self.snapMode)
                
            if fltk.Fl.event_key() == ord(' ') and self.snapMode:
                backdropName = "backdrop_" + str(self.nameNumber)
                self.snap(backdropName)
                self.nameNumber = self.nameNumber + 1
                
            if fltk.Fl.event_key() == ord('d'):
                self.checkIoU()
                
            if fltk.Fl.event_key() == ord('q'):
                self.showModel = not self.showModel
                
            if fltk.Fl.event_key() == ord('t'):
                self.rotateMode = False
                
            fltk.Fl_check()
            return 1
        
        else:
            return fltk.Fl_Gl_Window.handle(self, event)
    
    def snap(self,name, save = True, binary = False):
        
        print("snap!")
        GL.glPixelStorei(GL.GL_PACK_ALIGNMENT, 1)
        data = GL.glReadPixels(0, 0, 600, 600, GL.GL_RGBA, GL.GL_UNSIGNED_BYTE)
        image = Image.frombytes("RGBX", (600, 600), data)
        
        image = ImageOps.flip(image) 
        imagename = name + ".jpg"
        print("save at :",imagename)
        
        if save:
            if binary:
                imagenp = np.asarray(image.convert('RGB')).copy()
                imagenp[imagenp> 128] = 200
                imagenp = Image.fromarray(imagenp.astype('uint8'), 'RGB')
                imagenp.save(imagename, 'JPEG')
            image.save(imagename, 'JPEG')
            
        else:
            image = image.convert('L')
            return image
        
    def checkIoU(self):
        self.snapMode = True
        self.redraw()
        fltk.Fl_check()
        
        backdropImg = self.backdropImageFile.convert('L')
        bw = np.asarray(backdropImg).copy()
        bw[bw < 80] = 0    # Black
        bw[bw >= 80] = 255 # White
        
        testimg = self.snap("test",save=False)
        testbw = np.asarray(testimg).copy()
        testbw[testbw < 80] = 0    # Black
        testbw[testbw >= 80] = 255 # White
        
        intersect = np.bitwise_and(bw,testbw)
        union = np.bitwise_or(bw,testbw)
        iou = np.sum(intersect==255)/np.sum(union==255)
        print("IoU: ",iou)
        
        self.snapMode = False
        self.redraw()
        fltk.Fl_check()
        
        return iou
    
    def lighting(self):
        GL.glEnable(GL.GL_LIGHTING)
        GL.glEnable(GL.GL_LIGHT0)
        ambientLight = [0.2, 0.2, 0.2, 1.0 ] # RGB and alpha channels
        diffuseLight = [0.8, 0.8, 0.8, 1.0 ] # RGB and alpha channels
        GL.glLightfv(GL.GL_LIGHT0, GL.GL_AMBIENT, ambientLight)
        GL.glLightfv(GL.GL_LIGHT0, GL.GL_DIFFUSE, diffuseLight)
        GL.glEnable(GL.GL_COLOR_MATERIAL)
        GL.glColorMaterial(GL.GL_FRONT, GL.GL_AMBIENT_AND_DIFFUSE)
        position = [ 100.0, 0.0, -1.0, 1.0 ]
        GL.glLightfv(GL.GL_LIGHT0, GL.GL_POSITION, position)

    def drawBackdrop(self):
        
        # set camera view
        GL.glMatrixMode(GL_PROJECTION)
        GL.glLoadIdentity()
        GL.glOrtho( -0.5, 0.5, -0.5, 0.5, -0.5, 1 )
        GL.glMatrixMode(GL_MODELVIEW)
        GL.glLoadIdentity()

        imageW = self.backdropImageFile.size[0]
        imageH = self.backdropImageFile.size[1]
        
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
            
        if not self.snapMode:
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