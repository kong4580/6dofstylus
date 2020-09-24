import sys
import math
from functools import partial
import time

from OpenGL import GL, GLUT, GLU
import fltk
from PIL import Image
from PIL import ImageOps
import numpy as np
from scipy.spatial.transform import Rotation as R
import csv

import drawFunc
from Model import Model,OBJ
from Controller import MainController,StylusController
class OpenGLWindow(fltk.Fl_Gl_Window):
    
    # init opengl window class
    def __init__(self, xpos, ypos, width, height, label):
        
        # create OpenGL window
        fltk.Fl_Gl_Window.__init__(self, xpos, ypos, width, height, label)

        # iniit model value
        self.positionValue = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

        #initcamera value
        self.cameravalue = [1,0,0,1,1,1]

        # variable for cursor position
        self.cursorTransform = np.eye(4)
        self.cursorOffset = np.eye(4)
        self.pose = [0,0,0,0,0,0,0,0,0]
        
        # variable for model in opengl window class
        self.modelDicts = {'model':[],
                           'movepose':[],
                           'isModelInit':[],
                           'runModelIdx':0,
                           'modelNum':0}
        
        # init cursor model
        self.cursor = Model('cursor',drawFunc.point)
        
        # init grid model
        self.grid = Model("grid",drawFunc.Grid)
        
        # init origin model
        self.origin = Model("origin",drawFunc.point)
        
        # variable for change backdrop image
        self.nameNumber = 0
        self.testNumber = 0
        
        # read backdrop file
        self.openBackdropFile("backdropImg/backdrop_0.jpg")
        
        # iou score buffer
        self.iouScore = np.array([])
        
        # all flags parameter
        self.flags = {'snapMode':False,
                      'showModel':True,
                      'resetModelTransform':False,
                      'lineupTestMode':False,
                      'showModelWireframe':False,
                      'offsetMode':False,
                      'opacityMode':False}
        
        # logger parameter
        self.log = {
                        "testNumber":6,
                        "iou":None,
                        "avgIou":None,
                        "totalTime":None,
                        "modelPerSec":None
                    }
        
        #log file name
        self.logFileName = "./testLogStylus.csv"
        
        self.ctl = None
        self.addLog = False
        self.logfinish = False
    
    # open backdrop file
    def openBackdropFile(self, filename):
        
        # open backdrop as pillow image object
        self.backdropImageFile = Image.open( filename )
        
        # conver pillow image object to bytes
        self.imageObj  = self.backdropImageFile.tobytes("raw", "RGBX", 0, -1)
        
        # set texture id variable
        self.texid = None
        
        
        
    # setter function
    def readPose(self,pose):
        self.pose = pose
        
    def readScaleX(self,scaleX):
        self.scaleX = scaleX
        
    def readScaleY(self,scaleY):
        self.scaleY = scaleY
        
    def readScaleZ(self,scaleZ):
        self.scaleZ = scaleZ
    
    def readRotX(self,scaleX):
        self.rotX = scaleX
        
    def readRotY(self,scaleY):
        self.rotY = scaleY
        
    def readRotZ(self,scaleZ):
        self.rotZ = scaleZ




    # main opengl window callback 
    def draw(self):
        
        # add lighting to opengl window class
        self.lighting()
        
        # set background color to black
        GL.glClearColor(0.0, 0.0, 0.0, 0.0)
        
        # set depth to draw front object
        GL.glClearDepth(1.0) 
        GL.glDepthFunc(GL.GL_LESS)
        GL.glEnable(GL.GL_DEPTH_TEST)
        
        # clear color buffer
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        
        # draw backdrop to always show with no lighting
        GL.glDisable(GL.GL_DEPTH_TEST)
        GL.glDisable(GL.GL_LIGHTING)
        self.drawBackdrop()
        GL.glEnable(GL.GL_LIGHTING)
        GL.glEnable(GL.GL_DEPTH_TEST)


        # SET CAMERA VIEW
        
        # set matrix mode to projection matrix
        GL.glMatrixMode(GL.GL_PROJECTION)
        
       
        # reset matrix to identity
        GL.glLoadIdentity()

        # set new view port
        GL.glViewport(0,0,self.w(),self.h())

        # set area of screen
        # implement zoom and tramslate view
        # minX, maxX, minY, maxY, minZ, maxZ
        GL.glFrustum(((-1*(self.w()/self.h()))/self.cameravalue[0])-self.cameravalue[1], ((1*(self.w()/self.h()))/self.cameravalue[0])-self.cameravalue[1], -1/self.cameravalue[0]-self.cameravalue[2], 1/self.cameravalue[0]-self.cameravalue[2], 1, 100) 

        #offset camera view
        GL.glTranslatef(0,-5,0)
        GL.glTranslatef(0,0,-10)
        
        
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()
        
        # DRAW MODEL
        
        # if turn on snap mode
        # close cursor model
        self.cursor.show = not self.flags['snapMode']
        
        # draw model with position and rotation
        # self.cursor.drawModel(position=(self.pose[1],self.pose[2],self.pose[0]),rotation=(self.pose[5],self.pose[3],self.pose[4]),showFrame=not self.snapMode)
        
        # draw model with transform matrix
        self.cursor.moveModel(self.cursorTransform)
        self.cursor.show = self.flags['showModel']
        self.cursor.drawMatrixModel(showFrame=not self.flags['snapMode'])
        
        # draw grid
        # self.grid.drawMatrixModel(np.eye(4),showFrame=not self.flags['snapMode'])
        
        # draw origin
        # self.origin.drawModel(position=(self.pose[6],self.pose[7],self.pose[8]),showFrame=True)
        
        # draw model
        if self.modelDicts['modelNum'] > 0:
            
            # run to all model to initialize model
            for idx in range(self.modelDicts['modelNum']):
                
                # if model is not init
                if self.modelDicts['isModelInit'][idx] == 0:
                    
                    # init model
                    model = self.modelDicts['model'][idx]
                    model.initModel(matrixView = GL.glGetFloatv(GL.GL_MODELVIEW_MATRIX).T)
                    
                    # set init model flags to true
                    self.modelDicts['isModelInit'][idx] = 1
            
            # select model to draw ## because use for test mode ##
            self.modelDicts['runModelIdx'] = self.testNumber % 2
            model = self.modelDicts['model'][self.modelDicts['runModelIdx']]
            
            # set pose to move model
            ### now not use ###
            movePose = self.modelDicts['movepose'][self.modelDicts['runModelIdx']]
            
            # set show or not show model
            model.show = self.flags['showModel']
            
            # if model is selected
            if model.isSelected:
                
                # move model follow cursor
                # now use only newM because use transform matrix to draw model
                targetPosition,targetRotation,newM = model.followCursor(self.cursor)
            
            # if model is not selected
            else:
                
                # reset value from model
                model.cursorM = None
                model.cursorPose = None
                
                # if reset mode trigger
                if self.flags['resetModelTransform']:
                    
                    # set model position to home position ( identity )
                    model.currentM = np.eye(4)
                    
                    # turn off reset flags
                    self.flags['resetModelTransform'] = False
                
                # model position is remain position
                targetPosition,targetRotation,newM = model.centerPosition,model.rotation,model.currentM
            
            # draw model with position and rotation
            ### now not use ###
            # model.drawModel(position = targetPosition,rotation = targetRotation,showFrame=False)
            model.moveModel(newM)
            # draw model with transform matrix
            model.drawMatrixModel(showFrame=False,wireFrame = self.flags['showModelWireframe'], opacity = self.flags['opacityMode'])
                
                
                
    # handle function of opengl window class
    def handle(self,event):
        
        # get mouse position
        # print("ctl")
        self.ctl.readEvent(event)
        return fltk.Fl_Gl_Window.handle(self, event)
        
        
        # xMousePosition = fltk.Fl.event_x()
        # yMousePosition = fltk.Fl.event_y()
        
        # # if left mouse button is pressed
        # if event == fltk.FL_PUSH and fltk.Fl.event_button() == 1: 
        #     # print("mouse position:",xMousePosition,yMousePosition)
        #     return 1
        
        # check keyborad key event
        # if event == fltk. FL_KEYUP: #keyboard handle
        #     # print("key press : ",chr(fltk.Fl.event_key())) #check key #type:int 
        #      #check key #type:int 
            
        
                
        
                
        
                
        #     # # reset model position flags
        #     # if fltk.Fl.event_key() == ord('m'):
        #     #     self.flags['resetModelTransform'] = True
            
        #     # add user profile
        #     if fltk.Fl.event_key() == ord('l'):
        #         self.addLogProfile()
            # add user profile
            if fltk.Fl.event_key() == ord('l'):
                self.addLog = True
            
            # # start test mode
            # if fltk.Fl.event_key() == ord('p'):
            #     print("Enable test mode\nNumber of test: ",self.log['testNumber'])
            #     self.testMode(self.log['testNumber'])
            
        #     # calculate iou
        #     if fltk.Fl.event_key() == ord('d'):
                
        #         # create flags buffer remember current flags
        #         oldFlags = self.flags.copy()
                
        #         # set model to solid
        #         self.flags['showModelWireframe']=False
                
        #         # turn off opacity
        #         self.flags['showModel'] = True
                
        #         # calculate iou
        #         score = self.checkIoU()
                
        #         # update flags states from flags buffer
        #         self.flags=oldFlags
                
        #         # if in test mode
        #         if self.flags['lineupTestMode']:
                    
        #             # reset model position
        #             self.flags['resetModelTransform'] = True         
                    
        #             # add iou score to buffer
        #             self.iouScore = np.append(self.iouScore,score)
                    
        #             # check that test all model or not
        #             self.testMode(self.log['testNumber'])
                    
        #             # update test number
        #             self.testNumber += 1
                    
        #             # if still in test mode
        #             if self.flags['lineupTestMode']:
                        
        #                 # open next backdrop
        #                 self.openBackdropFile("backdropImg/backdrop_"+str(self.testNumber)+".jpg")
            
        #     # FOR CREATE BACKDROP IMAGE
            
        #     # turn on snap mode
        #     if fltk.Fl.event_key() == ord('s'):
        #         self.flags['snapMode'] = not self.flags['snapMode']
        #         self.redraw()
        #         print("set Snapmode",self.flags['snapMode'])
                
        #     # snap current opengl window image
        #     if fltk.Fl.event_key() == ord(' ') and self.flags['snapMode']:
                
        #         # set backdrop file name
        #         backdropName = "backdropImg/backdrop_" + str(self.nameNumber)
                
        #         # snap current opengl window image and save
        #         self.snap(backdropName)
                
        #         # update backdrop number name
        #         self.nameNumber = self.nameNumber + 1
                
        #     # STILL TESTING
        #     # offset mouse mode
        #     if fltk.Fl.event_key() == ord('c'):
        #         self.flags['offsetMode'] = not self.flags['offsetMode']
            
        #     # break condition to run main call back
        #     fltk.Fl_check()
        #     return 1
        
        # # if no event
        # else:
            
        #     # wait for handle
        #     return fltk.Fl_Gl_Window.handle(self, event)
                
                
                
    # add model to opengl window class
    def addModel(self,name,drawFunction=None,position=(0,0,0),rotation=(0,0,0),obj=None):
        
        # create model class
        model = Model(name,drawFunction,position,rotation,obj=obj)
        
        # add model to model dicts
        self.modelDicts['model'].append(model)
        
        # set move pose to position that create model
        self.modelDicts['movepose'].append([(position[0],position[1],position[2]),(rotation[0],rotation[1],rotation[2])])
        
        # set init model flags to false
        self.modelDicts['isModelInit'].append(0)
        
        # update model number
        self.modelDicts['modelNum'] = len(self.modelDicts['model'])
    
    # move model to specified position
    ### now not use ###
    def moveModel(self,name,position,rotation):
        for idx in range(self.modelDicts['modelNum']):
            model = self.modelDicts['model'][idx]
            if model.getName() == name:
                self.modelDicts['movepose'][idx][0] = 'opacityMode'(position[0],position[1],position[2])
                self.modelDicts['movepose'][idx][1] = (rotation[0],rotation[1],rotation[2])
     
    # test mode
    def testMode(self,numberOfBackdrop):
        
        # if test mode is not start
        if not self.flags['lineupTestMode']:
            
            # start timer
            self.startLineupTime = time.time()
            
            # turn test mode flags to true
            self.flags['lineupTestMode'] = True
        
        # if test all model
        if len(self.iouScore) == numberOfBackdrop:
            
            # reset backdrop number to start backdrop
            self.testNumber = 0  
            
            # stop timer
            self.stopLineupTime = time.time()
            
            # turn of test mode flags
            self.flags['lineupTestMode'] = False
            
            # add iou score to logger
            self.log["iou"] = self.iouScore
            
            # calculate average iou score and at to logger
            self.log["avgIou"] = np.sum(self.iouScore)/numberOfBackdrop
            
            # calculate totaltime to logger
            self.log["totalTime"] = self.stopLineupTime - self.startLineupTime
            
            # calculate time that use per 1 model
            self.log["modelPerSec"] = (self.stopLineupTime - self.startLineupTime)/5
            
            # show score
            print("\nIoU : ",self.log["iou"])
            print("average IoU: ",self.log["avgIou"])
            print("total time: ",self.log["totalTime"])
            print("ModelPerSec: ",self.log["modelPerSec"])
            
    # snap opengl window image
    def snap(self,name, save = True, binary = False):
        
        # read curren pixel element
        GL.glPixelStorei(GL.GL_PACK_ALIGNMENT, 1)

        xMin = (self.w()-self.h())/2
        xMax = self.h()
        yMin = 0
        yMax = self.h() 
        wSize = self.h()
        hSize = self. h()   
        
        # save pixel to variable as usinged bytes
        data = GL.glReadPixels(xMin, yMin, xMax, yMax, GL.GL_RGBA, GL.GL_UNSIGNED_BYTE)
        
        # convert bytes data to pillow image data
        image = Image.frombytes("RGBX", (wSize, hSize), data)
        
        # turn image upside down
        image = ImageOps.flip(image) 
        
        # set image file name
        imagename = name + ".jpg"
        
        # if save enable
        if save:
            
            print("snap!")
            print("save at :",imagename)
            
            # if binary mode enable
            if binary:
                
                # convert image from rgb to binary
                imagenp = np.asarray(image.convert('RGB')).copy()
                imagenp[imagenp> 128] = 200
                imagenp = Image.fromarray(imagenp.astype('uint8'), 'RGB')
                
                # save binary image
                imagenp.save(imagename, 'JPEG')
            
            # save image
            image.save(imagename, 'JPEG')
        
        # if not save 
        else:
            
            # change image to 8 bits RGB
            image = image.convert('L')
            
            # return gray scale image
            return image
    
    # calculate iou
    def checkIoU(self):
        
        # remember current flags to buffer
        oldflags = self.flags.copy()
        
        # turn on snap mode
        self.flags['snapMode'] = False
        
        # change model to solid
        self.flags['showModelWireframe'] = False
        
        # turn off opacity
        self.flags['opacityMode'] = False
        
        # set model to show
        self.flags['showModel'] = True
        
        # update opengl window
        self.redraw()
        fltk.Fl_check()
        
        # change backdrop image to binary
        fltk.Fl_wait(0.5)
        self.snap("backdrop",save=True)
        backdropImg = self.snap("backdrop",save=False)
        bw = np.asarray(backdropImg).copy()
        bw[bw < 80] = 0    # Black
        bw[bw >= 80] = 255 # White
        
        self.flags['snapMode'] = True
        self.flags['showModelWireframe'] = False
        self.flags['opacityMode'] = False
        self.flags['showModel'] = True
        self.redraw()
        fltk.Fl_check()
        fltk.Fl_wait(0.5)
        self.snap("test",save=True)

        # get 8 bits RGB of model image
        testimg = self.snap("test",save=False)
        
        # change image to binary
        testbw = np.asarray(testimg).copy()
        testbw[testbw < 80] = 0    # Black
        testbw[testbw >= 80] = 255 # White
        
        # check intersect pixel
        intersect = np.bitwise_and(bw,testbw)
        
        # check union pixel
        union = np.bitwise_or(bw,testbw)
        
        # calculate IoU
        iou = np.sum(intersect==255)/np.sum(union==255)
        print("IoU: ",iou)
        
        # set flags to remain flags
        self.flags = oldflags
        
        # update opengl window
        self.redraw()
        fltk.Fl_check()
        
        # return iou score
        return iou
    
    # create lighting
    def lighting(self):
        
        # enable light mode
        GL.glEnable(GL.GL_LIGHTING)
        GL.glEnable(GL.GL_LIGHT0)
        
        # set light color and ambient
        ambientLight = [0.2, 0.2, 0.2, 1.0 ] # RGB and alpha channels
        diffuseLight = [0.8, 0.8, 0.8, 1.0 ] # RGB and alpha channels
        GL.glLightfv(GL.GL_LIGHT0, GL.GL_AMBIENT, ambientLight)
        GL.glLightfv(GL.GL_LIGHT0, GL.GL_DIFFUSE, diffuseLight)
        GL.glEnable(GL.GL_COLOR_MATERIAL)
        GL.glColorMaterial(GL.GL_FRONT, GL.GL_AMBIENT_AND_DIFFUSE)
        
        # set light source position
        # x, y, z, alpha
        position = [ 100.0, 0.0, -1.0, 1.0 ]
        GL.glLightfv(GL.GL_LIGHT0, GL.GL_POSITION, position)

    # draw backdrop
    def drawBackdrop(self):
        
        # set camera view
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        GL.glOrtho( (-0.5*(self.w()/self.h())/self.cameravalue[0])-0.5*self.cameravalue[1], (0.5*(self.w()/self.h())/self.cameravalue[0])-0.5*self.cameravalue[1], -0.5/self.cameravalue[0]-0.5*self.cameravalue[2], 0.5/self.cameravalue[0]-0.5*self.cameravalue[2], -0.5, 1 )
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()

        # get width and height of backdropImage
        imageW = self.backdropImageFile.size[0]
        imageH = self.backdropImageFile.size[1]
        
        # if texture id is not set
        if( self.texid == None ):
            
            # bind texture
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
        
        # if not in snap mode
        if not self.flags['snapMode']:
            
            # read texture image
            GL.glBindTexture( GL.GL_TEXTURE_2D, self.texid )
            GL.glEnable( GL.GL_TEXTURE_2D )
            
            # create quads and map texture
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
    def getPositionFromSlider(self,num,value):
        self.positionValue[num] = value

    def getCameraFromSlider(self,num,value):
        self.cameravalue[num] = value
        self.redraw()