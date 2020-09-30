import fltk
import numpy as np
from scipy.spatial import ConvexHull
from OpenGL import GL, GLUT, GLU
from scipy.spatial.transform import Rotation as R
import math
class Handler():
    def __init__(self):
        self.push = False
        self.release = True
        self.buttonNum = None
        
        self.keyCha = None
        
class MainController(Handler):
    def __init__(self):
        
        super().__init__()
        
        self.controllerList = []
    def registerController(self,controller):
        self.controllerList.append(controller)
    
    def getController(self,idx=0):
        return self.controllerList[idx]
    def setWindowWidthHeight(self,width,height):
        for ctl in self.controllerList:
           ctl.windowWidth = width
           ctl.windowHeight = height
            
           
    def readEvent(self,event):
        for ctl in self.controllerList:
            
            
            if event == fltk.FL_KEYUP:
                # print(fltk.Fl.event_key())
                ctl.keyCha = fltk.Fl.event_key()
                
            ctl.runEvent(event)

             
class CommonController(Handler):
    
    def __init__(self,packData):
        super().__init__()
        self.flags = packData['flags']
        self.modelDicts = packData['modelDicts']
        self.log = packData['log']
        self.windowHeight = None
        self.windowWidth = None
        self.history = {'moveHistory':[np.eye(4)],
                        'moveHistoryPosition':0}
        
    def toggleFlags(self,flags):
        self.flags[flags] = not self.flags[flags]
        # return self.flags
        
    def undo(self):
        if self.history['moveHistoryPosition'] > 0:
            self.history['moveHistoryPosition'] -= 1
            model = self.modelDicts['model'][self.modelDicts['runModelIdx']]
            model.moveModel(self.history['moveHistory'][self.history['moveHistoryPosition']])
            
        else:
            print("nothing to undo")
            
    def redo(self):
        if self.history['moveHistoryPosition']+1 < len(self.history['moveHistory']):
            self.history['moveHistoryPosition'] += 1
            model = self.modelDicts['model'][self.modelDicts['runModelIdx']]
            model.moveModel(self.history['moveHistory'][self.history['moveHistoryPosition']])
            
        else:
            print("nothing to redo")
           
    def addHistory(self,history):
        self.history['moveHistoryPosition']+=1
        
        if len(self.history['moveHistory']) == self.history['moveHistoryPosition']:
            # This is a new event in hisory
            self.history['moveHistory'].append(history)
        else:
            # This occurs if there was one of more UNDOs and then a new
            # execute command happened. In case of UNDO, the history_position
            # changes, and executing new commands purges any history after
            # the current position"""
            self.history['moveHistory'] = self.history['moveHistory'][:self.history['moveHistoryPosition']+1]
            self.history['moveHistory'][self.history['moveHistoryPosition']] = history
    
    def clearHistory(self):
        self.history = {'moveHistory':[np.eye(4)],
                        'moveHistoryPosition':0}
    def runCommonEvent(self):
        # print(self.keyCha)
        
        if self.keyCha == ord('q'):
            # print("aa")
            self.toggleFlags('showModel')
            
        if self.keyCha == ord('1'):
            self.toggleFlags('showModelWireframe')
            
        if self.keyCha == ord('2'):
            self.toggleFlags('opacityMode')
        
        if self.keyCha == ord('s'):
            self.toggleFlags('snapMode')
            self.flags['showCursor'] = not self.flags['snapMode']
        
        if self.keyCha == ord('d'):
            self.toggleFlags('checkIoU')
            if self.flags['lineupTestMode']:
                self.clearHistory()
            
        if self.keyCha == ord('p'):
            self.toggleFlags('testMode')
            
        if self.keyCha == ord('m'):
            self.toggleFlags('resetModelTransform')

        if self.keyCha == ord('l'):
            self.flags['addLog'] = True
        
        if self.keyCha == ord('t'):
            self.undo()
        
        if self.keyCha == ord('y'):
            self.redo()
             
        self.keyCha = None
        return 1


class StylusController(CommonController):
    def __init__(self,packData):
        super().__init__(packData)
        # set config
        self.cfg={"homeCfg":(6.02190372, -0.06466488,  0.0184181)}
        self.transform = np.eye(4)
        self.cursor = packData['cursor']
        self.selectedModel = []
        self.hOpenGlToBase = np.array([[0,1,0,0],
                                  [0,0,1,0],
                                  [1,0,0,0],
                                  [0,0,0,1]])
    def runEvent(self,event):
        self.runCommonEvent()
        if event == 999: # move cursor
            self.cursor.moveModel(self.transform)
            
            model = self.modelDicts['model'][self.modelDicts['runModelIdx']]
            
            # if model is selected
            if model.isSelected:
                
                # move model follow cursor
                # now use only newM because use transform matrix to draw model
                newM = self.followCursor(model,self.cursor)
            
            # if model is not selected
            else:
                
                # reset value from model
                model.cursorM = None
                model.cursorPose = None
                
                # if reset mode trigger
                if self.flags['resetModelTransform']:
                    for m in self.modelDicts['model']:
                        
                        # set model position to home position ( identity )
                        m.currentM = np.eye(4)
                        
                        # turn off reset flags
                        self.flags['resetModelTransform'] = False
                    self.addHistory(m.currentM)
                    
                # model position is remain position
                newM = model.currentM
            model.moveModel(newM)
        
        if event >= 1000:
            key = event%1000
            if key == 1:
                self.push = True
                self.release = False
                self.buttonNum = 1
                print("left click!")
                self.selectedModel = self.selectModel()
                
                
            elif key == 2:
                self.push = True
                self.release = False
                self.buttonNum = 2
                print("releaseModel")
                model = self.modelDicts['model'][self.modelDicts['runModelIdx']]
                if model.isSelected:
                    self.addHistory(model.currentM)
                
                self.releaseModel()
            else:
                self.push = False
                self.release = True
                self.buttonNum = None
        
            
    def setTransform(self, cursorTransform):
        # read cursor transform from stylus and scaling
        cursorTransform[0:3,3] = cursorTransform[0:3,3].copy() * 0.05
        
        # offset home position
        cursorTransform[0,3] -= self.cfg["homeCfg"][0]
        cursorTransform[1,3] -= self.cfg["homeCfg"][1]
        cursorTransform[2,3] -= self.cfg["homeCfg"][2]
        cursorTransform = np.dot(self.hOpenGlToBase,cursorTransform)
        self.transform = cursorTransform.copy()
    
    def selectModel(self):
        
        
        # create selected model buffer
        selectModel = []
        model = self.modelDicts['model'][self.modelDicts['runModelIdx']]
        
        # run through all models in opengl window class
        # for model in self.modelDicts['model']:
            
            # if cursor position is in model
        if self.isPointInsideConvexHull(model,self.cursor.centerPosition):
            
            # model is selected
            model.isSelected = True
            
            # add model to selected model buffer
            selectModel.append(model)
        
        # return selected model buffer
        return selectModel
    
    # release model
    def releaseModel(self):
        
        # run through all models in opengl window class
        for model in self.modelDicts['model']:
            
            # change model to not selected
            model.isSelected = False
        self.selectedModel=[]
            
    # check is point inside model
    def isPointInsideConvexHull(self,model,point):
        
        # create convex hull of model with current vertices
        hull = ConvexHull(model.obj.current_vertices,incremental = True)
        
        # add point into hull and draw new convexHull
        newHull = ConvexHull(np.concatenate((hull.points, [point])))
        
        # if shape of hull and newHull is the same, then this point is inside model
        if np.array_equal(newHull.vertices, hull.vertices): 
            return True
        
        # if shape of newHull is not the same, then this point is outside model
        return False
    
    # move model follow cursor
    def followCursor(self,model,cursor):
        
        # if not init
        if model.cursorPose == None or type(model.cursorM) == type(None):
            
            # remember rotattion and position
            ### now not use ###
            model.cursorPose = cursor.rotation
            model.currentRotation = model.rotation
            
            # remember cursor transform matrix when first clicked
            model.cursorM = cursor.currentM.copy()
            
            # remember model transform matrix when first clicked
            model.startclickM = model.currentM.copy()
        
        # print(cursor.currentM)
        # calcualte delta TRANSFORM of cursor that first clicked and current cursor transform
        deltaTransform = np.dot(cursor.currentM,np.linalg.inv(model.cursorM))
        
        # calculate new model transformation
        newM = np.eye(4)
        newM = np.dot(deltaTransform,model.startclickM)
        
        
        # return new model transform
        return newM

class MouseController(CommonController):
    def __init__(self,packData):
        super().__init__(packData)
        self.flags['showCursor'] = False
        self.flags['showModelFrame'] = True
        self.transform = np.eye(4)
        self.selectedModel = []
        self.windowHeight = packData['height']
        self.windowWidth = packData['width']
        self.cameraValue = packData['camera']
        self.flags['mouseMode']= 'trans'
        
    def runEvent(self,event):
        
        # mouse position
        self.xMousePosition = fltk.Fl.event_x()
        self.yMousePosition = fltk.Fl.event_y()
        
        # select mode to move
        if self.keyCha == ord('w'):
            self.flags['mouseMode'] = 'trans'

        if self.keyCha == ord('z'):
            self.flags['mouseMode'] = 'rotZ'

        if self.keyCha == ord('x'):
            self.flags['mouseMode'] = 'rotX'

        if self.keyCha == ord('c'):
            self.flags['mouseMode'] = 'rotY'

        # check model selection when mouse click
        if event == fltk.FL_PUSH:
            self.lastPosX = self.xMousePosition
            self.lastPosY = self.yMousePosition
            # print("left click!")
            self.selectedModel = self.selectModel()

        # run when there is something selected   
        if self.selectedModel != []:
            model = self.modelDicts['model'][self.modelDicts['runModelIdx']]
            newM = model.currentM

            # move method when model is selected 
            if model.isSelected:

                # mousewheel event
                if event == fltk.FL_MOUSEWHEEL:
                    newM = self.mouseWheel()

                #mouse drag event
                if event == fltk.FL_DRAG:
                    newM = self.mouseDrag()

                # store model position after release mouse# model is selected
                    self.addHistory(newM)
                    # print(self.history)
            # move model with new matrix
            model.moveModel(newM)
            
        # run common controller event 
        self.runCommonEvent()
        
        # reset model
        if self.flags['resetModelTransform']:
            for model in self.modelDicts['model']:

                # set model position to home position ( identity )
                model.currentM = np.eye(4)
        
                # turn off reset flags
                self.flags['resetModelTransform'] = False

                # set new matrix model
                newM = model.currentM
                
                # move model to the new matrix model
                model.moveModel(newM)
            self.addHistory(newM)
            
        # every model will be deselected during checking Iou
        if self.flags['checkIoU']:
            for model in self.modelDicts['model']:
                model.isSelected = False

    # mouse wheel moving method
    def mouseWheel(self):
        model = self.modelDicts['model'][self.modelDicts['runModelIdx']]
        newM = model.currentM.copy()

        # translate in z axis
        if self.flags['mouseMode'] == 'trans':
            newM[2][3] = newM[2][3] + 0.1*(fltk.Fl.event_dy())
        
        # rotate model
        else:
            newM = self.rotationMatrixTransform(self.flags['mouseMode'],fltk.Fl.event_dy())
        return newM

    # mouse drag moving method
    def mouseDrag(self):

        # get the distance when mouse moving from one point to another point
        recentX = self.xMousePosition - self.lastPosX
        recentY = self.yMousePosition - self.lastPosY

        #store the current por=sition to calculate  distance for the next position
        self.lastPosX = self.xMousePosition
        self.lastPosY = self.yMousePosition 
        # print(self.xMousePosition,self.yMousePosition)

        model = self.modelDicts['model'][self.modelDicts['runModelIdx']]
        newM = model.currentM.copy()

        # calculate ratio from mouse to window
        ratioX = -(5*(self.windowWidth/10)/(newM[2][3] -10))
        ratioY = -(5*(self.windowHeight/10)/(newM[2][3] -10))
        
        # drag to translate
        if self.flags['mouseMode'] == 'trans':
            xTranslatePosition = recentX/ratioX + newM[0][3]
            yTranslatePosition = -(recentY/ratioY) + newM[1][3]
            newM[0][3] = xTranslatePosition
            newM[1][3] = yTranslatePosition
            
        # drag to rotate
        if self.flags['mouseMode'] != 'trans':
            # calculate displacement of mouse moving
            dx = recentX
            dy = -recentY
            degree = math.sqrt(dx*dx+dy*dy)

            # get new rotation matrix
            if self.flags['mouseMode']!= 'rotZ':
                if dx+dy>0:
                    newM =self.rotationMatrixTransform(self.flags['mouseMode'],degree)
                else:
                    newM = self.rotationMatrixTransform(self.flags['mouseMode'],-degree)
            else:
                if dx+dy<0:
                    newM = self.rotationMatrixTransform(self.flags['mouseMode'],degree)
                else:
                    newM = self.rotationMatrixTransform(self.flags['mouseMode'],-degree)    

        return newM

    # calculate rotation matrix to rotate around current local axis
    def rotationMatrixTransform(self,rotationAxis,deg):
        model = self.modelDicts['model'][self.modelDicts['runModelIdx']]
        newM = model.currentM.copy()

        # decrease degree size
        degree = deg*0.005

        # calculate rotation matrix
        degreeXMatrix = np.asarray([[1,0,0],[0,math.cos(degree),-math.sin(degree)],[0,math.sin(degree),math.cos(degree)]])
        degreeYMatrix = np.asarray([[math.cos(degree),0,math.sin(degree)],[0,1,0],[-math.sin(degree),0,math.cos(degree)]])
        degreeZMatrix = np.asarray([[math.cos(degree),-math.sin(degree),0],[math.sin(degree),math.cos(degree),0],[0,0,1]])

        # rotate condition
        if rotationAxis == 'rotX':
            newM[0:3,0:3] = np.dot(newM[0:3,0:3],degreeXMatrix)
        elif rotationAxis == 'rotY':
            newM[0:3,0:3] = np.dot(newM[0:3,0:3],degreeYMatrix)
        elif rotationAxis == 'rotZ':
            newM[0:3,0:3] = np.dot(newM[0:3,0:3],degreeZMatrix)

        return newM

    # check model selection
    def selectModel(self):
        # create selected model buffer
        selectModel = []
        model = self.modelDicts['model'][self.modelDicts['runModelIdx']]

        # check selection
        if self.mouseSelectedCheck() == model.modelId or self.mouseSelectedCheck() == True:

            # model is selected
            model.isSelected = True

            #add to selected model buffer
            selectModel.append(model)
        else:

            #model is deselected
            model.isSelected = False

        return selectModel

    # mouse click selection checking
    def mouseSelectedCheck(self):

        # read pixel color and depth
        self.depth = GL.glReadPixels(self.xMousePosition,self.windowHeight - self.yMousePosition, 1,1,GL.GL_DEPTH_COMPONENT,GL.GL_UNSIGNED_BYTE)
        self.color = GL.glReadPixels(self.xMousePosition,self.windowHeight - self.yMousePosition, 1,1,GL.GL_RGBA,GL.GL_UNSIGNED_BYTE)
        
        # color checking
        # if it's black mouse doesn't select anything
        if self.color[0] == 0 and self.color[1] == 0 and self.color[2] == 0:
            mouseSelected = False
        else:
            mouseSelected = True

        model = self.modelDicts['model'][self.modelDicts['runModelIdx']]

        # SET CAMERA VIEW
        vp = GL.glGetIntegerv(GL.GL_VIEWPORT)
        
        # set matrix mode to projection matrix
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()

        # pick position
        GLU.gluPickMatrix(self.xMousePosition, vp[3] - self.yMousePosition, 1, 1, vp)

        # set viewport for select mode
        GL.glFrustum(((-1*(self.windowWidth/self.windowHeight))/self.cameraValue[0])-self.cameraValue[1], ((1*(self.windowWidth/self.windowHeight))/self.cameraValue[0])-self.cameraValue[1], -1/self.cameraValue[0]-self.cameraValue[2], 1/self.cameraValue[0]-self.cameraValue[2], 1, 100)
        GL.glTranslatef(0,-5,0)
        GL.glTranslatef(0,0,-10)

        # set buffer size
        sel = GL.glSelectBuffer(100)

        # rebder mode
        GL.glRenderMode(GL.GL_SELECT)

        # select buffer initial
        GL.glInitNames()
        GL.glPushName(0)

        # model view mode
        GL.glMatrixMode(GL.GL_MODELVIEW)

        # draw model in select mode
        model.drawMatrixModel(selectedMode = True)

        # return model id
        hits = GL.glRenderMode(GL.GL_RENDER)

        # if mouse select something
        if hits != []:
            modelselected = hits[0].names[0]
        else:
            modelselected = 0

        return mouseSelected

class StylusController2(StylusController):
    def __init__(self,packData):
        super().__init__(packData)
        self.hBaseToWorld = np.eye(4)
        self.isImuInit = False
        self.imuTrans = np.eye(4)
    def runEvent(self,event):
        if self.keyCha == ord('k'):
            self.isImuInit = True
            
        
        super().runEvent(event)
        
    def setTransform(self, cursorTransform,imuQuat):
        # read cursor transform from stylus and scaling
        cursorTransform[0:3,3] = cursorTransform[0:3,3].copy() * 0.05
       
        # offset home position
        cursorTransform[0,3] -= self.cfg["homeCfg"][0]
        cursorTransform[1,3] -= self.cfg["homeCfg"][1]
        cursorTransform[2,3] -= self.cfg["homeCfg"][2]
        self.readImuQuat(imuQuat)
        cursorTransform[0:3,0:3] = self.imuTrans[0:3,0:3].copy()
        cursorTransform = np.dot(self.hOpenGlToBase,cursorTransform)
        
        self.transform = cursorTransform.copy()
    
    def initImu(self,quat):
        h = np.eye(4)
        r = R.from_quat(quat)
        h[0:3,0:3] = r.as_matrix()
        self.hBaseToWorld = np.linalg.inv(h.copy())
    def readImuQuat(self,quat):
        if not self.isImuInit:
            self.initImu(quat)
        else:
            
            hWorldToImu = np.eye(4)
            r = R.from_quat(quat)
            hWorldToImu[0:3,0:3] = r.as_matrix()
            self.imuTrans = np.dot(self.hBaseToWorld,hWorldToImu)