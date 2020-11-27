import fltk
import numpy as np
from scipy.spatial import ConvexHull
from OpenGL import GL, GLUT, GLU
from scipy.spatial.transform import Rotation as R
import math
from Ray import Ray,Plane,Line
import sys
import time
class Handler():
    def __init__(self):
        self.push = False
        self.release = True
        self.buttonNum = None
        
        self.keyCha = None
        self.keyHold = None
        
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
           
            if not fltk.Fl.event_alt() or(fltk.Fl.event_alt() and  (fltk.Fl.event_key() == 65513)):
                ctl.resetKey()
                ctl.keyHold = None
            elif event == fltk.FL_SHORTCUT and fltk.Fl.event_alt():
                ctl.keyHold = fltk.Fl.event_key()
            
            if event == fltk.FL_KEYUP:
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
                        'moveHistoryPosition':0,
                        'modelName':['']}
        self.windowHeight = packData['height']
        self.windowWidth = packData['width']
        self.cameraValue = packData['camera']
        self.fineRot = False
        self.fineTran = False
        self.modelMode = packData['modelMode']
        self.ikMode = packData['ikMode']
    
    def updateModelPose(self,model,transform,artiModel,mode = 'fk'):
        if self.modelMode =='fk':
            
            if type(model).__name__ == "Joint":
                transform[0:3,3] = model.currentM[0:3,3]
            model.moveModel(transform)
            
        elif self.modelMode =='ik':
            
            if type(model).__name__ != "Joint":
                
                # # move transform
                # if model.name != 'base':
                #     transform[0:3,0:3] = np.eye(3)
                # else:
                #     pass
                ot = transform.copy()
                
                model.moveModel(transform,mode='absolute')
                
                try:
                    # rotate J2 back to align with J1
                    invJ2 = np.eye(4)
                    invJ2[0:3,0:3] = np.linalg.inv(artiModel.listOfJoint[1].parentToLocal[0:3,0:3])
                    artiModel.listOfJoint[1].moveModel(invJ2,mode='relative')
                    
                    # calculate theta 2
                    from scipy.spatial import distance
                    distJ1ToJ2 = artiModel.listOfJoint[1].getDist
                    distJ2ToJ3 = artiModel.listOfJoint[2].getDist
                    distJ1ToTarget = distance.euclidean(artiModel.listOfJoint[0].worldToLocal[0:3,3],artiModel.target.worldToLocal[0:3,3])
                
                
                    
                    # if cannot reach
                    if distJ1ToTarget > distJ1ToJ2+distJ2ToJ3:
                        theta2 = 0

                    else:
                        
                        # get theta2
                        cthetha2 = -(distJ1ToJ2**2+distJ2ToJ3**2-distJ1ToTarget**2)/(2*distJ1ToJ2*distJ2ToJ3)
                        theta2 = math.acos(cthetha2)
                        
                        # move J2 with theta2
                        c22 = np.array([[math.cos(theta2),-math.sin(theta2),0,0],
                                        [math.sin(theta2),math.cos(theta2),0,0],
                                        [0,0,1,0],
                                        [0,0,0,1]])
                        j2 = artiModel.listOfJoint[1].worldToLocal.copy()
                        artiModel.listOfJoint[1].moveModel(c22,mode='relative')
                except:
                    pass   
                
                # get vector wrt. J1
                v1Tov3 = np.dot(np.linalg.inv( artiModel.listOfJoint[0].worldToLocal),np.append(artiModel.listOfJoint[2].worldToLocal[0:3,3],[1]))[0:3]
                v1ToTarget = np.dot(np.linalg.inv( artiModel.listOfJoint[0].worldToLocal),np.append(artiModel.target.worldToLocal[0:3,3],[1]))[0:3]
                
                # cal angle between J1J3 and J1Target
                v1 = v1Tov3/np.linalg.norm(v1Tov3)
                v2 = v1ToTarget/np.linalg.norm(v1ToTarget)
               
                r = self.getVectorAngle(v1,v2)
                
                rotation = R.from_matrix(r)
                
                # rotate J1 to map J3 on target  
                tt = np.eye(4)
                tt[0:3,0:3] = r.copy()
                j1 = artiModel.listOfJoint[0].worldToLocal   
                artiModel.listOfJoint[0].moveModel(tt,mode="relative")
                
                # get vector wrt. J1
                v1Tov2 = np.dot(np.linalg.inv( artiModel.listOfJoint[0].worldToLocal),np.append(artiModel.listOfJoint[1].worldToLocal[0:3,3],[1]))[0:3]
                v1ToTarget = np.dot(np.linalg.inv( artiModel.listOfJoint[0].worldToLocal),np.append(artiModel.target.worldToLocal[0:3,3],[1]))[0:3]
                v1ToPole = np.dot(np.linalg.inv( artiModel.listOfJoint[0].worldToLocal),np.append(artiModel.poleVertex.worldToLocal[0:3,3],[1]))[0:3]
                
                # cal nomal vector of two plane
                nvJ1J2 = np.cross(v1ToTarget,v1Tov2)/np.linalg.norm(np.cross(v1ToTarget,v1Tov2))
                nvJ1Pole = np.cross(v1ToTarget,v1ToPole)/ np.linalg.norm(np.cross(v1ToTarget,v1ToPole))
                
                # cal angle between two plane
                planeAngle = self.getVectorAngle(nvJ1J2,nvJ1Pole)
                
                if not np.isnan(planeAngle).any():
                        
                    # rotate J2 plane to poleVertex plane
                    tt = np.eye(4)
                    tt[0:3,0:3] = planeAngle.copy()  
                    
                    
                    artiModel.listOfJoint[0].moveModel(tt,mode="relative")
                
                if self.ikMode == 'global':
                    newGbOrientationEF = np.eye(4)
                    gbOrientationEF = np.dot(np.linalg.inv(artiModel.listOfJoint[1].worldToLocal[0:3,0:3]),artiModel.target.worldToLocal[0:3,0:3])
                    newGbOrientationEF[0:3,0:3] = artiModel.target.worldToLocal[0:3,0:3].copy()
                    newGbOrientationEF[0:3,3] = artiModel.listOfJoint[2].worldToLocal[0:3,3].copy()
                    artiModel.listOfJoint[2].moveModel(newGbOrientationEF,mode="absolute")
            
    def getVectorAngle(self,v1,v2):
        if not np.equal(v1,v2).all():
            if np.linalg.norm(v1) == 0 or np.linalg.norm(v2) == 0:
                print( "one of the vectors is 0")
            v = np.cross(v1,v2)
            s = np.linalg.norm(v)
            c = np.dot(v1,v2)
            clamped_c = max(min(c, 1), -1)
            angle_c = math.acos( clamped_c )
            
            if angle_c >0.000001 :
                skV = np.array([[0,-v[2],v[1]],
                                [v[2],0,-v[0]],
                                [-v[1],v[0],0]])
                r = np.eye(3)+skV+np.dot(skV,skV)*((1-c)/(1-c**2))
                
            else :
                
                r= np.eye(3)
            
            return r
        else:
            r = np.eye(3)
            return r
        
    def toggleFlags(self,flags):
        self.flags[flags] = not self.flags[flags]
    
    def toggleModelFlags(self,flags):
        artiModel = self.modelDicts['model'][self.modelDicts['runModelIdx']]
        modelList = artiModel.getSubModel()
        # for model in modelList:
        #     # if model.isSelected:
        #     newFlags = not model.flags[flags]
        #     model.updateFlags(flags,newFlags)
        newFlags = not artiModel.flags[flags]
        artiModel.updateFlags(flags,newFlags)    
        
    def undo(self):
        
        if self.history['moveHistoryPosition'] > 0:
            modelPose = self.history['moveHistoryPosition'] 
            self.history['moveHistoryPosition'] -= 1
            
            
            artiModel = self.modelDicts['model'][self.modelDicts['runModelIdx']]
            modelList = artiModel.getSubModel()
            
            
            for model in modelList:
                
                if model.name == self.history['modelName'][modelPose]:
                    
                    newM = self.history['moveHistory'][modelPose][0]
                    self.updateModelPose(model,newM,artiModel)
                
        else:
            print("nothing to undo")
            
    def redo(self):
        
        if self.history['moveHistoryPosition']+1 < len(self.history['moveHistory']):
            self.history['moveHistoryPosition'] += 1
            modelPose = self.history['moveHistoryPosition']
            
            artiModel = self.modelDicts['model'][self.modelDicts['runModelIdx']]
            modelList = artiModel.getSubModel()
            for model in modelList:
                if model.name == self.history['modelName'][modelPose]:
                    
                    newM = self.history['moveHistory'][modelPose][1]
                    self.updateModelPose(model,newM,artiModel)
            
        else:
            print("nothing to redo")
           
    def addHistory(self,old,new,model):
        
        history = [old,new]
        # update history position
        self.history['moveHistoryPosition']+=1
        # if history is uptodate
        if len(self.history['moveHistory']) == self.history['moveHistoryPosition']:
            # This is a new event in hisory
            self.history['moveHistory'].append(history)
            self.history['modelName'].append( model.name)
            
        else:
            # This occurs if there was one of more UNDOs and then a new
            # execute command happened. In case of UNDO, the history_position
            # changes, and executing new commands purges any history after
            # the current position
            
            self.history['moveHistory'] = self.history['moveHistory'][:self.history['moveHistoryPosition']+1]
            self.history['moveHistory'][self.history['moveHistoryPosition']] = history
            self.history['modelName'] = self.history['modelName'][:self.history['moveHistoryPosition']+1]
            self.history['modelName'][self.history['moveHistoryPosition']] = model.name
        
        
    def resetKey(self):
        self.fineRot = False
        self.fineTran = False
    def clearHistory(self):
        self.history = {'moveHistory':[np.eye(4)],
                        'moveHistoryPosition':0,
                        'modelName':['']}
    
    def selectObjectWithBuffer(self,xPos,yPos):
        model = self.modelDicts['model'][self.modelDicts['runModelIdx']]

        # SET CAMERA VIEW
        vp = GL.glGetIntegerv(GL.GL_VIEWPORT)
        
        # set matrix mode to projection matrix
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()

        # pick position
        GLU.gluPickMatrix(xPos, vp[3] - yPos, 1, 1, vp)

        # set viewport for select mode
        # GLU.gluPerspective(30,self.windowWidth/self.windowHeight,1,100)
        
        
        ratio = 0.2679491924311227
        # ratio = 0.5773502691896257

        
        GL.glFrustum(((-ratio*(self.windowWidth/self.windowHeight))/self.cameraValue[0])-ratio*self.cameraValue[1], 
                    ((ratio*(self.windowWidth/self.windowHeight))/self.cameraValue[0])-ratio*self.cameraValue[1], 
                    -ratio/self.cameraValue[0]-ratio*self.cameraValue[2], 
                    ratio/self.cameraValue[0]-ratio*self.cameraValue[2],
                     1, 100)
        
        GL.glTranslatef(0,0,-35)

        #offset camera view
        GL.glTranslatef(0,-5,0)

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
        model.drawMatrixModel(selectedMode = True,mode = self.flags['mouseMode'],coordinate = self.flags['coordinate'],camera = self.cameraValue[0:3])

        # return model id
        hits = GL.glRenderMode(GL.GL_RENDER)
        def ele(elem):
            return elem.near
        # if mouse select something
        if hits != []:
            if len(hits) > 1:
                modelselected = min(hits,key=ele).names[0]
            else:
                modelselected = hits[0].names[0]
            
                
                
        else:
            modelselected = 0
        
        # return selectedModel id
        return modelselected
    
    def runCommonEvent(self):

        if self.keyCha == ord('q'):

            self.toggleModelFlags('showModel')
            
        if self.keyCha == ord('1'):
            self.toggleModelFlags('showModelWireframe')
            
        if self.keyCha == ord('2'):
            self.toggleModelFlags('opacityMode')
        
        # if self.keyCha == ord('s'):
        #     self.toggleFlags('snapMode')
        
        
            
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
            
        # select mode to move
        if self.keyCha == ord('w'):
            self.flags['mouseMode'] = 'trans'

        if self.keyCha == ord('e'):
            self.flags['mouseMode'] = 'rot'

        if self.keyCha == ord('r'):
            self.flags['coordinate'] = not self.flags['coordinate']
        
        if self.keyHold == ord('e'):
            
            self.fineRot = True
        if self.keyHold == ord('w'):
            
            self.fineTran = True
        
        
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
        self.cursorSpeed = self.flags['cursorSpeed']
        self.speed = 0.1
        self.lastClick = 0
        self.stillClick = False
        self.flags['showModelFrame'] = False
        
    def runEvent(self,event):
        
        # Run common event
        self.runCommonEvent()
        # Check specific event for stylus
        
        self.flags['showCursor'] = not self.flags['snapMode']

        if event == 999: # move cursor
            self.cursor.moveModel(self.transform)
            
            # model = self.modelDicts['model'][self.modelDicts['runModelIdx']]
            artiModel = self.modelDicts['model'][self.modelDicts['runModelIdx']]
            modelList = artiModel.getSubModel()
            # if self.selectedModel != []:
            for model in modelList:
                
                # model = self.selectedModel[0]
                # artiModel = self.modelDicts['model'][self.modelDicts['runModelIdx']]
                # modelList = artiModel.getSubModel()
                # for model in modelList:
                    # if model is selected
                if model.isSelected:
                    speed = 1

                    # move model follow cursor
                    # now use only newM because use transform matrix to draw model
                    newM = self.followCursor(model,self.cursor)
                    self.updateModelPose(model,newM,artiModel)
                    
                        
                
                # if model is not selected
                else:
                    if self.flags['resetModelTransform']:
                        artiModel = self.modelDicts['model'][self.modelDicts['runModelIdx']]
                        modelList =artiModel.getSubModel()
                        
                        # newM = model.currentM.copy()
                        for model in modelList:

                            # set model position to home position ( identity )
                            # model.currentM = np.eye(4)
                    
                            # turn off reset flags
                            self.flags['resetModelTransform'] = False

                            # set new matrix model
                            newM = model.startWorldToLocal
                            # move model to the new matrix model
                            self.updateModelPose(model,newM,artiModel)
                    model.cursorM = None
                    model.cursorPose = None
                    newM = model.currentM   
    
        if event >= 1000: # check button status
            
            # Get button key
            key = event%1000
            
            # LEFT CLICK
            if key == 1:
                self.push = True
                self.release = False
                self.buttonNum = 1
                
                self.leftClick()
                
            # RIGHT CLICK
            elif key == 2:
                self.push = True
                self.release = False
                self.buttonNum = 2
                # self.rightClick()
                
            
            # REALEASE ALL BUTTON
            else:
                self.push = False
                self.release = True
                self.buttonNum = None
                self.stillClick = False
                
    def leftClick(self):
        # Check select model
        newClick = time.time()
        deltaTime = newClick-self.lastClick    
            
        if self.selectedModel == [] and self.stillClick == False:
            
            self.selectedModel = self.selectModel(mode = "buffer")
            if self.selectedModel != []:
                self.old=self.selectedModel[0].currentM.copy()
            self.lastClick = newClick
            self.stillClick = True
            print("left click!")
            
        elif self.selectedModel != [] and self.stillClick == False:
            self.rightClick()
            self.stillClick = True
        
    def rightClick(self):
        # Check select model
        # add History log
        model = self.modelDicts['model'][self.modelDicts['runModelIdx']]
        artiModel = self.modelDicts['model'][self.modelDicts['runModelIdx']]
        modelList = artiModel.getSubModel()
        # for model in modelList:
        #     if model.isSelected:
        if self.selectedModel != []:
            
            self.addHistory(self.old,self.selectedModel[0].currentM,self.selectedModel[0])
                
            # Release model
        self.releaseModel()
    def setTransform(self, cursorTransform):
        # read cursor transform from stylus and scaling
        cursorTransform[0:3,3] = cursorTransform[0:3,3].copy() * 0.05
        
        # offset home position
        cursorTransform[0,3] -= self.cfg["homeCfg"][0]
        cursorTransform[1,3] -= self.cfg["homeCfg"][1]
        cursorTransform[2,3] -= self.cfg["homeCfg"][2]
        cursorTransform = np.dot(self.hOpenGlToBase,cursorTransform)
        self.transform = cursorTransform.copy()
    
    def selectModel(self,mode = "convex"):
        
        # create selected model buffer
        selectModel = []
        # model = self.modelDicts['model'][self.modelDicts['runModelIdx']]
        artiModel = self.modelDicts['model'][self.modelDicts['runModelIdx']]
        modelList = artiModel.getSubModel()
        # run through all models in opengl window class
        # for model in self.modelDicts['model']:
        if mode == "convex":
            for model in modelList:
                if self.isPointInsideConvexHull(model,self.cursor.centerPosition):
                    
                    # model is selected
                    model.isSelected = True
                    
                    # add model to selected model buffer
                    selectModel.append(model)
        elif mode == "buffer":
            cX,cY =self.getCursor2DPos()
            modelId = self.selectObjectWithBuffer(cX,cY)
            for model in modelList:
                
                
                if modelId == model.modelId:
                    # model is selected
                    model.isSelected = True
                    
                    # add model to selected model buffer
                    selectModel.append(model)
            # if cursor position is in model
        
            
        
        # return selected model buffer
        return selectModel
    
    # release model
    def releaseModel(self):
        print("releaseModel")
        
        artiModel = self.modelDicts['model'][self.modelDicts['runModelIdx']]
        modelList = artiModel.getSubModel()
        # run through all models in opengl window class
        for model in modelList:
            
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
    def followCursor(self,model,cursor,origin = "centermodel"):
        
        # move object with cursor is origin
        if origin == "cursor":
            
            # if not init
            if type(model.cursorM) == type(None):
                
                # remember cursor transform matrix when first clicked
                model.cursorM = cursor.currentM.copy()
                
                # remember model transform matrix when first clicked
                model.startclickM = model.currentM.copy()
            
            # calcualte delta TRANSFORM of cursor that first clicked and current cursor transform
            deltaTransform = np.dot(cursor.currentM,np.linalg.inv(model.cursorM))
            
            # calculate new model transformation
            newM = np.eye(4)
            newM = np.dot(deltaTransform,model.startclickM)
        
        # move object with center model is origin
        elif origin == "centermodel":
            
            # if not init
            if type(model.cursorM) == type(None) or ((self.fineRot or self.fineTran) and self.cursorSpeed != self.speed) or ((not (self.fineRot or self.fineTran)) and self.cursorSpeed != 0.5):
                
                # remember cursor transform matrix when first clicked
                model.cursorM = cursor.currentM.copy()
                
                # remember model transform matrix when first clicked
                model.startclickM = model.currentM.copy()
                
                # remember offset from model currenM
                model.offsetTran = np.eye(4)
                model.offsetTran[0,3] = -model.startclickM[0,3]
                model.offsetTran[1,3] = -model.startclickM[1,3]
                model.offsetTran[2,3] = -model.startclickM[2,3]
                model.invOffsetTran = np.eye(4)
                model.invOffsetTran[0,3] = model.startclickM[0,3]
                model.invOffsetTran[1,3] = model.startclickM[1,3]
                model.invOffsetTran[2,3] = model.startclickM[2,3]

            ## hNewM = hInvCurrentMOffsetTrans * deltaCntoC0 * hInvCurrentMOffsetTrans * hCurrentM
            ## newM  = model.invOffsetTran * tranCnC0 * rotCnC0 * model.offsetTran * model.startclickM
            ## newM  = model.invOffsetTran * tranCn * inv(tranC0) * rotCn * inv(rotC0) * model.offsetTran * model.startclickM
            if self.fineRot :
                
                self.cursorSpeed = self.speed
            elif self.fineTran:
                self.cursorSpeed = self.speed
                
            else:
                self.cursorSpeed = 0.5
            # Split rot and trans from hC0 (model.cursorM)
            rotc0 = np.eye(4)
            rotc0[0:3,0:3] = model.cursorM[0:3,0:3].copy()
            tranc0 = np.eye(4)
            tranc0[0:3,3] = model.cursorM[0:3,3].copy()
            
            # Split rot and trans from hCn (cursor.currentM)
            rotcn = np.eye(4)
            rotcn[0:3,0:3] = cursor.currentM[0:3,0:3].copy()
            trancn = np.eye(4)
            trancn[0:3,3] = cursor.currentM[0:3,3].copy()
            
            # Find hCnC0 wuth split inverse
            rotC0CnM = np.dot(rotcn,np.linalg.inv(rotc0))
            tranC0CnM = np.dot(trancn,np.linalg.inv(tranc0))
            
            
            rotvec = R.from_matrix(rotC0CnM[0:3,0:3])
            angle = rotvec.magnitude()
            
            if angle ==0:
                angle = 0.00000000000000000001
            rotvecOld = rotvec.as_rotvec()/angle
            
            rotvecNew = R.from_rotvec(angle*self.cursorSpeed*rotvecOld)
            rotC0CnM[0:3,0:3] = rotvecNew.as_matrix()
            
            tranC0CnM[0,3] = tranC0CnM[0,3].copy() * self.cursorSpeed
            tranC0CnM[1,3] = tranC0CnM[1,3].copy() * self.cursorSpeed
            tranC0CnM[2,3] = tranC0CnM[2,3].copy() * self.cursorSpeed
            
            
            
            hCnC0M = np.dot(tranC0CnM,rotC0CnM)
            
            # Cal NewM
            newM = np.dot(model.invOffsetTran,hCnC0M)
            newM = np.dot(newM,model.offsetTran)
            newM = np.dot(newM,model.startclickM)
            
            if self.fineRot:
                newM[0,3] = model.startclickM[0,3]
                newM[1,3] = model.startclickM[1,3]
                newM[2,3] = model.startclickM[2,3]
            elif self.fineTran:
                
                newM[0:3,0:3] = model.startclickM[0:3,0:3]
                # sys.exit()
            
                
        # return new model transform
        return newM
    
    # Cal stylus cursor 3d coordinate to 2d screen coordinate
    def getCursor2DPos(self):
        
        ### OpenGl matrix 
        ### Vproj = Mproj * Mmodelview * VscaleCam * Vmodel
        ### Vproj = Vproj/VscaleCam
        ### Vscreen = Vstartviewport + Vendviewport * (Vprojx,y + 1)/2
        
        # Get projection matrix
        proj = GL.glGetFloatv(GL.GL_PROJECTION_MATRIX).T
        
        # Get Vproj
        t = np.dot(proj,self.cursor.currentM)
        t = np.dot(t,np.array([[0,0,0,10]]).T)
        newT = t/t[3,0]

        # Get Vscreen
        vp = GL.glGetIntegerv(GL.GL_VIEWPORT)
        wx = vp[0]+vp[2]*(newT[0]+1)/2
        wy = vp[1]+vp[3]*(newT[1]+1)/2
        
        # return screen coordinates
        return wx,vp[3]-wy

class MouseController(CommonController):
    def __init__(self,packData):
        super().__init__(packData)
        self.flags['showCursor'] = False
        self.flags['showModelFrame'] = True
        self.selectedModel = []
        self.flags['mouseMode']= 'trans'
        self.flags['coordinate'] = True
        self.rotationAxis = None
        self.translationAxis = None

    def runEvent(self,event):
        
        # mouse position
        self.xMousePosition = fltk.Fl.event_x()
        self.yMousePosition = fltk.Fl.event_y()
        
        # check model selection when mouse click
        if event == fltk.FL_PUSH:
            self.selectedModel = self.selectModel()
            self.oldRay = Ray(self.xMousePosition,self.yMousePosition)
            if self.selectedModel != []:
                self.old=self.selectedModel[0].currentM.copy()
  

        # run when there is something selected   
        if self.selectedModel != []:
            artiModel = self.modelDicts['model'][self.modelDicts['runModelIdx']]
            model =self.selectedModel[0]
            
            newM = model.currentM.copy()

            # move method when model is selected 
            if model.isSelected:
                
                #mouse drag event
                if event == fltk.FL_DRAG:
                    newM = self.mouseDrag()
                    
                # store model position after release mouse# model is selected
                if event == fltk.FL_RELEASE:
                    self.addHistory(self.old,newM,model)
                 
            # move model with new matrix
            self.updateModelPose(model,newM,artiModel)
            
        # run common controller event 
        self.runCommonEvent()
        self.flags['showModelFrame'] = not self.flags['snapMode']

        # reset model
        if self.flags['resetModelTransform']:
            artiModel = self.modelDicts['model'][self.modelDicts['runModelIdx']]
            modelList =artiModel.getSubModel()
            
            # newM = model.currentM.copy()
            for model in modelList:

                # set model position to home position ( identity )
                # model.currentM = np.eye(4)
        
                # turn off reset flags
                self.flags['resetModelTransform'] = False

                # set new matrix model
                newM = model.startWorldToLocal
                

                # move model to the new matrix model
                self.updateModelPose(model,newM,artiModel)
            
        # every model will be deselected during checking Iou
        if self.flags['checkIoU']:
            for model in self.modelDicts['model']:
                model.isSelected = False

    # mouse drag moving method
    def mouseDrag(self):

        model = self.modelDicts['model'][self.modelDicts['runModelIdx']]
        model =self.selectedModel[0]
        newTranslate = np.eye(4)
        newM = model.worldToLocal.copy()
        
        # drag to translate
        if self.flags['mouseMode'] == 'trans':
            if self.translationAxis == 'None':
                recent = self.mouseTranslate(self.xMousePosition,self.yMousePosition,self.translationAxis,"worldxy")
                
            else:
                
                recent = self.mouseTranslate(self.xMousePosition,self.yMousePosition,self.translationAxis,self.flags['coordinate'])
            
            
            newTranslate[0:3,3] = recent
            newM = np.dot(newTranslate,newM)
            
        # drag to rotate
        elif self.flags['mouseMode'] == 'rot':
            # if self.rotationAxis != 'None':
            newM = self.mouseRotate(self.xMousePosition,self.yMousePosition,self.rotationAxis,self.flags['coordinate'])

        # return homogenous matrix
        return newM

    # check model selection
    def selectModel(self):
        # create selected model buffer
        selectModel = []
        artiModel = self.modelDicts['model'][self.modelDicts['runModelIdx']]
        modelList = artiModel.getSubModel()
        
        # self.rotationAxis = None
        # check selection   
        mouseSelectedCheck = self.mouseSelectedCheck()
        
        for model in modelList:
            
            if mouseSelectedCheck == model.modelId or mouseSelectedCheck == True:

                # for m in modelList:
                if len(self.selectedModel) >0:
                    self.selectedModel[0].isSelected = False
                # model is selected
                model.isSelected = True
                self.rotationAxis = 'None'
                self.translationAxis = 'None'

                #add to selected model buffer
                if len(selectModel)>0:
                    for i in selectModel:
                        i.isSelected = False
                        selectModel.remove(i)
                selectModel.append(model)

            elif mouseSelectedCheck == 201:
                self.rotationAxis = 'rotX'
                if model.isSelected:
                    model.isSelected = True
                    selectModel.append(model)

            elif mouseSelectedCheck == 202:
                self.rotationAxis = 'rotY'
                if model.isSelected:
                    model.isSelected = True
                    selectModel.append(model)

            elif mouseSelectedCheck == 203:
                self.rotationAxis = 'rotZ'
                
                if model.isSelected:
                    model.isSelected = True
                    selectModel.append(model)

            elif mouseSelectedCheck == 101:
                self.translationAxis = 'transX'
                if model.isSelected:
                    model.isSelected = True
                    selectModel.append(model)

            elif mouseSelectedCheck == 102:
                self.translationAxis = 'transY'
                if model.isSelected:
                    model.isSelected = True
                    selectModel.append(model)

            elif mouseSelectedCheck == 103:
                self.translationAxis = 'transZ'
                if model.isSelected:
                    model.isSelected = True
                    selectModel.append(model)

            elif mouseSelectedCheck == 301:
                self.translationAxis = 'transXY'
                if model.isSelected:
                    model.isSelected = True
                    selectModel.append(model)

            elif mouseSelectedCheck == 302:
                self.translationAxis = 'transYZ'
                if model.isSelected:
                    model.isSelected = True
                    selectModel.append(model)

            elif mouseSelectedCheck == 303:
                self.translationAxis = 'transXZ'
                if model.isSelected:
                    model.isSelected = True
                    selectModel.append(model)

            else:

                #model is deselected
                model.isSelected = False
        return selectModel

    # mouse click selection checking
    def mouseSelectedCheck(self):

        # read pixel color
        self.color = GL.glReadPixels(self.xMousePosition,self.windowHeight - self.yMousePosition, 1,1,GL.GL_RGBA,GL.GL_UNSIGNED_BYTE)

        # color checking
        # if it's black mouse doesn't select anything
        if self.color[0] == 0 and self.color[1] == 0 and self.color[2] == 0:
            mouseSelected = False
        else:
            mouseSelected = True
        mouseSelected = False
        
        if mouseSelected == False:
            mouseSelected = self.selectObjectWithBuffer(self.xMousePosition,self.yMousePosition)

        return mouseSelected

    # get rotation matrix from vector
    def getVectorAngle(self,v1,v2):

        # get normal vector
        v = np.cross(v1,v2)
        
        # get vector norm
        s = np.linalg.norm(v)

        if s >0.0:
            c = np.dot(v1,v2)
            skV = np.array([[0,-v[2],v[1]],
                            [v[2],0,-v[0]],
                            [-v[1],v[0],0]])
            r = np.eye(3)+skV+np.dot(skV,skV)*((1-c)/s**2)
        else:
            r = np.eye(3)
        return r

    # get rotation matrix using ray casting
    def mouseRotate(self,x,y,rotationAxis,mode):

        # check rotation axis 
        if rotationAxis == 'rotX':
            axis = 0
            array = np.asarray([1.,0.,0.])
        elif rotationAxis == 'rotY':
            axis = 1
            array = np.asarray([0.,1.,0.])
        elif rotationAxis == 'rotZ':
            axis = 2
            array = np.asarray([0.,0.,1.])   
        # get model currentM
        # model = self.modelDicts['model'][self.modelDicts['runModelIdx']]
        model = self.selectedModel[0]
        newM = model.currentM.copy()
        newN = model.currentM.copy()
        if rotationAxis != 'None':
            # rotate in world coordinate
            if mode == False:
                newM[0:3,0:3] = np.eye(3)

            # set center position
            center = [newM[0][3],newM[1][3],newM[2][3]]

            # init plane Normal and Position
            plane = Plane([newM[0][axis],newM[1][axis],newM[2][axis]],center)

            # get current Ray casting position
            newRay = Ray(x,y)

            # get position where ray intersect with plane
            newIntersect = newRay.intersects(plane)
            oldIntersect = self.oldRay.intersects(plane)

            # check if rays intersect plane or not
            # if not project it on front plane
            if newIntersect[0] ==  None:
                newplane = Plane([0,0,1],[newM[0][3],newM[1][3],newM[2][3]+5])
                newIntersect = newRay.intersects(newplane)
                oldIntersect = self.oldRay.intersects(newplane)

            #check again
            if newIntersect[0]!= None and oldIntersect[0] != None:
                
                # get vector from center
                vec1 = oldIntersect - center
                vec2 = newIntersect - center

                # normalize vector
                vector1 = vec1/np.linalg.norm(vec1)
                vector2 = vec2/np.linalg.norm(vec2)

                #get rotate angle
                angle = self.productAngle(vector1,vector2)

                # check axis direction
                axisDir  = np.dot(array,np.asarray([[newM[0][axis]],[newM[1][axis]],[newM[2][axis]]]))

                # get angle direction
                rotationM = self.getVectorAngle(vector1,vector2)
                r = R.from_matrix(rotationM)
                matDir = r.as_rotvec()

                # set angle  ratio
                ratio = 0.04

                # get rotation matrix
                if rotationAxis == 'rotX':
                    matrix = (self.matrixM(angle*ratio,axisDir*matDir[0],0,0))
                elif rotationAxis == 'rotY':
                    matrix = (self.matrixM(angle*ratio,0,axisDir*matDir[1],0))
                elif rotationAxis == 'rotZ':
                    matrix = (self.matrixM(angle*ratio,0,0,axisDir*matDir[2]))
                else:
                    matrix = newM[0:3,0:3]
                if mode == False:
                    newMatrix = np.dot(matrix,newN[0:3,0:3])
                else:
                    newMatrix = np.dot(newN[0:3,0:3],matrix)

            else:
                print("no intersect out")
                newMatrix = newN[0:3,0:3]
            # set current ray for the next ray 
            # self.oldRay = newRay
            newN[0:3,0:3] = newMatrix
        else:
            center = [newM[0][3],newM[1][3],newM[2][3]]
            newRay = Ray(x,y)
            cur = newRay.sphereIntersect(center,20)
            last = self.oldRay.sphereIntersect(center,20)
            if cur[0]!= None and last[0] != None:
                diff = cur - last
                angle = 90 * math.sqrt(diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2])
                axis = [0,0,0]
                axis[0] = last[1]*cur[2] - last[2]*cur[1]
                axis[1] = last[2]*cur[0] - last[0]*cur[2]
                axis[2] = last[0]*cur[1] - last[1]*cur[0]
                matrix = self.matrixM(angle*0.0005,-axis[0],-axis[1],-axis[2])
                newMatrix = np.dot(matrix,newN[0:3,0:3])
                newN[0:3,0:3] = newMatrix
        self.oldRay = newRay
        return newN

    # translation function
    def mouseTranslate(self,x,y,translationAxis,mode):

        #init variable
        newPoint1 = np.asarray([None,None,None])
        oldPoint1 = np.asarray([None,None,None])
        newPoint1 = np.asarray([None,None,None])
        newPoint2 = np.asarray([None,None,None])
        newPoint = np.asarray([0,0,0])
        oldPoint = np.asarray([0,0,0])
        norm1 = 0
        norm2 = 0
        # get model currentM
        # model = self.modelDicts['model'][self.modelDicts['runModelIdx']]
        model = self.selectedModel[0]
        newM = model.currentM.copy()

        # translate in world coordinate
        if mode == False:
            newM[0:3,0:3] = np.eye(3)

        # set center position
        center = [newM[0][3],newM[1][3],newM[2][3]]

        # get current Ray casting position
        newRay = Ray(x,y)
        if translationAxis == 'transX' or translationAxis == 'transY' or translationAxis == 'transZ':
            # check rotation axis 
            if translationAxis == 'transX':
                # itself axis
                axis0 = 0

                # near by axis
                axis1 = 1
                axis2 = 2
                
            elif translationAxis == 'transY':
                # itself axis
                axis0 = 1

                # near by axis
                axis1 = 0
                axis2 = 2
                
            elif translationAxis == 'transZ':
                # itself axis
                axis0 = 2

                # near by axis
                axis1 = 0
                axis2 = 1
                

            # check moving coordinate
            if mode == True or mode == False:

                # init plane Normal and Position
                plane1 = Plane([newM[0][axis1],newM[1][axis1],newM[2][axis1]],center)
                plane2 = Plane([newM[0][axis2],newM[1][axis2],newM[2][axis2]],center)

                # get position where ray intersect with 2 planes
                newIntersect1 = newRay.intersects(plane1)
                newIntersect2 = newRay.intersects(plane2)
                oldIntersect1 = self.oldRay.intersects(plane1)
                oldIntersect2 = self.oldRay.intersects(plane2)
                
                # init line equation
                lineIntersect = Line([newM[0][axis0],newM[1][axis0],newM[2][axis0]],center)

                # check intersection for 2 ray
                if newIntersect1[0] != None and oldIntersect1[0] != None:

                    # project the point on plane to line equation
                    newPoint1 = self.pointProjectOnLine(lineIntersect,newIntersect1)
                    oldPoint1 = self.pointProjectOnLine(lineIntersect,oldIntersect1)
                    norm1 = round(math.sqrt(np.linalg.norm(newIntersect1)),0)

                if newIntersect2[0] != None and oldIntersect2[0] != None:

                    # project the point on plane to line equation
                    newPoint2 = self.pointProjectOnLine(lineIntersect,newIntersect2)
                    oldPoint2 = self.pointProjectOnLine(lineIntersect,oldIntersect2)
                    norm2 = round(math.sqrt(np.linalg.norm(newIntersect2)),0)
                # choose the moving point
                # choose the lower norm (the closest)
                if newPoint1[0] != None and newPoint2[0] != None:
                    if norm1>=norm2:
                        newPoint = newPoint2
                        oldPoint = oldPoint2
            
                    elif norm1<norm2:
                        newPoint = newPoint1
                        oldPoint = oldPoint1
                        
                # if one have no intersect choose another
                elif newPoint1[0] != None and newPoint2[0] == None:
                    newPoint = newPoint1
                    oldPoint = oldPoint1
                elif newPoint1[0] == None and newPoint2[0] != None:
                    newPoint = newPoint2
                    oldPoint = oldPoint2
        

        elif translationAxis == 'transXY' or translationAxis == 'transYZ' or translationAxis == 'transXZ':
            if translationAxis == 'transXY':
                axis = 2

            if translationAxis == 'transYZ':
                axis = 0
            
            if translationAxis == 'transXZ':
                axis = 1

            plane = Plane([newM[0][axis],newM[1][axis],newM[2][axis]],center)
            newPoint = newRay.intersects(plane)
            oldPoint = self.oldRay.intersects(plane)
            
        # moving along the screen
        elif mode == "worldxy":

            # init plane Normal and Position
            plane = Plane([0,0,1],center)

            # point that the ray intersect on the plane
            newPoint = newRay.intersects(plane)
            oldPoint = self.oldRay.intersects(plane)
        else:
            newPoint = np.asarray([0,0,0])
            oldPoint = np.asarray([0,0,0])
        #  distant vector
        distant = newPoint - oldPoint
        # set current ray for the next ray
        self.oldRay = newRay
        

        return distant
    def productAngle(self,vector1,vector2):

        # get angle from dot product equation
        # angle = arccos(vector1.vector2/|vector1||vector2|)
        # vector1.vector2
        dotProduct = np.dot(vector1,vector2)

        # |vector1|
        sqrVec1 = vector1[0]*vector1[0]+vector1[1]*vector1[1]+vector1[2]*vector1[2]
        vector1Norm = math.sqrt(sqrVec1)

        # |vector2|
        sqrVec2 = vector2[0]*vector2[0]+vector2[1]*vector2[1]+vector2[2]*vector2[2]
        vector2Norm = math.sqrt(sqrVec2)

        # vector from vector1 to vector2
        dx = vector2[0] - vector1[0]
        dy = vector2[1] - vector1[1]
        dz = vector2[2] - vector1[2]

        # cos result must be between -1 to 1 
        if dotProduct/(vector1Norm*vector2Norm)>=1 or dotProduct/(vector1Norm*vector2Norm)<=-1:

            # calculate vector from resultant vector
            angle = (90*math.sqrt(dx*dx + dy*dy + dz*dz))%1
        else:

            # calculate vector from dot product
            angle = math.acos(dotProduct/(vector1Norm*vector2Norm))

            # convert radian into degree
            angle = ((angle/math.pi)*180)%1

        # return degree
        return angle
    
    # project point on line receive line equation and point position
    def pointProjectOnLine(self,line,point):

        # line direction
        dir = line.direction

        # init point on line 
        P = line.point

        # point that we want to project
        Q = point

        # vector from point on the line to the set point
        PQ = Q - P

        # project vector PQ on the desire line
        projPQ = (np.dot(PQ,dir)/(dir[0]*dir[0]+dir[1]*dir[1]+dir[2]*dir[2]))*dir

        # point that projected on desired line
        s = P + projPQ

        # return vector point
        return s

    # get rotation matrix from angle and axis of rotation for each axis
    def matrixM(self,angle,x,y,z):
        v = [x,y,z]
        matrix = np.eye(3)
        if v!= [0,0,0]:
            vec = np.asarray(v)

            # vector norm
            a = np.linalg.norm(vec)

            # normalise axis of rotation if it not yet normalise
            if a!=1:
                v[0] /= a 
                v[1] /= a
                v[2] /= a
   
            vec2 = np.asarray(v)
          
            x = vec2[0]
            y = vec2[1]
            z = vec2[2]
            c = math.cos(angle)
            s = math.sin(angle)
            t = 1-c
            matrix[0][0] = x*x*t + c
            matrix[0][1] = x*y*t - z*s
            matrix[0][2] = x*z*t + y*s
            matrix[1][0] = y*x*t + z*s
            matrix[1][1] = y*y*t + c
            matrix[1][2] = y*z*t - x*s
            matrix[2][0] = x*z*t - y*s
            matrix[2][1] = y*z*t + x*s
            matrix[2][2] = z*z*t + c

        # return rotation matrix
        return matrix

class StylusController2(StylusController):
    def __init__(self,packData):
        super().__init__(packData)
        self.hBaseToWorld = np.eye(4)
        self.hOpenGlToImu = np.array([[1,0,0,0],
                                  [0,0,-1,0],
                                  [0,1,0,0],
                                  [0,0,0,1]])
        self.cfg={"homeCfg":(72.6287059, -4.60903242,   20.4418449)}
        self.isImuInit = False
        self.imuTrans = np.eye(4)
        
    def runEvent(self,event):
        if self.keyCha == ord('k'):
            print("Init Imu..")
            self.isImuInit = True
            print("Finish !!")
            
        super().runEvent(event)
        
    def setTransform(self, cursorTransform,imuQuat):
        
        # read cursor transform from stylus and scaling
        cursorTransform[0,3] = cursorTransform[0,3].copy() * 0.3
        cursorTransform[1,3] = cursorTransform[1,3].copy() * 0.2
        cursorTransform[2,3] = cursorTransform[2,3].copy() * 0.3
        
        # offset home position
        cursorTransform[0,3] -= self.cfg["homeCfg"][0]
        cursorTransform[1,3] -= self.cfg["homeCfg"][1]
        cursorTransform[2,3] -= self.cfg["homeCfg"][2]
        
        # cursorTransform[0:3,3] = cursorTransform[0:3,3].copy() * 1
        
        # get translation from dof 1 to enf
        cursorTransform = np.dot(self.hOpenGlToBase,cursorTransform)
        
        # # get endeffector from orientation
        self.readImuQuat(imuQuat)
        
        cursorTransform[0:3,0:3] = self.imuTrans[0:3,0:3].copy()
        cursorTransform[0:3,0:3] = np.dot(self.hOpenGlToImu[0:3,0:3],cursorTransform[0:3,0:3])
        cursorTransform[0:3,0:3] = np.dot(cursorTransform[0:3,0:3],self.hOpenGlToImu[0:3,0:3].T)
        
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
            