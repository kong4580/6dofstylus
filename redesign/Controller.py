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
                        'moveHistoryPosition':0,
                        'modelName':['']}
        self.windowHeight = packData['height']
        self.windowWidth = packData['width']
        self.cameraValue = packData['camera']
        self.fineRot = False
        self.fineTran = False
        self.modelMode = packData['modelMode']
    
    def updateModelPose(self,model,transform,artiModel,mode = 'fk'):
        if self.modelMode =='fk':
            if type(model).__name__ == "Joint":
                transform[0:3,3] = model.currentM[0:3,3]
            model.moveModel(transform)
            
        elif self.modelMode =='ik':
            
            if type(model).__name__ != "Joint":
                
                # move transform
                transform[0:3,0:3] = np.eye(3)
                model.moveModel(transform)
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
                
            
            # model.moveModel(self.history['moveHistory'][self.history['moveHistoryPosition']])
        else:
            print("nothing to undo")
            
    def redo(self):
        print(self.history['moveHistoryPosition'] , len(self.history['moveHistory']))
        if self.history['moveHistoryPosition']+1 < len(self.history['moveHistory']):
            self.history['moveHistoryPosition'] += 1
            modelPose = self.history['moveHistoryPosition']
            
            artiModel = self.modelDicts['model'][self.modelDicts['runModelIdx']]
            modelList = artiModel.getSubModel()
            for model in modelList:
                if model.name == self.history['modelName'][modelPose]:
                    
                    newM = self.history['moveHistory'][modelPose][1]
                    self.updateModelPose(model,newM,artiModel)
            # model = self.modelDicts['model'][self.modelDicts['runModelIdx']]
            # model.moveModel(self.history['moveHistory'][self.history['moveHistoryPosition']])
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
        print(self.history)
        
    def resetKey(self):
        self.fineRot = False
        self.fineTran = False
    def clearHistory(self):
        self.history = {'moveHistory':[np.eye(4)],
                        'moveHistoryPosition':0}
    
    def selectObjectWithBuffer(self,xPos,yPos):
        model = self.modelDicts['model'][self.modelDicts['runModelIdx']]

        # SET CAMERA VIEW
        vp = GL.glGetIntegerv(GL.GL_VIEWPORT)
        
        # set matrix mode to projection matrix
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()

        # pick position
        GLU.gluPickMatrix(xPos, vp[3] - yPos, 3, 3, vp)

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
        model.drawMatrixModel(selectedMode = True,mode = self.flags['mouseMode'])

        # return model id
        hits = GL.glRenderMode(GL.GL_RENDER)

        # if mouse select something
        if hits != []:
            modelselected = hits[0].names[0]
            # for i in hits:
            #     print(i.names[0],"print")
        else:
            modelselected = 0
        
        # return selectedModel id
        return modelselected
    
    def runCommonEvent(self):

        if self.keyCha == ord('q'):

            self.toggleFlags('showModel')
            
        if self.keyCha == ord('1'):
            self.toggleFlags('showModelWireframe')
            
        if self.keyCha == ord('2'):
            self.toggleFlags('opacityMode')
        
        if self.keyCha == ord('s'):
            self.toggleFlags('snapMode')
            
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
        self.cursorSpeed = 1
        self.speed = 0.1
    def runEvent(self,event):
        
        # Run common event
        self.runCommonEvent()
        # Check specific event for stylus
        
        self.flags['showCursor'] = not self.flags['snapMode']

        if event == 999: # move cursor
            self.cursor.moveModel(self.transform)
            model = self.modelDicts['model'][self.modelDicts['runModelIdx']]
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
                    print(self.fineTran)
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
                self.updateModelPose(model,newM,artiModel)
                # model.moveModel(newM)
                # print(model.name)
        if event >= 1000: # check button status
            
            # Get button key
            key = event%1000
            
            # LEFT CLICK
            if key == 1:
                self.push = True
                self.release = False
                self.buttonNum = 1
                
                print("left click!")
                
                # Check select model
                self.selectedModel = self.selectModel(mode = "buffer")
                
            # RIGHT CLICK
            elif key == 2:
                self.push = True
                self.release = False
                self.buttonNum = 2
                
                # add History log
                model = self.modelDicts['model'][self.modelDicts['runModelIdx']]
                artiModel = self.modelDicts['model'][self.modelDicts['runModelIdx']]
                modelList = artiModel.getSubModel()
                for model in modelList:
                    if model.isSelected:
                        self.addHistory(model.currentM)
                        
                    # Release model
                    self.releaseModel()
                print("releaseModel")
            
            # REALEASE ALL BUTTON
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
                
                print(modelId,model.modelId)
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
            # print(cursor.currentM)
            # calcualte delta TRANSFORM of cursor that first clicked and current cursor transform
            deltaTransform = np.dot(cursor.currentM,np.linalg.inv(model.cursorM))
            
            # calculate new model transformation
            newM = np.eye(4)
            newM = np.dot(deltaTransform,model.startclickM)
        
        # move object with center model is origin
        elif origin == "centermodel":
            
            # if not init
            if type(model.cursorM) == type(None) or ((self.fineRot or self.fineTran) and self.cursorSpeed != self.speed) or ((not (self.fineRot or self.fineTran)) and self.cursorSpeed != 1):
                
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
            if self.fineRot or self.fineTran:
                
                self.cursorSpeed = self.speed
            else:
                self.cursorSpeed = 1
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
                print("s",self.fineTran)
                newM[0:3,0:3] = model.startclickM[0:3,0:3]
        
            if model.modelId<10:
                newM[0,3] = model.startclickM[0,3]
                newM[1,3] = model.startclickM[1,3]
                newM[2,3] = model.startclickM[2,3]
                
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
        self.transform = np.eye(4)
        self.selectedModel = []
        # self.windowHeight = packData['height']
        # self.windowWidth = packData['width']
        # self.cameraValue = packData['camera']
        self.flags['mouseMode']= 'trans'
        self.rotationAxis = None
    def runEvent(self,event):
        
        # mouse position
        self.xMousePosition = fltk.Fl.event_x()
        self.yMousePosition = fltk.Fl.event_y()
        
        

        # if self.keyCha == ord('x'):
        #     self.flags['mouseMode'] = 'rotX'

        # if self.keyCha == ord('c'):
        #     self.flags['mouseMode'] = 'rotY'

        # check model selection when mouse click
        if event == fltk.FL_PUSH:
            self.lastPosX = self.xMousePosition
            self.lastPosY = self.yMousePosition
            # print("left click!")
            self.selectedModel = self.selectModel()
            if self.selectedModel != []:
                self.old=self.selectedModel[0].currentM.copy()

        # run when there is something selected   
        if self.selectedModel != []:
            artiModel = self.modelDicts['model'][self.modelDicts['runModelIdx']]
            model =self.selectedModel[0]
            
            newM = model.currentM.copy()

            # move method when model is selected 
            if model.isSelected:
                
                # mousewheel event
                if event == fltk.FL_MOUSEWHEEL:
                    newM = self.mouseWheel()

                #mouse drag event
                if event == fltk.FL_DRAG:
                    newM = self.mouseDrag()
                    # print(newM)
                # store model position after release mouse# model is selected
                if event == fltk.FL_RELEASE:
                    self.addHistory(self.old,newM,model)
                    
                    # print(self.history)
            # move model with new matrix
            self.updateModelPose(model,newM,artiModel)
            # print(model.name)
            # model.moveModel(newM)
            
        # run common controller event 
        self.runCommonEvent()
        self.flags['showModelFrame'] = not self.flags['snapMode']
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
                self.updateModelPose(model,newM,artiModel)
                # model.moveModel(newM)
            self.addHistory(self.old,newM,model)
            
        # every model will be deselected during checking Iou
        if self.flags['checkIoU']:
            for model in self.modelDicts['model']:
                model.isSelected = False

    # mouse wheel moving method
    def mouseWheel(self):
        model = self.modelDicts['model'][self.modelDicts['runModelIdx']]
        model =self.selectedModel[0]
        
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
        model =self.selectedModel[0]
        
        newM = model.currentM.copy()

        # calculate ratio from mouse to window
        ratioX = -(5*(self.windowWidth/10)/(newM[2][3] -10))
        ratioY = -(5*(self.windowHeight/10)/(newM[2][3] -10))
        # newM[0,3] = -5
        
        # drag to translate
        if self.flags['mouseMode'] == 'trans':
            dx = newM[0,3].copy()
            dy = newM[1,3].copy()
            xTranslatePosition = recentX/ratioX + dx
            yTranslatePosition = -(recentY/ratioY) + dy
            
            
            newM[0,3] = xTranslatePosition
            newM[1,3] = yTranslatePosition
            
        # drag to rotate
        elif self.flags['mouseMode'] == 'rot':
            
            # calculate displacement of mouse moving
            dx = recentX
            dy = -recentY
            degree = math.sqrt(dx*dx+dy*dy)

            # get new rotation matrix
            if self.rotationAxis != 'rotZ':
                if dx+dy>0:
                    newM =self.rotationMatrixTransform(self.rotationAxis,degree)
                else:
                    newM = self.rotationMatrixTransform(self.rotationAxis,-degree)
            else:
                if dx+dy<0:
                    newM = self.rotationMatrixTransform(self.rotationAxis,degree)
                else:
                    newM = self.rotationMatrixTransform(self.rotationAxis,-degree)    

        return newM

    # calculate rotation matrix to rotate around current local axis
    def rotationMatrixTransform(self,rotationAxis,deg):
        # model = self.modelDicts['model'][self.modelDicts['runModelIdx']]
        model =self.selectedModel[0]
        newM = model.currentM.copy()

        # decrease degree size
        degree = deg*0.005

        # calculate rotation matrix
        degreeXMatrix = np.asarray([[1,0,0],
                                    [0,math.cos(degree),-math.sin(degree)],
                                    [0,math.sin(degree),math.cos(degree)]])
        degreeYMatrix = np.asarray([[math.cos(degree),0,math.sin(degree)],
                                    [0,1,0],
                                    [-math.sin(degree),0,math.cos(degree)]])
        degreeZMatrix = np.asarray([[math.cos(degree),-math.sin(degree),0],
                                    [math.sin(degree),math.cos(degree),0],
                                    [0,0,1]])

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
        artiModel = self.modelDicts['model'][self.modelDicts['runModelIdx']]
        modelList = artiModel.getSubModel()
        
        # self.rotationAxis = None
        # check selection    
        for model in modelList:
            # print(self.mouseSelectedCheck(),model.modelId)
            if self.mouseSelectedCheck() == model.modelId or self.mouseSelectedCheck() == True:

                # model is selected
                model.isSelected = True
                print("model")
                #add to selected model buffer
                selectModel.append(model)
            elif self.mouseSelectedCheck() == 201:
                self.rotationAxis = 'rotX'
                model.isSelected = True
                selectModel.append(model)
            elif self.mouseSelectedCheck() == 202:
                self.rotationAxis = 'rotY'
                model.isSelected = True
                selectModel.append(model)
            elif self.mouseSelectedCheck() == 203:
                self.rotationAxis = 'rotZ'
                model.isSelected = True
                selectModel.append(model)
            else:

                #model is deselected
                model.isSelected = False
            # print(self.rotationAxis)
        # print(selectModel[0].name)
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
        mouseSelected = False
        
        if mouseSelected == False:
            mouseSelected = self.selectObjectWithBuffer(self.xMousePosition,self.yMousePosition)
        # print(mouseSelected)

        return mouseSelected

class StylusController2(StylusController):
    def __init__(self,packData):
        super().__init__(packData)
        self.hBaseToWorld = np.eye(4)
        self.hOpenGlToImu = np.array([[-1,0,0,0],
                                  [0,0,-1,0],
                                  [0,-1,0,0],
                                  [0,0,0,1]])
        
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
        cursorTransform[0:3,3] = cursorTransform[0:3,3].copy() * 0.05
       
        # offset home position
        cursorTransform[0,3] -= self.cfg["homeCfg"][0]
        cursorTransform[1,3] -= self.cfg["homeCfg"][1]
        cursorTransform[2,3] -= self.cfg["homeCfg"][2]
        # cursorTransform[0:3,3] = cursorTransform[0:3,3].copy() * 1
        
        # get translation from dof 1 to enf
        cursorTransform = np.dot(self.hOpenGlToBase,cursorTransform)
        
        # get endeffector from orientation
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