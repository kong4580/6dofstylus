import fltk
import numpy as np
from scipy.spatial import ConvexHull
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
    
    def readEvent(self,event):
        for ctl in self.controllerList:
            
            
            if event == fltk.FL_KEYUP:
                
                ctl.keyCha = chr(fltk.Fl.event_key())
                
            ctl.runEvent(event)    
             
class CommonController(Handler):
    
    def __init__(self,packData):
        super().__init__()
        self.flags = packData['flags']
        self.modelDicts = packData['modelDicts']
        self.log = packData['log']
    
    
    def toggleFlags(self,flags):
        self.flags[flags] = not self.flags[flags]
        # return self.flags
        
    
        
    def runCommonEvent(self):
        # print(self.keyCha)
        
        if self.keyCha == 'q':
            # print("aa")
            self.toggleFlags('showModel')
            
        if self.keyCha == '1':
            self.toggleFlags('showModelWireframe')
            
        if self.keyCha == '2':
            self.toggleFlags('opacityMode')
        
        if self.keyCha == 's':
            self.toggleFlags('snapMode')
            self.flags['showCursor'] = not self.flags['snapMode']
        
        if self.keyCha == 'd':
            
            self.toggleFlags('checkIoU')
        
        if self.keyCha == 'p':
            
            self.toggleFlags('testMode')
        if self.keyCha == 'm':
            
            self.toggleFlags('resetModelTransform')
        if self.keyCha == 'l':
            self.flags['addLog'] = True
        
            
        self.keyCha = None


class StylusController(CommonController):
    def __init__(self,packData):
        super().__init__(packData)
        # set config
        self.cfg={"homeCfg":(5.23747141, -0.01606842, -0.3270202)}
        self.transform = np.eye(4)
        self.cursor = packData['cursor']
        self.selectedModel = []
        
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
                    
                    # set model position to home position ( identity )
                    model.currentM = np.eye(4)
                    
                    # turn off reset flags
                    self.flags['resetModelTransform'] = False
                
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