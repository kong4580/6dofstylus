import fltk
import numpy as np
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
            
        if self.keyCha == 'l':
            self.flags['addLog'] = True
            
            
        # if self.keyCha == 'm':
        #     self.toggleFlags('resetModelTransform')
            
        self.keyCha = None


class StylusController(CommonController):
    def __init__(self,packData):
        super().__init__(packData)
        # set config
        self.cfg={"homeCfg":(5.23747141, -0.01606842, -0.3270202)}
        self.transform = np.eye(4)
        self.cursor = packData['cursor']
        
    def runEvent(self,event):
        self.runCommonEvent()
        if event == 999: # move cursor
            self.cursor.moveModel(self.transform)
        if event >= 1000:
            key = event%1000
            if key == 1:
                self.push = True
                self.release = False
                self.buttonNum = 1
                print("left click!")
                selectedModel = self.selectModel()
                
                # show all model that is selected
                for model in selectedModel:
                    print("selectModel = ",model.name)
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
        
        # run through all models in opengl window class
        for model in self.modelDicts['model']:
            
            # if cursor position is in model
            if model.isPointInsideConvexHull(self.cursor.centerPosition):
                
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