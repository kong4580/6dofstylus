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
        self.transFrom = np.eye(4)
    def registerController(self,controller):
        self.controllerList.append(controller)
    
    def readEvent(self,event):
        for ctl in self.controllerList:
            
            
            if event == fltk.FL_KEYUP:
                
                ctl.keyCha = chr(fltk.Fl.event_key())
                
            ctl.runEvent()    
             
class CommonController(Handler):
    
    def __init__(self,packData):
        super().__init__()
        self.flags = packData['flags']
        self.modelDicts = packData['modelDicts']
        self.log = packData['log']
    
    
    def toggleFlags(self,flags):
        self.flags[flags] = not self.flags[flags]
        # return self.flags
        
    
        
    def runEvent(self):
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
            
        if self.keyCha == 'l':
            self.flags['addLog'] = True
            
            
        # if self.keyCha == 'm':
        #     self.toggleFlags('resetModelTransform')
            
        self.keyCha = None


class StylusController(CommonController):
    def __init__(self,packData):
        super().__init__(packData)
        