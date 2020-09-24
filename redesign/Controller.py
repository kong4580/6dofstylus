import fltk
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
                print('ss')
                ctl.keyCha = chr(fltk.Fl.event_key())
                print(ctl.keyCha)
            ctl.runEvent()    
             
class CommonController(Handler):
    
    def __init__(self,flags,modelDicts):
        super().__init__()
        self.flags = flags
        self.modelDicts = modelDicts
        
    def toggleFlags(self,flags):
        self.flags[flags] = not self.flags[flags]
        # return self.flags
    
    def runEvent(self):
        print(self.keyCha)
        if self.keyCha == 'q':
            print(self.keyCha)
            self.toggleFlags('showModel')
        self.keyCha = None


class StylusController(CommonController):
    def __init__(self,flags,modelDicts):
        super().__init__(flags,modelDicts)
        