
import sys
from functools import partial

import fltk
import math
from scipy.spatial.transform import Rotation as R
import numpy as np

from drawFunc import *
from OpenGLWindow import OpenGLWindow
from Model import Model

#test git rearrange
class Gui():
    
    # init class GUI
    def __init__(self):
        
        # init fltk window
        self.__initWindow(size=(800,600),name="UI")
        
        # init opengl window class
        self.__initOpenglWindow(size=(0,0,600,600),name="opengl")

        #resize fltk and opengl window
        self.window.resizable(self.openglWindow)
        
        # init side output
        self.__initOutputWidgetStorage()
        
        # set config
        self.cfg={"homeCfg":(5.23747141, -0.01606842, -0.3270202)}

        # set cursor scale
        self.cursorSpeed = 0.05
        
        # variable for revolute cursor mode
        self.cursorIsHold = False
        self.initHoldCursor = None
        self.offsetCursor = np.eye(4)
        
        # variable for creating log 
        self.addLog = False
        self.log = {
                        "name":None,
                        "department":None,
                        "mayaFamiliar":None,
                        "dominantHand":None,
                        "testNumber":6
                    }
        # log file name
        self.logFileName = "./testLogStylus.csv"

    # init fltk window
    def __initWindow(self,size=(800,600),name="UI"):
        print("Init GUI window ... ",end="")
        self.window = fltk.Fl_Window(size[0],size[1],name)
        print("Done !")
        
    # init opengl window
    def __initOpenglWindow(self,size=(0,0,600,600),name="opengl"):
        print("Init openGl window ... ",end="")
        self.openglWindow = OpenGLWindow(size[0],size[1],size[2],size[3],name)
        print("Done !")
        
    # init side output
    def __initOutputWidgetStorage(self):
        self.outputWidgetStorage = self.__createOutputWidget()
        self.cameraSliderBarWidget()
        self.cameraInputWidget()
        # self.positionOutputWidget()
        self.textWidget()
    
    # create text widget
    def textWidget(self):
        disp = fltk.Fl_Text_Display(600, 250, 300, 350, "Shortcut Key")
        tbuff = fltk.Fl_Text_Buffer()
        disp.buffer(tbuff)
        tbuff.text("Key\tDescription\n"
                   " Q\tShow and hide model\n"
                   " 1\tShow model wireframe\n"
                   " 2\tTurn model to opacity 50%\n"
                   " M\tReset model position\n"
                   " L\tAdd User profie\n"
                   " P\tStart test mode (start timer)\n"
                   " D\tCalculate IoU\n"
                   " W\tTranslation Mode\n"
                   "\n"
                   "In Translation Mode\n"
                   "**USE Scrollbar to translate\nalong Z-axis\n"
                   "**USE Mouse Drag to translate\n in XY-plane\n"
                   "\n"
                   "In Rotation Mode\n"
                   "Key to rotate around\n"
                   " Z\tBLUE axis\n"
                   " X\tRED  axis\n"
                   " C\tGreen axis\n"
                   "**USE Scrollbar or Mouse Drag\n to rotate object\n "
                   )

    # create slider bar widget
    def __createSliderBarWidget(self,xSliderBarPosition,ySliderBarPosition,widthSliderBar,heightSliderBar,nameLabel,minValue,maxValue,beginValue,stepValue):
            slider = fltk.Fl_Hor_Slider(xSliderBarPosition,ySliderBarPosition,widthSliderBar,heightSliderBar)
            slider.minimum(minValue)
            slider.maximum(maxValue)
            slider.value(beginValue)
            slider.step(stepValue)
            slider.align(fltk.FL_ALIGN_LEFT)
            return slider

    # create input widget
    def __createInputWidget(self,xInputPosition,yInputPosition,widthInput,heightInput,nameLabel,beginValue):
            inp = fltk.Fl_Input(xInputPosition,yInputPosition,widthInput,heightInput,nameLabel)
            inp.value(str(beginValue))
            inp.align(fltk.FL_ALIGN_LEFT)
            inp.when(fltk.FL_WHEN_ENTER_KEY)
            return inp
    
    # create output widget
    def __createOutputWidget(self,xInputPosition,yInputPosition,widthInput,heightInput,nameLabel,beginValue):
            out = fltk.Fl_Output(xInputPosition,yInputPosition,widthInput,heightInput,nameLabel)
            out.value(str(beginValue))
            out.align(fltk.FL_ALIGN_LEFT)
            return out

    #bunch of slider bar for controlling this UI
    def __allsliderBarWidget(self):
        self.storageArea = []
        # self.sliderBarValue =[0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.sliderBarValue = self.openglWindow.positionValue
        self.nameLabel = ["TransX", "TransY", "TransZ", "RotX", "RotY", "RotZ"]
        minValue =       [-10     , -10     , -10     , -180  , -180  , -180  ]
        maxValue =       [ 10     ,  10     ,  10     ,  180  ,  180  ,  180  ]
        stepValue = 0.1
        yWidgetPosition = 0
        for n in range(6) :
            slider = self.__createSliderBarWidget(695,yWidgetPosition,105,25,self.nameLabel[n],minValue[n],maxValue[n],self.sliderBarValue[n],stepValue)
            yWidgetPosition = yWidgetPosition + 25
            slider.callback(self.__sliderCB,n)
            self.storageArea.append(slider)
    
    # camera sliderbar control widget
    def cameraSliderBarWidget(self):
        self.storageCamera = []
        self.cameravalue= self.openglWindow.cameravalue
        self.nameLabel = ["Zoom", "CameraX", "CameraY"]
        minValue =       [ 0.01     , -1     , -1     ]
        # minValue = [1]*6
        maxValue =       [ 10       ,  1     ,  1     ]
        stepValue = 0.001
        yWidgetPosition = 150
        for n in range(3) :
            slider = self.__createSliderBarWidget(695,yWidgetPosition,105,25,self.nameLabel[n],minValue[n],maxValue[n],self.cameravalue[n],stepValue)
            yWidgetPosition = yWidgetPosition + 25
            slider.callback(self.__sliderCameraCB,n)
            self.storageCamera.append(slider)

    # model position output        
    def positionOutputWidget(self):
        self.storageOutput = []
        yWidgetPosition = 0
        self.sliderBarValue = self.openglWindow.positionValue
        self.nameLabel = ["TransX", "TransY", "TransZ", "RotX", "RotY", "RotZ"]
        for n in range(6) :
            output = self.__createOutputWidget(655,yWidgetPosition,145,25,self.nameLabel[n],self.sliderBarValue[n])
            yWidgetPosition = yWidgetPosition + 25
            output.callback(self.__outputCB,n)
            self.storageOutput.append(output)
        # update fltk from opengl 

    def updateFltk(self):
        for i in range(len(self.nameLabel)):
            # self.storageArea[i].value(self.openglWindow.positionValue[i])
            # self.storageInput[i].value(str(self.openglWindow.positionValue[i]))
            # self.storageOutput[i].value(str(self.openglWindow.positionValue[i]))
            pass
        self.addLog = self.openglWindow.flags['addLog']
        self.loghandle()

    # collect information from tester
    def loghandle(self):
        if self.addLog == True:
            self.createlogWindow()
            self.addLog = False
            self.openglWindow.flags['addLog'] = False
        if self.openglWindow.flags['logFinish'] == True:
            # print(self.openglWindow.log)
            self.log.update(self.openglWindow.log)
            with open(self.logFileName,"a+",newline='') as csvFile:
                dictWriter = csv.DictWriter(csvFile,fieldnames = self.log.keys())
                dictWriter.writerow(self.log)
            # print(self.log)
            print("\nFinished Test mode\nsave log at: {}\n".format(self.logFileName))
            self.openglWindow.flags['logFinish'] = False

    # create log window
    def createlogWindow(self):
        x = 150
        self.logWindow = fltk.Fl_Window(500,200)
        self.name = fltk.Fl_Input(x,10,300,25,"Name                  ")
        self.department = fltk.Fl_Input(x,40,300,25,"Department       ")
        self.Hand = fltk.Fl_Choice(x,70,300,25,"Dominant Hand")
        self.Hand.add("Right")
        self.Hand.add("Left")
        self.Hand.value(0)
        self.Maya = fltk.Fl_Choice(x,100,300,25,"Maya Familiar   ")
        for i in range(11):
            self.Maya.add(str(i))
        self.Maya.value(0)
        button = fltk.Fl_Button(200,150,100,25,"Confirm")
        button.callback(self.buttonCB,1)
        self.logWindow.show()
        self.logWindow.end()
        if self.addLog == False:
            self.logWindow.hide()
        elif self.addLog == True:
            self.logWindow.show()

    # buttin callback from log window
    def buttonCB(self,widget,v):
        self.logWindow.hide()
        self.log["name"] = self.name.value()
        self.log["department"] = self.department.value()
        self.log["mayaFamiliar"] = self.Maya.value()
        strhand = ""
        if self.Hand.value() == 0:
            strhand = "Right"
        if self.Hand.value() ==1:
            strhand = "Left"
        self.log["dominantHand"] = strhand
        # print(self.log)
        
    # bunch of input for controlling this UI
    def cameraInputWidget(self):
        self.storageInput = []
        yWidgetPosition = 150
        self.cameravalue = self.openglWindow.cameravalue
        self.nameLabel = ["Zoom", "CamX", "CamY"]
        for n in range(3) :
            inp = self.__createInputWidget(655,yWidgetPosition,40,25,self.nameLabel[n],self.cameravalue[n])
            yWidgetPosition = yWidgetPosition + 25
            inp.callback(self.__inputCB,n)
            self.storageInput.append(inp)


    # camera slider bar call back
    def __sliderCameraCB(self,widget, v):
        widgetValue = widget.value()
        self.cameravalue[v] = widgetValue
        self.storageInput[v].value(str(widgetValue))
        self.openglWindow.getCameraFromSlider(v,widgetValue)
        self.openglWindow.redraw()
    
    # camera input call back
    def __inputCB(self,widget, v):
        widgetValue = float(widget.value())
        widget.value(str(widgetValue))
        self.cameravalue[v] = widgetValue
        self.openglWindow.getCameraFromSlider(v,widgetValue)
        self.openglWindow.redraw()

    # model position output call back
    def __outputCB(self,widget, v):
        widgetValue = float(widget.value())
        widget.value(str(widgetValue))
        self.sliderBarValue[v] = widgetValue
        self.openglWindow.getPositionFromSlider(v,widgetValue)
        self.openglWindow.redraw()

    # camera slider bar call back
    def __sliderCameraCB(self,widget, v):
        widgetValue = widget.value()
        self.cameravalue[v] = widgetValue
        self.storageInput[v].value(str(widgetValue))
        self.openglWindow.getCameraFromSlider(v,widgetValue)
        self.openglWindow.redraw()
    
    # camera input call back
    def __inputCB(self,widget, v):
        widgetValue = float(widget.value())
        widget.value(str(widgetValue))
        # self.storageArea[v].value(widgetValue)
        self.cameravalue[v] = widgetValue
        self.openglWindow.getCameraFromSlider(v,widgetValue)
        self.openglWindow.redraw()

    # model position output call back
    def __outputCB(self,widget, v):
        widgetValue = float(widget.value())
        widget.value(str(widgetValue))
        self.sliderBarValue[v] = widgetValue
        self.openglWindow.getPositionFromSlider(v,widgetValue)
        self.openglWindow.redraw()

    # draw side output
    def __createOutputWidget(self):
        
        # side output area
        storage_area = []
        
        # create text box
        name=["TransX", "TransY", "TransZ", "RotX", "RotY", "RotZ","OriX","OriY","OriZ"]
        y=0
        for n in range(6):
            output = fltk.Fl_Output(660,y,60,25,name[n])
            y = y + 25
            output.align(fltk.FL_ALIGN_LEFT)
            output.value("0")
            storage_area.append(output)
        return storage_area
           
           
    
    # main call back update ui
    def updateUI(self,cursorPose,buttonStates,scale=20,cursorTransform=None):
        
        # convert pose format
        ### now not use ###
        cvtedPose = self.__cvtPose(cursorPose,self.cursorSpeed)
        
        # add cursor pose to openglWindow class
        ### now not use ###
        self.openglWindow.readPose(cvtedPose)
        
        # read cursor transform from stylus and scaling
        cursorTransform[0:3,3] = cursorTransform[0:3,3].copy() * 0.05
        
        # offset home position
        cursorTransform[0,3] -= self.cfg["homeCfg"][0]
        cursorTransform[1,3] -= self.cfg["homeCfg"][1]
        cursorTransform[2,3] -= self.cfg["homeCfg"][2]
        
        # cursorTransform[0:3,3] = cursorTransform[0:3,3].copy() * self.cursorSpeed
        # offset mouse mode
        if self.openglWindow.flags['offsetMode']:
            
            # find delta transform between first transform that trigger mode and last transform that turn off this mode
            self.offsetCursor = np.dot(cursorTransform,np.linalg.inv(self.openglWindow.cursorTransform.copy()))
            
            # offset only translation
            # self.offsetCursor[0:3,0:3] = np.eye(3)
            
        else:
            
            # new cursor transform is
            # inv(offsetCursor) * current cursor transform from stylus
            self.openglWindow.cursorTransform = np.dot(np.linalg.inv(self.offsetCursor),cursorTransform)
            
            # update opengl window
            self.openglWindow.redraw()
            
            # update side output value
            self.__updateOutput(cvtedPose)
            
        # check left mouse is clicked
        if buttonStates[0] == 1 and buttonStates[1] == 0:
            
            # pass
            
            # check what model is clicked
            print("left click!")
            selectedModel = self.selectModel()
            
            # show all model that is selected
            for model in selectedModel:
                print("selectModel = ",model.name)
        
        # check right mouse is clicked
        elif buttonStates[0] == 0 and buttonStates[1] == 1: 
            
            # pass     
            # realease model
            print("releaseModel")
            self.releaseModel()
            
        return
    
    
    
    # update output function
    def __updateOutput(self,pos):
        for i in range(6):
            a=float("{:.2f}".format(pos[i]))
            self.outputWidgetStorage[i].value(str(a))
    
    # convert pose from stylus to opengl
    # from [rotX, rotY, rotZ, tranX, tranY, tranZ]
    # to [tranX, tranY, tranZ, rotX, rotY, rotZ]
    # scaling with cursor speed 
    ### now not use ###
    def __cvtPose(self,pose,scale,offset=True):
        
        # init buffer
        newPose = pose.copy()
        real = [0]*9
        
        # switch position and convert radians to degrees
        for i in range(len(newPose)):
            
            if i <3:
                real[i+3] = ((newPose[i])*180)/(math.pi)
            else:
                real[i-3] = newPose[i]*scale
        
        # offset home postion
        if offset:
            real[0] -= self.cfg["homeCfg"][0]
            real[1] -= self.cfg["homeCfg"][1]
            real[2] -= self.cfg["homeCfg"][2]
                  
        # return convert pose
        return real
    
    # add model to opengl window class
    def addModel(self,name,drawFunction = None,position=(0,0,0),rotation=(0,0,0),obj=None):
        print("Add model name",name)
        self.openglWindow.addModel(name,drawFunction,position,rotation,obj=obj)
        print("Done!")
    
    # move model to specified position and rotation
    ### now not use ###
    def moveModel(self,name,pose=None,position=None,rotation=None):
        
        # if input is not pose massage
        if pose!=None:
            cvtedPose = self.__cvtPose(pose,scale=1,offset=False)
            position = (cvtedPose[0],cvtedPose[1],cvtedPose[2])
            rotation = (cvtedPose[3],cvtedPose[4],cvtedPose[5])
            self.openglWindow.moveModel(name,position,rotation)
        
        # if input is position and rotation
        elif position != None and rotation != None:
            self.openglWindow.moveModel(name,position,rotation)
            
    # select model
    def selectModel(self):
        
        # create selected model buffer
        selectModel = []
        
        # run through all models in opengl window class
        for model in self.openglWindow.modelDicts['model']:
            
            # if cursor position is in model
            if model.isPointInsideConvexHull(self.openglWindow.cursor.centerPosition):
                
                # model is selected
                model.isSelected = True
                
                # add model to selected model buffer
                selectModel.append(model)
        
        # return selected model buffer
        return selectModel
    
    # release model
    def releaseModel(self):
        
        # run through all models in opengl window class
        for model in self.openglWindow.modelDicts['model']:
            
            # change model to not selected
            model.isSelected = False