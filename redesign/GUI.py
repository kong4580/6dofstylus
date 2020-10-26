
import sys
from functools import partial

import fltk
import math
from scipy.spatial.transform import Rotation as R
import numpy as np

from drawFunc import *
from OpenGLWindow import OpenGLWindow
from Model import Model

import csv 

#test git rearrange
class Gui():
    
    # init class GUI
    def __init__(self):
        self.controllerMode = None
        # init fltk window
        self.initWindow(size=(800,600),name="UI")
        
        # init opengl window class
        self.initOpenglWindow(size=(0,0,600,600),name="opengl")

        #resize fltk and opengl window
        self.window.resizable(self.openglWindow)
        
        # init side output
        self.initOutputWidgetStorage()
        
        # set config
        self.cfg={"homeCfg":(5.23747141, -0.01606842, -0.3270202)}

        # set cursor scale
        # self.cursorSpeed = 0.05
        
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
                        "testNumber":6,
                        "iou":None,
                        "avgIou":None,
                        "totalTime":None,
                        "modelPerSec":None
                    }
        # log file name
        self.logFileName = "./testLogStylus.csv"
        
        

    # init fltk window
    def initWindow(self,size=(800,600),name="UI"):
        print("Init GUI window ... ",end="")
        # set position x,position y, width, height, name
        self.window = fltk.Fl_Window(int(fltk.Fl.w()/2-size[0]/2),int(fltk.Fl.h()/2-size[1]/2),size[0],size[1],name)
        print("Done !")

    # init opengl window
    def initOpenglWindow(self,size=(0,0,600,600),name="opengl"):
        print("Init openGl window ... ",end="")
        # set position x,position y, width, height, name
        self.openglWindow = OpenGLWindow(size[0],size[1],size[2],size[3],name)
        print("Done !")
        
    # init side output
    def initOutputWidgetStorage(self):
        # model position output
        self.positionOutputWidget()

        # camera widget
        self.cameraSliderBarWidget()
        self.cameraInputWidget()

        # cursor speed widget
        self.cursorSpeedSliderBarWidget()
        
        # self.textWidget()

        # test mode informing
        self.testModeUIOff()
        self.testModeUIOn()

        # iou score output
        self.iouOutput()

        # coordinate informing
        self.coordinateOn()
        self.coordinateOff()
        
    # model position output        
    def positionOutputWidget(self):
        self.storageOutput = []
        
        # start position
        yWidgetPosition = 0

        # init value
        self.sliderBarValue = self.openglWindow.positionValue

        # init label name
        self.nameLabel = ["TransX", "TransY", "TransZ", "RotX", "RotY", "RotZ"]

        for n in range(6) :

            output = self.createOutputWidget(655,yWidgetPosition,145,25,self.nameLabel[n],self.sliderBarValue[n])
            yWidgetPosition = yWidgetPosition + 25
            self.storageOutput.append(output)

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
            slider = self.createSliderBarWidget(695,yWidgetPosition,105,25,self.nameLabel[n],minValue[n],maxValue[n],self.cameravalue[n],stepValue)
            yWidgetPosition = yWidgetPosition + 25
            slider.callback(self.sliderCameraCB,n)
            self.storageCamera.append(slider)

    # bunch of input for controlling this UI
    def cameraInputWidget(self):
        self.storageInput = []
        yWidgetPosition = 150
        self.cameravalue = self.openglWindow.cameravalue
        self.nameLabel = ["Zoom", "CamX", "CamY"]
        for n in range(3) :
            inp = self.createInputWidget(655,yWidgetPosition,40,25,self.nameLabel[n],self.cameravalue[n])
            yWidgetPosition = yWidgetPosition + 25
            inp.callback(self.inputCameraCB,n)
            self.storageInput.append(inp)


    # camera slider bar call back
    def sliderCameraCB(self,widget, v):
        widgetValue = widget.value()
        self.cameravalue[v] = widgetValue
        self.storageInput[v].value(str(widgetValue))
        self.openglWindow.getCameraFromSlider(v,widgetValue)
    
    # camera input call back
    def inputCameraCB(self,widget, v):
        widgetValue = float(widget.value())
        widget.value(str(widgetValue))
        self.cameravalue[v] = widgetValue
        self.openglWindow.getCameraFromSlider(v,widgetValue)

    # cursor speed control widget
    def cursorSpeedSliderBarWidget(self):
        self.cursorSpeed = []
        self.cursorSpeedValue = self.openglWindow.flags['cursorSpeed']
        slider = self.createSliderBarWidget(695,225,105,25,"Speed",0.0,2.0,self.cursorSpeedValue,0.25)
        slider.callback(self.sliderCursorSpeedCB,1)
        inputw = self.createInputWidget(655,225,40,25,"Speed",self.cursorSpeedValue)  
        inputw.callback(self.inputCursorSpeedCB,1)      
        self.cursorSpeed.append(slider)
        self.cursorSpeed.append(inputw)

    def sliderCursorSpeedCB(self,widget,n):
        widgetValue = widget.value()
        self.cursorSpeed[0] = widgetValue
        self.openglWindow.flags['cursorSpeed'] = widgetValue
        self.cursorSpeed[1].value(str(widgetValue))
        

    def inputCursorSpeedCB(self,widget,n):
        widgetValue = float(widget.value())
        widget.value(str(widgetValue))
        self.cursorSpeed[0] = widgetValue
        self.openglWindow.flags['cursorSpeed'] = widgetValue

    def cursorSpeedWidget(self):
        if self.controllerMode == 'stylus':
            self.cursorSpeed[0].hide()
            self.cursorSpeed[1].hide()

    # text test mode off
    def testModeUIOff(self):
        testStr = "Testing Mode : " + "OFF"
        self.testTextOff = fltk.Fl_Box(600,250,200,25,testStr)
        self.testTextOff.labelsize(18)
        self.testTextOff.labelfont(fltk.FL_BOLD)
        c = fltk.fl_rgb_color(204, 41, 0)
        self.testTextOff.labelcolor(c)

    # text test mode on
    def testModeUIOn(self):
        testStr = "Testing Mode : " + "ON"
        self.testTextOn = fltk.Fl_Box(600,250,200,25,testStr)
        self.testTextOn.labelsize(18)
        self.testTextOn.labelfont(fltk.FL_BOLD)
        c = fltk.fl_rgb_color(51, 204, 0)
        self.testTextOn.labelcolor(c)

    # test mode text handle
    def testModeUIHandle(self,testMode):
        if testMode == True:
            self.testTextOn.show()
            self.testTextOff.hide()
        elif testMode == False:
            self.testTextOn.hide()
            self.testTextOff.show()

    # create IoU output for all image
    def iouOutput(self):
        self.storageIouOutput = []
        y = 300
        width = 50
        height = 25
        iouBox = fltk.Fl_Box(600,275,100,height,"IoU Score")
        iouBox.box(fltk.FL_NO_BOX)
        for i in range(6):
            output = fltk.Fl_Output(650,y,width,height,str(i+1))
            output.value(str(0))
            y+=30
            self.storageIouOutput.append(output)

    # text local coordinate
    def coordinateOn(self):
        testStr = "AXIS : LOCAL"
        self.coordinateTextON = fltk.Fl_Box(682,570,100,25,testStr)
        self.coordinateTextON.labelfont(fltk.FL_BOLD)

    # text world coordinate
    def coordinateOff(self):
        testStr = "AXIS : GLOBAL"
        self.coordinateTextOff = fltk.Fl_Box(688,570,100,25,testStr)
        self.coordinateTextOff.labelfont(fltk.FL_BOLD)

    # coordinate text handle
    def coordinateUIHandle(self,mode):
        if self.controllerMode == 'mouse':
            if mode == True:
                self.coordinateTextON.show()
                self.coordinateTextOff.hide()
            elif mode == False:
                self.coordinateTextON.hide()
                self.coordinateTextOff.show() 
        else:
            self.coordinateTextON.hide()
            self.coordinateTextOff.hide()
    
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
    def createSliderBarWidget(self,xSliderBarPosition,ySliderBarPosition,widthSliderBar,heightSliderBar,nameLabel,minValue,maxValue,beginValue,stepValue):
            slider = fltk.Fl_Hor_Slider(xSliderBarPosition,ySliderBarPosition,widthSliderBar,heightSliderBar)
            slider.minimum(minValue)
            slider.maximum(maxValue)
            slider.value(beginValue)
            slider.step(stepValue)
            slider.align(fltk.FL_ALIGN_LEFT)
            return slider

    # create input widget
    def createInputWidget(self,xInputPosition,yInputPosition,widthInput,heightInput,nameLabel,beginValue):
            inp = fltk.Fl_Input(xInputPosition,yInputPosition,widthInput,heightInput,nameLabel)
            inp.value(str(beginValue))
            inp.align(fltk.FL_ALIGN_LEFT)
            inp.when(fltk.FL_WHEN_ENTER_KEY)
            return inp
    
    # create output widget
    def createOutputWidget(self,xInputPosition,yInputPosition,widthInput,heightInput,nameLabel,beginValue):
            out = fltk.Fl_Output(xInputPosition,yInputPosition,widthInput,heightInput,nameLabel)
            out.value(str(beginValue))
            out.align(fltk.FL_ALIGN_LEFT)
            return out

    # update fltk from opengl 
    def updateFltk(self):
        model = self.openglWindow.modelDicts['model'][self.openglWindow.modelDicts['runModelIdx']]
        r = R.from_matrix(model.worldToLocal[0:3,0:3])
        degree = r.as_euler('zyx', degrees=True)
        self.storageOutput[0].value(str(round(model.worldToLocal[0][3],2)))
        self.storageOutput[1].value(str(round(model.worldToLocal[1][3],2)))
        self.storageOutput[2].value(str(round(model.worldToLocal[2][3],2)))
        self.storageOutput[3].value(str(round(degree[2],2)))
        self.storageOutput[4].value(str(round(degree[1],2)))
        self.storageOutput[5].value(str(round(degree[0],2)))
        self.iouScore = [0,0,0,0,0,0]
        self.iouScore[:len(self.openglWindow.iouScore)] = self.openglWindow.iouScore
        for i in range(len(self.iouScore)):
            self.storageIouOutput[i].value(str(round(self.iouScore[i],2)))
        self.addLog = self.openglWindow.flags['addLog']
        self.loghandle()
        self.testModeUIHandle(self.openglWindow.flags['lineupTestMode'])
        self.coordinateUIHandle(self.openglWindow.flags['coordinate'])
        self.cursorSpeedWidget()
        for i in range(len(self.cameravalue)):
            self.storageCamera[i].value(self.openglWindow.cameravalue[i])
            self.storageInput[i].value(str(self.openglWindow.cameravalue[i]))
        

    # collect information from tester
    def loghandle(self):
        if self.addLog == True:
            self.log = {
                        "name":None,
                        "department":None,
                        "mayaFamiliar":None,
                        "dominantHand":None,
                        "testNumber":6,
                        "iou":None,
                        "avgIou":None,
                        "totalTime":None,
                        "modelPerSec":None
                    }
            self.createlogWindow()
            self.addLog = False
            self.openglWindow.flags['addLog'] = False
            self.addLog = False
        if self.openglWindow.flags['logFinish'] == True:
            self.log.update(self.openglWindow.log)
            with open(self.logFileName,"a+",newline='') as csvFile:
                dictWriter = csv.DictWriter(csvFile,fieldnames = self.log.keys())
                dictWriter.writerow(self.log)
            print("\nFinished Test mode\nsave log at: {}\n".format(self.logFileName))
            self.createScoreWindow()
            self.openglWindow.flags['logFinish'] = False

    # pop up score window after finish testing 
    def createScoreWindow(self):
        self.scoreWindow = fltk.Fl_Window(int(self.window.x() + self.window.w()/2-400/2),int(self.window.y()+self.window.h()/2-100/2),400,100)
        self.textbox = fltk.Fl_Box(100,10,200,25,"Finish Testing")
        self.textbox.box(fltk.FL_NO_BOX)
        self.textbox.labelsize(28)
        self.textbox.labelfont(fltk.FL_BOLD)
        self.avgIou = fltk.Fl_Output(225,40,300,25,"Average IoU : ")
        self.avgIou.labelfont(fltk.FL_BOLD)
        self.avgIou.box(fltk.FL_NO_BOX) 
        if self.log["avgIou"] != None and self.log["modelPerSec"]!=None:
            self.avgIou.value(str(round(self.log["avgIou"],2)))
            self.modelPerSec = fltk.Fl_Output(225,70,300,25,"Model Per sec : ")
            self.modelPerSec.labelfont(fltk.FL_BOLD)
            self.modelPerSec.box(fltk.FL_NO_BOX) 
            self.modelPerSec.value(str(round(self.log["modelPerSec"],2)))
            self.scoreWindow.show()
        self.scoreWindow.end()

    # create log window
    def createlogWindow(self):
        x = 150
        self.logWindow = fltk.Fl_Window(int(self.window.x() + self.window.w()/2-500/2),int(self.window.y()+self.window.h()/2-200/2),500,200)
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
        self.createScoreWindow()
        
                
    # main call back update ui
    def updateUI(self):
        
            
        self.checkModeEvent()
        # update opengl window
        self.openglWindow.redraw()
        self.updateFltk()
        # update side output value
        self.updateOutput(cvtedPose)
        
            
        return
    
    def checkModeEvent(self):
        
        if self.openglWindow.flags['checkIoU']:
            self.openglWindow.cameravalue=[1,0,0,1,1,1]
            self.openglWindow.flags['cursorSpeed'] = 1.0
            self.openglWindow.redraw()
            # fltk.Fl_wait(0.5)
            # calculate iou
            
            score = self.openglWindow.checkIoU()
            
            # update flags states from flags buffer
            
            # reset checkIoU flags
            self.openglWindow.flags['checkIoU'] = False
            # if in test mode
            if self.openglWindow.flags['lineupTestMode']:
               
                # reset model position
                self.openglWindow.flags['resetModelTransform'] = True         
                
                # add iou score to buffer
                self.openglWindow.iouScore = np.append(self.openglWindow.iouScore,score)
                
                # print(self.openglWindow.testNumber,self.openglWindow.log['testNumber'])
                # check that test all model or not
                self.openglWindow.testMode(self.openglWindow.log['testNumber'])
                # update test number
                self.openglWindow.testNumber += 1
                
                # if still in test mode
                if self.openglWindow.flags['lineupTestMode']:
                    
                    # open next backdrop
                    self.openglWindow.openBackdropFile("backdropImg/backdrop_"+str(self.openglWindow.testNumber)+".jpg")
            
            
            
        if self.openglWindow.flags['testMode']:
            print("Enable test mode\nNumber of test: ",self.openglWindow.log['testNumber'])
            self.openglWindow.testMode(self.openglWindow.log['testNumber'])
            # print( self.openglWindow.flags['testMode'])
            self.openglWindow.flags['testMode'] = False
            
        fltk.Fl_check()
    
    
    # update output function
    def updateOutput(self,pos):
        for i in range(6):
            a=float("{:.2f}".format(pos[i]))
            self.outputWidgetStorage[i].value(str(a))
    
    
    
    # add model to opengl window class
    def addModel(self,name,modelId,drawFunction = None,position=(0,0,0),rotation=(0,0,0),obj=None,objType = 'model',listOfJoint=[],showTarget = False,showPole = False):
        print("Add model name",name)
        self.openglWindow.addModel(name,modelId,drawFunction,position,rotation,obj=obj,objType=objType,listOfJoint=listOfJoint,showTarget=showTarget,showPole = showPole)
        print("Done!")
    
    