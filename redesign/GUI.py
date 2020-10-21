
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
                        "testNumber":6,
                        "iou":None,
                        "avgIou":None,
                        "totalTime":None,
                        "modelPerSec":None
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
        # self.outputWidgetStorage = self.__createOutputWidget()
        self.cameraSliderBarWidget()
        self.cameraInputWidget()
        self.positionOutputWidget()
        # self.textWidget()
        self.testModeUIOff()
        self.testModeUIOn()
        self.iouOutput()
    
    # text test mode off
    def testModeUIOff(self,lineUporNot = False):
        testStr = "Testing Mode : " + "OFF"
        self.testTextOff = fltk.Fl_Box(600,250,200,25,testStr)
        self.testTextOff.labelsize(18)
        self.testTextOff.labelfont(fltk.FL_BOLD)
        c = fltk.fl_rgb_color(204, 41, 0)
        self.testTextOff.labelcolor(c)

    # text test mode on
    def testModeUIOn(self,lineUporNot = False):
        testStr = "Testing Mode : " + "ON"
        self.testTextOn = fltk.Fl_Box(600,250,200,25,testStr)
        self.testTextOn.labelsize(18)
        self.testTextOn.labelfont(fltk.FL_BOLD)
        c = fltk.fl_rgb_color(51, 204, 0)
        self.testTextOn.labelcolor(c)

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
            self.storageOutput.append(output)
        
    # test mode text handle
    def testModeUIHandle(self,testMode):
        if testMode == True:
            self.testTextOn.show()
            self.testTextOff.hide()
        elif testMode == False:
            self.testTextOn.hide()
            self.testTextOff.show()

    # update fltk from opengl 
    def updateFltk(self):
        model = self.openglWindow.modelDicts['model'][self.openglWindow.modelDicts['runModelIdx']]
        # print(model)
        r = R.from_matrix(model.worldToLocal[0:3,0:3])
        degree = r.as_euler('zyx', degrees=True)
        self.storageOutput[0].value(str(round(model.worldToLocal[0][3],2)))
        self.storageOutput[1].value(str(round(model.worldToLocal[1][3],2)))
        self.storageOutput[2].value(str(round(model.worldToLocal[2][3],2)))
        # print(round(model.worldToLocal[2][3],2))
        # print("*********************")
        self.storageOutput[3].value(str(round(degree[2],2)))
        self.storageOutput[4].value(str(round(degree[1],2)))
        self.storageOutput[5].value(str(round(degree[0],2)))
        # print("what")
        self.iouScore = [0,0,0,0,0,0]
        self.iouScore[:len(self.openglWindow.iouScore)] = self.openglWindow.iouScore
        for i in range(len(self.iouScore)):
            self.storageIouOutput[i].value(str(round(self.iouScore[i],2)))
        self.addLog = self.openglWindow.flags['addLog']
        self.loghandle()
        self.testModeUIHandle(self.openglWindow.flags['lineupTestMode'])
        for i in range(len(self.cameravalue)):
            self.storageCamera[i].value(self.openglWindow.cameravalue[i])
            self.storageInput[i].value(str(self.openglWindow.cameravalue[i]))
            # self.storageOutput[i].value(str(self.openglWindow.positionValue[i]))
            # pass
        
        # self.storageInput[0].value[str(round(self.openglWindow.cameravalue[0],2))]
        # print(self.openglWindow.cameravalue)
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
            # print(self.openglWindow.log)
            self.log.update(self.openglWindow.log)
            with open(self.logFileName,"a+",newline='') as csvFile:
                dictWriter = csv.DictWriter(csvFile,fieldnames = self.log.keys())
                dictWriter.writerow(self.log)
            print("\nFinished Test mode\nsave log at: {}\n".format(self.logFileName))
            self.createScoreWindow()
            self.openglWindow.flags['logFinish'] = False

    # pop up score window after finish testing 
    def createScoreWindow(self):
        self.scoreWindow = fltk.Fl_Window(400,100)
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
                
    # main call back update ui
    def updateUI(self):
        
            
        self.checkModeEvent()
        # update opengl window
        self.openglWindow.redraw()
        self.updateFltk()
        # update side output value
        self.__updateOutput(cvtedPose)
        
            
        return
    
    def checkModeEvent(self):
        
        if self.openglWindow.flags['checkIoU']:
            self.openglWindow.cameravalue=[1,0,0,1,1,1]
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
            self.openglWindow.flags['testMode'] = False
            
        fltk.Fl_check()
    
    
    # update output function
    def __updateOutput(self,pos):
        for i in range(6):
            a=float("{:.2f}".format(pos[i]))
            self.outputWidgetStorage[i].value(str(a))
    
    
    
    # add model to opengl window class
    def addModel(self,name,modelId,drawFunction = None,position=(0,0,0),rotation=(0,0,0),obj=None,objType = 'model',listOfJoint=[],showTarget = False,showPole = False):
        print("Add model name",name)
        self.openglWindow.addModel(name,modelId,drawFunction,position,rotation,obj=obj,objType=objType,listOfJoint=listOfJoint,showTarget=showTarget,showPole = showPole)
        print("Done!")
    
    