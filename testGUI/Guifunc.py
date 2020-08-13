from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
# from PIL.Image import *
from fltk import *
import sys
import math
from drawFunc import *
from functools import partial

def sliderScaleCB(opengl,widget,v):
    a = widget.value()
    size = (a/10)
    if v == 0:
        size = size -10
        opengl.readScaleX(size)
    elif v == 1:
        opengl.readScaleY(size)
    else:
        opengl.readScaleZ(size)
    opengl.redraw()
def InitGL(): 
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0) 
    glDepthFunc(GL_LESS)
    glEnable(GL_DEPTH_TEST)
    glShadeModel(GL_SMOOTH)   
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    # gluPerspective(45.0, float(Width)/float(Height), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
	# initialize texture mapping
    glEnable(GL_TEXTURE_2D)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)
    
class OpenGLWindow(Fl_Gl_Window):
    def __init__(self, xpos, ypos, width, height, label):
        Fl_Gl_Window.__init__(self, xpos, ypos, width, height, label)
        glColor3f(1.0, 1.0, 1.0)
        self.pose = [0,0,0,0,0,0,0,0,0]
    def readPose(self,pose):
        self.pose = pose
    def readScaleX(self,scaleX):
        self.scaleX = scaleX
    def readScaleY(self,scaleY):
        self.scaleY = scaleY
    def readScaleZ(self,scaleZ):
        self.scaleZ = scaleZ
    def draw(self):
        firstTime = True
        if (firstTime):
            InitGL()
            firstTime = False # ondraw is called all the time
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glFrustum(-1, 1, -1, 1, 1, 100)
        glTranslatef(self.scaleY,self.scaleZ,self.scaleX)
        # DrawCube()
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        # glTranslatef(0,0,0)
        gluLookAt(0, 0, 10, 0, 0, 0, 0, 1, 0)
        glPushMatrix()
        Grid()
        glPopMatrix()
        glPushMatrix()
        glTranslatef(self.pose[1],self.pose[2],self.pose[0])
        glRotatef(self.pose[5],0,1,0) #y#z
        glRotatef(self.pose[3],0,0,1) #z#x
        glRotatef(self.pose[4],1,0,0) #x#y
        glTranslatef(-self.pose[1],-self.pose[2],-self.pose[0])
        glTranslatef(self.pose[1],self.pose[2],self.pose[0])
        DrawCube()
        coordinate()
        # glLoadIdentity()
        glPopMatrix()
        glPushMatrix()
        glTranslatef(self.pose[6],self.pose[7],self.pose[8])
        coordinate()
        point()
        glPopMatrix()
        
def change(arg,scale):
    inputArg = arg.copy()
    # set home position
    
    # inputArg[0] = inputArg[0] + 3.141592653589793
    # inputArg[1] = inputArg[1] - 1.5707963057214722
    # inputArg[2] = inputArg[2] - 0.0
    # inputArg[3] = inputArg[3] - 61.5
    # inputArg[4] = inputArg[4] - 0.0
    # inputArg[5] = inputArg[5] + 26.5
    
    real = [0]*9
    for i in range(len(inputArg)):
        if i <3:
            # arg[i] = ((arg[i])*360)/(2*math.pi)
            real[i+3] = ((inputArg[i])*360)/(2*math.pi)
            # print(real)
        else:
            # arg[i] = arg[i]/10

            real[i-3] = inputArg[i]/scale
            # print(real)
    print(real)
    # print(arg,"arg")
    real[0] -= 5.231761051519499
    real[1] -= -0.2290998697402224
    real[2] -= -1.8696414028770085
    
    return real

def updateSlider(storage,pos):
    for i in range(6):
        a=float("{:.2f}".format(pos[i]))
        storage[i].value(str(a))
        
def createOutputWidget(opengl):
    storage_area = []
    name=["TransX", "TransY", "TransZ", "RotX", "RotY", "RotZ","OriX","OriY","OriZ"]
    y=0
    for n in range(6):
        output = Fl_Output(660,y,60,25,name[n])
        y = y + 25
        output.align(FL_ALIGN_LEFT)
        output.value("0")
        storage_area.append(output)
    y=150
    name_Scale = ["ScaleX","ScaleY","ScaleZ"]
    for n in range(3):
        slider = Fl_Hor_Value_Slider(655, y, 145,25,name_Scale[n])
        y = y + 25
        slider.minimum(-100)
        slider.maximum(100)
        slider.value(0)
        slider.align(FL_ALIGN_LEFT)
        sldCallback = partial(sliderScaleCB,opengl)
        slider.callback(sldCallback, n)
    opengl.readScaleX(-10)
    opengl.readScaleY(0)
    opengl.readScaleZ(0)
    return storage_area

def updateUI(pose,opengl,storage,buttonStates,scale=20):
    pos = change(pose,scale)
    
    opengl.readPose(pos)
    opengl.redraw()
    updateSlider(storage,pos)
   
# def isModelSelected(mousePose,model,buttonStates):
    
