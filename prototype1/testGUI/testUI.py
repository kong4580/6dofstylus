from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
# from PIL.Image import *
from fltk import *
import sys
import math
from drawFunc import *
from functools import partial
# def point():
#     glEnable(GL_POINT_SMOOTH)
#     glPointSize(20)
#     glBegin(GL_POINTS)
#     glColor3f(1, 1, 1)
#     glVertex3f(0, 0, 0)
#     glEnd()a = a-1
def slider_cb(widget, v):
    global pos, drawing
    if type(widget.value()) == str:
        a = float(widget.value())
    else:
        a = widget.value()
    pos[v] = a
    # print(opengl.pose())
    # print("p")
    # a=float("{:.2f}".format(pos[v]))
    # q.value(str(a))
    # a = widget.value()
    # print(pos[0],pos[1],v)
    opengl.redraw()
def sliderScaleCB(widget,v):
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
# def coordinate():
    
#     glColor3f(0.0,1.0,0.0) # green y
#     glBegin(GL_LINES)
#     # x aix
#     glVertex3f(-4.0, 0.0, 0.0)
#     glVertex3f(4.0, 0.0, 0.0)
#     glEnd()
#     # y 
#     glColor3f(0.0,0.0,1.0) # blue z
#     glBegin(GL_LINES)
#     glVertex3f(0.0, -4.0, 0.0)
#     glVertex3f(0.0, 4.0, 0.0)
#     glEnd()
 
#     # z 
    
#     glColor3f(1.0,0.0,0.0) # red x
#     glBegin(GL_LINES)
#     glVertex3f(0.0, 0.0 ,-4.0 )
#     glVertex3f(0.0, 0.0 ,4.0 )
#     glEnd()
# def DrawCube():
#     glBegin(GL_QUADS)

#     glColor3f(1, 0, 0)
#     glVertex3f(-1, 1, 1)
#     glVertex3f(-1, -1, 1)
#     glVertex3f(1, -1, 1)
#     glVertex3f(1, 1, 1)

#     glColor3f(0, 1, 0)
#     glVertex3f(-1, 1, -1)
#     glVertex3f(1, 1, -1)
#     glVertex3f(1, -1, -1)
#     glVertex3f(-1, -1, -1)

#     glColor3f(0, 0, 1)
#     glVertex3f(-1, 1, -1)
#     glVertex3f(-1, 1, 1)
#     glVertex3f(1, 1, 1)
#     glVertex3f(1, 1, -1)

#     glColor3f(1, 1, 0)
#     glVertex3f(-1, -1, -1)
#     glVertex3f(1, -1, -1)
#     glVertex3f(1, -1, 1)
#     glVertex3f(-1, -1, 1)

#     glColor3f(0, 1, 1)
#     glVertex3f(-1, 1, -1)
#     glVertex3f(-1, -1, -1)
#     glVertex3f(-1, -1, 1)
#     glVertex3f(-1, 1, 1)

#     glColor3f(1, 0, 1)
#     glVertex3f(1, 1, 1)
#     glVertex3f(1, -1, 1)
#     glVertex3f(1, -1, -1)
#     glVertex3f(1, 1, -1)
#     glEnd()

# def Grid():
#     glBegin(GL_LINES)
#     for i in range(-10,10,1):
#         glVertex3f(i, 0, 10)
#         glVertex3f(i, 0, -10)
#         glVertex3f(10, 0, i) 
#         glVertex3f(-10, 0, i)
#     glEnd()
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
        
 
def DrawGLScene(Filename):
    # pass
    size = loadImage(Filename)
    # print(size)
    size[0] = size[0]/10
    size[1] = size[1]/10
    # glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glLoadIdentity()
    glTranslatef(0.0,0.0,-10.0)
    glEnable(GL_TEXTURE_2D)
    glBegin(GL_QUADS)
    glColor3f(1,0,0)
    # glTexCoord2f(0.0, 0.0)
    glVertex3f(-size[0]/2,-size[1]/2,-1)
    glTexCoord2f(0.0, 0.0)
    # glTexCoord2f(1.0, 0.0)
    glVertex3f(size[0]/2,-size[1]/2,-1)
    glTexCoord2f(1.0, 0.0)
    # glTexCoord2f(1.0, 1.0)
    glVertex3f(size[0]/2,size[1]/2,-1)
    glTexCoord2f(1.0, 1.0)
    # glTexCoord2f(0.0, 1.0)
    glVertex3f(-size[0]/2,size[1]/2,-1)
    glTexCoord2f(0.0, 1.0)
    glEnd()
    # glutSwapBuffers()
    glDisable(GL_TEXTURE_2D)
class MyWindow(Fl_Gl_Window):
    def __init__(self, xpos, ypos, width, height, label):
        Fl_Gl_Window.__init__(self, xpos, ypos, width, height, label)
        glColor3f(1.0, 1.0, 1.0)
        self.pose = [0,0,0,0,0,0]
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
            firstTime = False                                    # ondraw is called all the time
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glFrustum(-1, 1, -1, 1, 1, 100)
        glTranslatef(self.scaleY,self.scaleZ,self.scaleX)
        # DrawCube()
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        # glTranslatef(0,0,0)
        gluLookAt(0, 0, 1, 0, 0, 0, 0, 1, 0)
        glPushMatrix()
        Grid()
        glPopMatrix()
        glPushMatrix()
        # glTranslatef(self.pose[1],self.pose[2],self.pose[0])
        glRotatef(self.pose[5],0,1,0) #y#z
        glRotatef(self.pose[3],0,0,1) #z#x
        glRotatef(self.pose[4],1,0,0) #x#y
        # glTranslatef(-self.pose[1],-self.pose[2],-self.pose[0])
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
window = Fl_Window(800,600,"UI")
opengl = MyWindow(0,0,600,600,'opengl')
def createOutputWidget():
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
        slider.callback(sliderScaleCB, n)
    opengl.readScaleX(-10)
    opengl.readScaleY(0)
    opengl.readScaleZ(0)
    return storage_area
def updateSlider(storage,pos):
    for i in range(6):
        a=float("{:.2f}".format(pos[i]))
        storage[i].value(str(a))
def UI(arg):
    global storage
    pos = change(arg)
    opengl.readPose(pos)
    opengl.redraw()
    updateSlider(storage,pos)
# y = 0
# for n in range(6) :
#     slider = Fl_Hor_Value_Slider(655, y, 145,25,name[n])
#     y = y + 25
#     if n < 3:
#         slider.minimum(-10)
#         slider.maximum(10)
#     elif 3<=n and n<6:
#         # pass
#         slider.minimum(-180)
#         slider.maximum(180)
#     slider.step(1)
#     slider.align(FL_ALIGN_LEFT)
#     slider.callback(slider_cb, n)
#     slider.value(0)
#     # store s, so that the s widget is not deleted on next loop
#     storage_area.append(slider)

    # window.redraw()
    # window.show()
    # Fl.run()
def change(arg):
    inputArg = arg.copy()
    # inputArg[0] = arg[0] + 3.141592653589793
    # inputArg[1] = arg[1] - 1.5707963057214722
    # inputArg[2] = arg[2] - 0.0
    # inputArg[3] = arg[3] + 61.5
    # inputArg[4] = arg[4] - 0.0
    # inputArg[5] = arg[5] + 26.5
    real = [0]*9
    for i in range(len(arg)):
        if i <3:
            # arg[i] = ((arg[i])*360)/(2*math.pi)
            real[i+3] = ((inputArg[i])*360)/(2*math.pi)
            # print(real)
        else:
            # arg[i] = arg[i]/10

            real[i-3] = inputArg[i]/20
            # print(real)
    # print(arg,"arg")
    print(real)
    return real
datas = [[0, 0, 0, 111.18420055965112, -12.977730802458181, 35.252358895392774],
        [math.pi/2, 0, 0, 111.18705790994423, -12.804297143508233, 5.192916787841966],
        [0.1328845675511949, -0.31123808064600844, 2.5919708990192887, 111.26625842527096, -11.893111936910449, 33.13759045287591],
        [0.13883531311823802, -0.3085955522946684, 2.604547581533135, 110.70017722872467, -11.760976828953925, 32.51728920171727],
        [0.12954513280070573, -0.31778065822081913, 2.5989897249489995, 109.83520293400512, -10.400061076477893, 31.968193212634503],
        [0.1606048613894023, -0.3077308914616088, 2.635539435312429, 108.9587843582027, -11.717226532915525, 31.702686139101367],
        [0.1251624139956076, -0.32648260454235456, 2.605417020802588, 108.56266239091954, -8.658053780329524, 31.339627035425018]]
count = 0
def send_data():
    global datas
    global count
    if count == 0:
        count = 1
    else:
        count = 0
    print('data',datas[count])
    return(datas[count])
def callback():
    print("TICK")
    UI(send_data())
    Fl_repeat_timeout(1.0,callback)

def main():
    global storage
    # win = Fl_Window(720,486)
    # out = Fl_Output(100,100,100,50,"out") 
    storage = createOutputWidget()
    window.show()
    Fl_add_timeout(1.0,callback)
    return Fl_run()
main()