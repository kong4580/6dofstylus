from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import time
import math
RedColorVector = (1,0,0)
GreenColorVector = (0,1,0)
BlueColorVector = (0,0,1)
YellowColorVector = (1,1,0)
SkyColorVector = (0,1,1)
MagentaColorVector = (1,0,1)
WhiteColorVector = (1,1,1)
GrayColorVector = (0.75,0.75,0.75)

BlackColorVector = (0,0,0)





def point():
    glEnable(GL_POINT_SMOOTH)
    glPointSize(10)
    glBegin(GL_POINTS)
    glColor3fv(MagentaColorVector)
    glVertex3f(0, 0, 0)
    glEnd()
def coordinate():
    glColor3f(0.0,1.0,0.0) # green y
    glBegin(GL_LINES)
    # x aix
    glVertex3f(0, 0, 0.0)
    glVertex3f(0.0, 4.0, 0.0)
    glEnd()
    
    # y 
    glColor3f(0.0,0.0,1.0) # blue z
    glBegin(GL_LINES)
    glVertex3f(0.0, 0, 0.0)
    glVertex3f(0.0, 0.0, 4.0)
    glEnd()
 
    # z 
    glColor3f(1.0,0.0,0.0) # red x
    glBegin(GL_LINES)
    glVertex3f(0.0, 0.0 ,0 )
    glVertex3f(4.0, 0.0 ,0.0 )
    glEnd()
def DrawCube():
    glBegin(GL_QUADS)

    glColor3f(1, 0, 0)
    glVertex3f(-0.5, 0.5, 0.5)
    glVertex3f(-0.5, -0.5, 0.5)
    glVertex3f(0.5, -0.5, 0.5)
    glVertex3f(0.5, 0.5, 0.5)

    glColor3f(0, 1, 0)
    glVertex3f(-0.5, 0.5, -0.5)
    glVertex3f(0.5, 0.5, -0.5)
    glVertex3f(0.5, -0.5, -0.5)
    glVertex3f(-0.5, -0.5, -0.5)

    glColor3f(0, 0, 1)
    glVertex3f(-0.5, 0.5, -0.5)
    glVertex3f(-0.5, 0.5, 0.5)
    glVertex3f(0.5, 0.5, 0.5)
    glVertex3f(0.5, 0.5, -0.5)

    glColor3f(1, 1, 0)
    glVertex3f(-0.5, -0.5, -0.5)
    glVertex3f(0.5, -0.5, -0.5)
    glVertex3f(0.5, -0.5, 0.5)
    glVertex3f(-0.5, -0.5, 0.5)

    glColor3f(0, 1, 1)
    glVertex3f(-0.5, 0.5, -0.5)
    glVertex3f(-0.5, -0.5, -0.5)
    glVertex3f(-0.5, -0.5, 0.5)
    glVertex3f(-0.5, 0.5, 0.5)

    glColor3f(1, 0, 1)
    glVertex3f(0.5, 0.5, 0.5)
    glVertex3f(0.5, -0.5, 0.5)
    glVertex3f(0.5, -0.5, -0.5)
    glVertex3f(0.5, 0.5, -0.5)
    glEnd()
def Grid():
    glBegin(GL_LINES)
    glColor3f(1, 1, 1)
    for i in range(-10,10,1):
        glVertex3f(i, 0, 10)
        glVertex3f(i, 0, -10)
        glVertex3f(10, 0, i) 
        glVertex3f(-10, 0, i)
    glEnd()
    
def drawTeapot():
    start = time.time()
    glutWireTeapot(2)
    end = time.time()
    print("time =",end - start)
def drawCircleZ():
    ratio = 5
    num = 600
    glColor3f(0.0,0.0,1.0)
    glLineWidth(10)
    glBegin(GL_LINE_LOOP)
    for i in range(num):
        angle = 2 * math.pi * i / 300
        x = math.cos(angle)*ratio
        y = math.sin(angle)*ratio
        glVertex3d(x,y,0)
    glEnd()
    glLineWidth(1)
def drawCircleX():
    ratio = 5
    num = 600
    glLineWidth(10)
    glColor3f(1.0,0.0,0.0)
    glBegin(GL_LINE_LOOP)
    for i in range(num):
        angle = 2 * math.pi * i / 300
        x = math.cos(angle)*ratio
        y = math.sin(angle)*ratio
        glVertex3d(0,x,y)
    glEnd()
    glLineWidth(1)
def drawCircleY():
    ratio = 5
    num = 600
    glLineWidth(10)
    glColor3f(0.0,1.0,0.0)
    glBegin(GL_LINE_LOOP)
    for i in range(num):
        angle = 2 * math.pi * i / 300
        x = math.cos(angle)*ratio
        y = math.sin(angle)*ratio
        glVertex3d(x,0,y)
    glEnd()
    glLineWidth(1)
