from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
def point():
    glEnable(GL_POINT_SMOOTH)
    glPointSize(20)
    glBegin(GL_POINTS)
    glColor3f(1, 1, 1)
    glVertex3f(0, 0, 0)
    glEnd()
def coordinate():
    glColor3f(0.0,1.0,0.0) # green y
    glBegin(GL_LINES)
    # x aix
    glVertex3f(-4.0, 0.0, 0.0)
    glVertex3f(4.0, 0.0, 0.0)
    glEnd()
    
    # y 
    glColor3f(0.0,0.0,1.0) # blue z
    glBegin(GL_LINES)
    glVertex3f(0.0, -4.0, 0.0)
    glVertex3f(0.0, 4.0, 0.0)
    glEnd()
 
    # z 
    glColor3f(1.0,0.0,0.0) # red x
    glBegin(GL_LINES)
    glVertex3f(0.0, 0.0 ,-4.0 )
    glVertex3f(0.0, 0.0 ,4.0 )
    glEnd()
def DrawCube():
    glBegin(GL_QUADS)

    glColor3f(1, 0, 0)
    glVertex3f(-1, 1, 1)
    glVertex3f(-1, -1, 1)
    glVertex3f(1, -1, 1)
    glVertex3f(1, 1, 1)

    glColor3f(0, 1, 0)
    glVertex3f(-1, 1, -1)
    glVertex3f(1, 1, -1)
    glVertex3f(1, -1, -1)
    glVertex3f(-1, -1, -1)

    glColor3f(0, 0, 1)
    glVertex3f(-1, 1, -1)
    glVertex3f(-1, 1, 1)
    glVertex3f(1, 1, 1)
    glVertex3f(1, 1, -1)

    glColor3f(1, 1, 0)
    glVertex3f(-1, -1, -1)
    glVertex3f(1, -1, -1)
    glVertex3f(1, -1, 1)
    glVertex3f(-1, -1, 1)

    glColor3f(0, 1, 1)
    glVertex3f(-1, 1, -1)
    glVertex3f(-1, -1, -1)
    glVertex3f(-1, -1, 1)
    glVertex3f(-1, 1, 1)

    glColor3f(1, 0, 1)
    glVertex3f(1, 1, 1)
    glVertex3f(1, -1, 1)
    glVertex3f(1, -1, -1)
    glVertex3f(1, 1, -1)
    glEnd()
def Grid():
    glBegin(GL_LINES)
    for i in range(-10,10,1):
        glVertex3f(i, 0, 10)
        glVertex3f(i, 0, -10)
        glVertex3f(10, 0, i) 
        glVertex3f(-10, 0, i)
    glEnd()