from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import time
import math
from OpenGL import GL
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

def drawCursor(ratio):
    glColor3f(1,0,1)
    r  = 0.3*ratio
    height = (r/3) *10
    sradius = r/3
    glutSolidSphere(r,50,50)
    glTranslatef(0,height,0)
    glRotatef(90,1,0,0)
    glutSolidCylinder(sradius,height,10,10)
    glutSolidSphere(sradius,50,50)

def drawAxisY():
    glColor3f(0.0,1.0,0.0) # green y
    length = 4
    radius = 0.25
    glRotatef(-90,1,0,0)
    glTranslatef(0,0,length)
    # glTranslatef(-radius,0,0)
    glutSolidCone(radius,length/4,10,10)
    glTranslatef(0,0,-length)
    glutSolidCylinder(radius/2,length,10,10)
    

def drawAxisZ():
    glColor3f(0.0,0.0,1.0) # blue z
    length = 4
    radius = 0.25
    # glTranslatef(-radius,0,0)
    glRotatef(90,0,0,1)
    glTranslatef(0,0,length)
    glutSolidCone(radius,length/4,10,10)
    glTranslatef(0,0,-length)
    glutSolidCylinder(radius/2,length,10,10)

def drawAxisX():
    glColor3f(1.0,0.0,0.0) # red x
    length = 4
    radius = 0.25
    # glTranslatef(-radius,0,0)
    glRotatef(90,0,1,0)
    glTranslatef(0,0,length)
    glutSolidCone(radius,length/4,10,10)
    glTranslatef(0,0,-length)
    glutSolidCylinder(radius/2,length,10,10)

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
    glColor3f(0.0,0.0,1.0)
    inner = 0.1
    outer = 5
    glutSolidTorus(inner,outer,10,100)
def drawCircleX():
    glColor3f(1.0,0.0,0.0)
    glRotatef(90,0,1,0)
    inner = 0.1
    outer = 5
    glutSolidTorus(inner,outer,10,100)
def drawCircleY():
    glColor3f(0.0,1.0,0.0)
    glRotatef(90,1,0,0)
    inner = 0.1
    outer = 5
    glutSolidTorus(inner,outer,10,100)
def drawPyramid(length,modelId):
    scale = 1
    bl = (-length*1*scale,0.5*scale,0.5*scale)
    br = (-length*1*scale,0.5*scale,-0.5*scale)
    fl = (-length*1*scale,-0.5*scale,0.5*scale)
    fr = (-length*1*scale,-0.5*scale,-0.5*scale)
    t = (0,0,0)
    GL.glColor4f(1,1,1,1)
    
    GL.glPushMatrix()
    
    if modelId != None:
        GL.glLoadName(modelId)
    glutSolidSphere(.74,10,10)
    GL.glPopMatrix()
    
    GL.glPushMatrix()
    GL.glBegin(GL.GL_TRIANGLES)
    GL.glVertex3fv( fr )
    GL.glVertex3fv(br )
    GL.glVertex3fv( bl )
    GL.glEnd()
    
    GL.glBegin(GL.GL_TRIANGLES)
    GL.glVertex3fv( fl )
    
    GL.glVertex3fv( fr )
    GL.glVertex3fv( bl )
    GL.glEnd()
    
    GL.glBegin(GL.GL_TRIANGLES)
    GL.glVertex3fv( t )
    
    GL.glVertex3fv( bl )
    GL.glVertex3fv( br )
    GL.glEnd()

    GL.glBegin(GL.GL_TRIANGLES)
    GL.glVertex3fv( t )
    
    GL.glVertex3fv( br )
    GL.glVertex3fv( fr )
    GL.glEnd()
    
    GL.glBegin(GL.GL_TRIANGLES)
    GL.glVertex3fv( t )
    
    GL.glVertex3fv( fr )
    GL.glVertex3fv( fl )
    GL.glEnd()
    
    GL.glBegin(GL.GL_TRIANGLES)
    GL.glVertex3fv( t )
    
    GL.glVertex3fv( fl )
    GL.glVertex3fv( bl )
    GL.glEnd()
    GL.glPopMatrix()
    