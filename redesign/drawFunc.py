import time
import math
from OpenGL import GL,GLUT,GLU
RedColorVector = (1,0,0)
GreenColorVector = (0,1,0)
BlueColorVector = (0,0,1)
YellowColorVector = (1,1,0)
SkyColorVector = (0,1,1)
MagentaColorVector = (1,0,1)
WhiteColorVector = (1,1,1)
GrayColorVector = (0.75,0.75,0.75)

BlackColorVector = (0,0,0)

# floor grid
def Grid():
    GL.glBegin(GL.GL_LINES)
    GL.glColor3f(1, 1, 1)
    for i in range(-20,20,1):
        GL.glVertex3f(i, 0, 100)
        GL.glVertex3f(i, 0, -100)
        GL.glVertex3f(100, 0, i) 
        GL.glVertex3f(-100, 0, i)
    GL.glEnd()

# point
def point():
    GL.glEnable(GL.GL_POINT_SMOOTH)
    GL.glPointSize(10)
    GL.glBegin(GL.GL_POINTS)
    GL.glColor3fv(MagentaColorVector)
    GL.glVertex3f(0, 0, 0)
    GL.glEnd()

# cursor model
def drawCursor():
    ratio = 0.75
    GL.glColor3f(1,0,1)
    r  = 0.3*ratio
    height = (r/3) *10
    sradius = r/3
    GLUT.glutSolidSphere(r,50,50)
    GL.glTranslatef(0,height,0)
    GL.glRotatef(90,1,0,0)
    GLUT.glutSolidCylinder(sradius,height,10,10)
    GLUT.glutSolidSphere(sradius,50,50)

# X translation axis
def drawAxisX(**kwargs):
    ratio = kwargs['ratio']
    selectedMode = kwargs['selectedMode']
    GL.glColor3f(1.0,0.0,0.0) # red x
    length = 4*ratio
    radius = 0.2*ratio
    cyradius = 0.1*ratio
    
    # in select mode
    if selectedMode:
        cyradius = 0.25*ratio
    
    GL.glRotatef(90,0,1,0)
    GL.glTranslatef(0,0,length)
    GLUT.glutSolidCone(radius,length/5,10,10)
    GL.glTranslatef(0,0,-length)
    GLUT.glutSolidCylinder(cyradius/2,length,10,10)

# Y translation axis
def drawAxisY(**kwargs):
    ratio = kwargs['ratio']
    selectedMode = kwargs['selectedMode']
    GL.glColor3f(0.0,1.0,0.0) # green y
    length = 4*ratio
    radius = 0.2 * ratio
    cyradius = 0.1*ratio

    # in select mode
    if selectedMode:
        cyradius = 0.25*ratio

    GL.glRotatef(-90,1,0,0)
    GL.glTranslatef(0,0,length)
    GLUT.glutSolidCone(radius,length/5,10,10)
    GL.glTranslatef(0,0,-length)
    GLUT.glutSolidCylinder(cyradius/2,length,10,10)

# Z translation axis    
def drawAxisZ(**kwargs):
    ratio = kwargs['ratio']
    selectedMode = kwargs['selectedMode']
    GL.glColor3f(0.0,0.0,1.0) # blue z
    length = 4*ratio
    radius = 0.2*ratio
    cyradius = 0.1*ratio

    #in select mode
    if selectedMode:
        cyradius = 0.25*ratio

    GL.glRotatef(90,0,0,1)
    GL.glTranslatef(0,0,length)
    GLUT.glutSolidCone(radius,length/5,10,10)
    GL.glTranslatef(0,0,-length)
    GLUT.glutSolidCylinder(cyradius/2,length,10,10)

# X rotation axis
def drawCircleX(**kwargs):
    ratio = kwargs['ratio']
    selectedMode = kwargs['selectedMode']
    GL.glColor3f(1.0,0.0,0.0)
    GL.glRotatef(90,0,1,0)
    inner = 0.05*ratio
    outer = 4*ratio

    # in select mode
    if selectedMode:
        inner = 0.1*ratio

    GLUT.glutSolidTorus(inner,outer,10,100)

# y rotation axis
def drawCircleY(**kwargs):
    ratio = kwargs['ratio']
    selectedMode = kwargs['selectedMode']
    GL.glColor3f(0.0,1.0,0.0)
    GL.glRotatef(90,1,0,0)
    inner = 0.05*ratio
    outer = 4*ratio

    # in select mode
    if selectedMode:
        inner = 0.1*ratio

    GLUT.glutSolidTorus(inner,outer,10,100)

# Z rotation axis
def drawCircleZ(**kwargs):
    ratio = kwargs['ratio']
    selectedMode = kwargs['selectedMode']
    GL.glColor3f(0.0,0.0,1.0)
    inner = 0.05*ratio
    outer = 4*ratio

    # in select mode
    if selectedMode:
        inner = 0.1*ratio

    GLUT.glutSolidTorus(inner,outer,10,100)

# 2d hexagon for base
def drawHex(ratio,selectedMode):
     # wire hexagon
    vertex = []
    for i in range(6):
        v = [ratio*math.sin(i/6.0*2*math.pi),ratio*math.cos(i/6.0*2*math.pi)]
        vertex.append(v)
    GL.glBegin(GL.GL_LINES)
    GL.glVertex2d(vertex[0][0], vertex[0][1])
    GL.glVertex2d(vertex[1][0], vertex[1][1])
    GL.glEnd()
    GL.glBegin(GL.GL_LINES)
    GL.glVertex2d(vertex[1][0], vertex[1][1])
    GL.glVertex2d(vertex[2][0], vertex[2][1])
    GL.glEnd()
    GL.glBegin(GL.GL_LINES)
    GL.glVertex2d(vertex[2][0], vertex[2][1])
    GL.glVertex2d(vertex[3][0], vertex[3][1])
    GL.glEnd()
    GL.glBegin(GL.GL_LINES)
    GL.glVertex2d(vertex[3][0], vertex[3][1])
    GL.glVertex2d(vertex[4][0], vertex[4][1])
    GL.glEnd()
    GL.glBegin(GL.GL_LINES)
    GL.glVertex2d(vertex[4][0], vertex[4][1])
    GL.glVertex2d(vertex[5][0], vertex[5][1])
    GL.glEnd()
    GL.glBegin(GL.GL_LINES)
    GL.glVertex2d(vertex[5][0], vertex[5][1])
    GL.glVertex2d(vertex[0][0], vertex[0][1])
    GL.glEnd()

    # transparent model
    GL.glPushAttrib(GL.GL_COLOR_BUFFER_BIT)
    GL.glEnable(GL.GL_BLEND)
    GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)

    # in select mode
    if selectedMode:

        # filled hexagon
        GL.glBegin(GL.GL_POLYGON)
        for i in range(6):
            GL.glVertex2d(ratio*math.sin(i/6.0*2*math.pi),ratio*math.cos(i/6.0*2*math.pi))
        GL.glEnd()

        # cube for better base selection
        GLUT.glutSolidCube(1.5)

    GL.glDisable(GL.GL_BLEND)
    GL.glPopAttrib()
# 2d circle for base
def drawBaseCircle(ratio,selectedMode):
     
    # wire circle 
    GL.glBegin(GL.GL_LINE_LOOP)
    for i in range(600):
        angle = 2 * math.pi * i / 300
        x = math.cos(angle)*ratio
        y = math.sin(angle)*ratio
        GL.glVertex2d(x,y)
    GL.glEnd()

    # transparent model
    GL.glPushAttrib(GL.GL_COLOR_BUFFER_BIT)
    GL.glEnable(GL.GL_BLEND)
    GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)

    #in select mode
    if selectedMode:

        # filled circle
        GL.glBegin(GL.GL_POLYGON)
        for i in range(100):
            GL.glVertex2d(ratio*math.sin(i/100*2*math.pi),ratio*math.cos(i/100*2*math.pi))
        GL.glEnd()

    GL.glDisable(GL.GL_BLEND)
    GL.glPopAttrib()

# base model
def drawBase(**kwargs):
    ratio = kwargs['ratio']
    selectedMode = kwargs['selectedMode']
    GL.glRotatef(90,1,0,0)
    drawBaseCircle(ratio*2,selectedMode)
    drawHex(ratio*4,selectedMode)

# 2d circle
def drawCircle(ratio):
    GL.glBegin(GL.GL_LINE_LOOP)
    for i in range(600):
        angle = 2 * math.pi * i / 300
        x = math.cos(angle)*ratio
        y = math.sin(angle)*ratio
        GL.glVertex2d(x,y)
    GL.glEnd()

# pole model
def drawPole(**kwargs):
    ratio = kwargs['ratio']
    selectedMode = kwargs['selectedMode']
    GL.glLineWidth(2)
    drawCircle(ratio)
    GL.glRotatef(90,1,0,0)
    drawCircle(ratio)
    GL.glRotatef(90,0,1,0)
    drawCircle(ratio)
    GL.glLineWidth(1)

    # transparent model
    GL.glPushAttrib(GL.GL_COLOR_BUFFER_BIT)
    GL.glEnable(GL.GL_BLEND)
    GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)

    # in select mode
    if selectedMode:
        GLUT.glutSolidSphere(ratio,10,10)

    GL.glDisable(GL.GL_BLEND)
    GL.glPopAttrib()
    GL.glTranslatef(0,-ratio/2,0)

# target model    
def drawTarget(**kwargs):
    ratio = kwargs['ratio'] * 0.8
    selectedMode = kwargs['selectedMode']
    GL.glLineWidth(2)
    GL.glRotatef(90,0,1,0)
    GL.glScalef(ratio,ratio,ratio)
    GLUT.glutWireDodecahedron()
    GL.glLineWidth(1)

    # transparent model
    GL.glPushAttrib(GL.GL_COLOR_BUFFER_BIT)
    GL.glEnable(GL.GL_BLEND)
    GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)

    # in select mode
    if selectedMode:
        GLUT.glutSolidDodecahedron()

    GL.glDisable(GL.GL_BLEND)
    GL.glPopAttrib()

# 2d filled square
def drawSqure(size):
    GL.glBegin(GL.GL_POLYGON)
    GL.glVertex2d( size,  size)
    GL.glVertex2d( size,  - size)
    GL.glVertex2d( - size,  - size)
    GL.glVertex2d( - size,  size)
    GL.glEnd()
    
# YZ translate controller
def drawSquareY(**kwargs):
    ratio = kwargs['ratio']
    selectedMode = kwargs['selectedMode']
    GL.glColor3f(0.0,1.0,0.0)
    length = 2* ratio
    GL.glTranslatef(0,length,length)
    GL.glRotatef(90,0,1,0)
    halfside = ratio*0.5
    drawSqure(halfside)

# XZ translate controller
def drawSquareZ(**kwargs):
    ratio = kwargs['ratio']
    selectedMode = kwargs['selectedMode']
    GL.glColor3f(0.0,0.0,1.0)
    length = 2* ratio
    GL.glTranslatef(length,0,length)
    GL.glRotatef(-90,1,0,0)
    halfside = ratio*0.5
    drawSqure(halfside)

# XY translate controller
def drawSquareX(**kwargs):
    ratio = kwargs['ratio']
    selectedMode = kwargs['selectedMode']
    GL.glColor3f(1.0,0.0,0.0)
    length = 2* ratio
    GL.glTranslatef(length,length,0)
    GL.glRotatef(90,0,0,1)
    halfside = ratio*0.5
    drawSqure(halfside)

def coordinate():
    
    GL.glColor3f(0.0,1.0,0.0) # green y
    GL.glBegin(GL.GL_LINES)
    # x aix
    GL.glVertex3f(0, 0, 0.0)
    GL.glVertex3f(0.0, 4.0, 0.0)
    GL.glEnd()
    
    
    # y 
    GL.glColor3f(0.0,0.0,1.0) # blue z
    GL.glBegin(GL.GL_LINES)
    GL.glVertex3f(0.0, 0, 0.0)
    GL.glVertex3f(0.0, 0.0, 4.0)
    GL.glEnd()
 
    # z 
    GL.glColor3f(1.0,0.0,0.0) # red x
    GL.glBegin(GL.GL_LINES)
    GL.glVertex3f(0.0, 0.0 ,0 )
    GL.glVertex3f(4.0, 0.0 ,0.0 )
    GL.glEnd()
    GL.glLineWidth(1)