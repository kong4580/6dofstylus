import numpy as np
import drawFunc
from OpenGL import GL, GLUT, GLU
class Manipulator:
    def __init__(self):
        self.currentM = np.eye(4)
        self.cameravalue = [1,0,0,1,1,1]
        self.w = 600
        self.h = 600
        self.lastwinZ = 0

    def drawManipulator(self,moveMode,dirMode,matrix,selectedMode,camera):
        self.cameravalue = camera
        self.currentM = matrix.copy().T
        if dirMode == False:
            self.currentM[0:3,0:3] = np.eye(3)
        if moveMode == 'trans':
            self.drawFrame(drawFunc.drawAxisX,101,selectedMode)
            self.drawFrame(drawFunc.drawAxisY,102,selectedMode)
            self.drawFrame(drawFunc.drawAxisZ,103,selectedMode)
            self.drawFrame(drawFunc.drawSquareX,301,selectedMode)
            self.drawFrame(drawFunc.drawSquareY,302,selectedMode)
            self.drawFrame(drawFunc.drawSquareZ,303,selectedMode)
        if moveMode == 'rot':
            self.drawFrame(drawFunc.drawCircleX,201,selectedMode)
            self.drawFrame(drawFunc.drawCircleY,202,selectedMode)
            self.drawFrame(drawFunc.drawCircleZ,203,selectedMode)
        
    def drawFrame(self,drawFrameFunc,frameId,selectedMode=False):
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glPushMatrix()
        GL.glLoadIdentity()
        GL.glLoadMatrixf(self.currentM)
        GL.glDisable(GL.GL_LIGHTING)
        if selectedMode:
            GL.glLoadName(frameId)
        ratio = (0.6/self.cameravalue[0])+0.4
        drawFrameFunc(ratio = ratio,selectedMode = selectedMode)
        

        GL.glEnable(GL.GL_LIGHTING)
        GL.glPopMatrix()
        
        