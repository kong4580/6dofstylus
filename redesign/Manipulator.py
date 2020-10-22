import numpy as np
import drawFunc
from OpenGL import GL, GLUT, GLU
class Manipulator:
    def __init__(self):
        self.currentM = np.eye(4)
    def drawManipulator(self,moveMode,dirMode,matrix,selectedMode):
        self.currentM = matrix.copy().T
        if dirMode == False:
            self.currentM[0:3,0:3] = np.eye(3)
        if moveMode == 'trans':
            self.drawFrame(drawFunc.drawAxisX,101,selectedMode)
            self.drawFrame(drawFunc.drawAxisY,102,selectedMode)
            self.drawFrame(drawFunc.drawAxisZ,103,selectedMode)
        if moveMode == 'rot':
            self.drawFrame(drawFunc.drawCircleX,201,selectedMode)

            self.drawFrame(drawFunc.drawCircleY,202,selectedMode)

            self.drawFrame(drawFunc.drawCircleZ,203,selectedMode)
    def drawFrame(self,drawFrameFunc,frameId,selectedMode=False):
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glPushMatrix()
        GL.glLoadIdentity()
        
        # apply transform to model
        GL.glLoadMatrixf(self.currentM)
        GL.glDisable(GL.GL_LIGHTING)
        if selectedMode:
            GL.glLoadName(frameId)
        drawFrameFunc()

        GL.glEnable(GL.GL_LIGHTING)
        GL.glPopMatrix()