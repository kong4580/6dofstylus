import fltk
import numpy as np
from OpenGL import GL,GLUT
from Model import Model,OBJ
import drawFunc
from StylusReciever import StylusReciever
from scipy.spatial.transform import Rotation as R
import sys
import math
class MyGLWindow( fltk.Fl_Gl_Window ):
    
    def __init__( self, x, y, w, h, l = "" ):
        fltk.Fl_Gl_Window.__init__( self, x, y, w, h, l )
        self.imu = Model('imu',drawFunc.point)
        
        self.teapot = Model('teapot',drawFunc.DrawCube)
        
   
        
    def draw( self ):
        # print( "MyGLWindow::draw()" )
        # set background color to black
        GLUT.glutInit(sys.argv)
        self.lighting()
        GL.glClearColor(0.0, 0.0, 0.0, 0.0)
        
        # set depth to draw front object
        GL.glClearDepth(1.0) 
        GL.glDepthFunc(GL.GL_LESS)
        GL.glEnable(GL.GL_DEPTH_TEST)
        
        # clear color buffer
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        valid = False
        if (not valid) :
            valid = True
            
            # reset matrix to identity
            GL.glLoadIdentity()

            # set new viewport if window resize
            GL.glViewport(0,0,self.w(),self.h())
            
            # minX, maxX, minY, maxY, minZ, maxZ
            GL.glFrustum((-1*(self.w()/self.h())), (1*(self.w()/self.h())), -1, 1, 1, 100)
        # offset camera view
        GL.glTranslatef(-0,0,-10)
        
        # offset camera view down for 5 units
        ### now use ###
        # GL.glTranslatef(0,-5,0)
        GL.glMatrixMode(GL.GL_MODELVIEW)
        
        # reset matrix to identity
        GL.glPushMatrix()
        GL.glLoadIdentity()
        
        newM = np.eye(4)
        newM[0,3] = 8
        newM[1,3] = 1
        newM[2,3] = 3
        self.teapot.drawMatrixModel(newM,showFrame=False)
        GL.glPopMatrix()
        proj = GL.glGetFloatv(GL.GL_PROJECTION_MATRIX).T
        modelview = GL.glGetFloatv(GL.GL_MODELVIEW_MATRIX).T
        # view = GL.glGetFloatv(GL.GL_VIEW_MATRIX).T
        t = np.dot(proj,self.teapot.currentM)
        t = np.dot(t,np.array([[0,0,0,10]]).T)
        # print(np.array([[1,1,1,10]]).T)
        newT = t/t[3,0]
        # print(np.array([self.teapot.currentM[:,3]]).T.shape,proj,t,newT)
        vp = GL.glGetIntegerv(GL.GL_VIEWPORT)
        wx = vp[0]+vp[2]*(newT[0]+1)/2
        wy = vp[1]+vp[3]*(newT[1]+1)/2
        
        print(vp,wx,wy,proj)
        # self.imu.show = True
        # self.imu.drawMatrixModel(newImu,showFrame=True)
        
    def lighting(self):
        
        # enable light mode
        GL.glEnable(GL.GL_LIGHTING)
        GL.glEnable(GL.GL_LIGHT0)
        
        # set light color and ambient
        ambientLight = [0.2, 0.2, 0.2, 1.0 ] # RGB and alpha channels
        diffuseLight = [0.8, 0.8, 0.8, 1.0 ] # RGB and alpha channels
        GL.glLightfv(GL.GL_LIGHT0, GL.GL_AMBIENT, ambientLight)
        GL.glLightfv(GL.GL_LIGHT0, GL.GL_DIFFUSE, diffuseLight)
        GL.glEnable(GL.GL_COLOR_MATERIAL)
        GL.glColorMaterial(GL.GL_FRONT, GL.GL_AMBIENT_AND_DIFFUSE)
        
        # set light source position
        # x, y, z, alpha
        position = [ 10.0, 0.0, -1.0, 1.0 ]
        GL.glLightfv(GL.GL_LIGHT0, GL.GL_POSITION, position)
    def handle(self,event):
        
        # get mouse position
        xMousePosition = fltk.Fl.event_x()
        yMousePosition = fltk.Fl.event_y()
        print(xMousePosition,480-yMousePosition)
        # print(event== fltk.FL_KEYUP)
        # if left mouse button is pressed
        if event == fltk.FL_PUSH and fltk.Fl.event_button() == 1: 
            # print("mouse position:",xMousePosition,yMousePosition)
            return 1
        
        # check keyborad key event
        elif event == fltk. FL_KEYUP: #keyboard handle
            # print("key press : ",chr(fltk.Fl.event_key())) #check key #type:int 
             #check key #type:int 
            
            # toggles show model flags
            if fltk.Fl.event_key() == ord('q'):
                self.isImuInit = True
                print("cal")
                
            if fltk.Fl.event_key() == ord('a'):
                self.add = not self.add
                # self.addAngleMatrix = np.dot(self.addAngleMatrix,self.addAngle(5))
                self.count+=1
                print("cal",self.count)
            if fltk.Fl.event_key() == ord('c'):
                if self.mode == 'relative':
                    self.mode = 'stop'
                else:
                    self.mode = 'relative'
                # self.addAngleMatrix = np.dot(self.addAngleMatrix,self.addAngle(5))
                
                
            if fltk.Fl.event_key() == ord('v'):
                if self.mode == 'absolute':
                    self.mode = 'stop'
                else:
                    self.mode = 'absolute'
                    self.f = True
                
                # self.addAngleMatrix = np.dot(self.addAngleMatrix,self.addAngle(5))
                
            # break condition to run main call back
            fltk.Fl_check()
            return 1
        
        
        # if no event
        else:
            
            # wait for handle
            return fltk.Fl_Gl_Window.handle(self, event)
if __name__ == "__main__":
    
    myGLWindow = MyGLWindow( 0, 0, 720, 480 )
    
    myGLWindow.show()
    
    while myGLWindow.shown():
        
        myGLWindow.redraw()
        fltk.Fl_check()
        
    
    fltk.Fl_run()