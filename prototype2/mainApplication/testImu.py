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
        self.imuTrans = np.eye(4)
        self.hBaseToWorld = np.eye(4)
        self.isImuInit = False
        self.addAngleMatrix = np.eye(4)
        self.add = False
        self.count = 0
        self.imuinit=False
        self.axisRotation = (None, None, None)
        self.stopAdd = None
        self.currentModel = np.eye(4)
        self.mode = 'absolute'
        self.f = True
    def initImu(self,quat):
        h = np.eye(4)
        r = R.from_quat(quat)
        h[0:3,0:3] = r.as_matrix()
        self.hBaseToWorld = np.linalg.inv(h.copy())
    def readImuQuat(self,quat):
        if not self.isImuInit:
            self.initImu(quat)
        else:
            
            hWorldToImu = np.eye(4)
            r = R.from_quat(quat)
            hWorldToImu[0:3,0:3] = r.as_matrix()
            self.imuTrans = np.dot(self.hBaseToWorld,hWorldToImu)
    def addAngle(self,angle):
        angle = angle*math.pi/180
        m = self.currentImu[0:3,0:3].copy()
        # m = np.array([[1,0,0],
        #               [0,0,-1],
        #               [0,1,0]])
                      
        dx = (m[2,1]-m[1,2])
        dy = (m[0,2]-m[2,0])
        dz = (m[1,0]-m[0,1])
        x = (dx)/(math.sqrt((dx)**2+dy**2+dz**2))
        y = (dy)/(math.sqrt((dx)**2+dy**2+dz**2))
        z = (dz)/(math.sqrt((dx)**2+dy**2+dz**2))
        
        c = math.cos(angle)
        s = math.sin(angle)
        t = 1-c
        
        newR = np.array([[t*x*x+c, t*x*y - z*s,t*x*z+y*s,0],
                         [t*x*y+z*s,t*y*y+c,t*y*z-x*s,0],
                         [t*x*z-y*s,t*y*z+x*s,t*z*z+c,0],
                         [0,0,0,1]])
        m = self.addAngleMatrix
        print(((m[0,0]+m[1,1]+m[2,2])/2))
        print(math.acos(((m[0,0]+m[1,1]+m[2,2])/2))*180/math.pi)
        return newR
    def getAxis(self,currentM):
        hC0Cn = np.dot(np.linalg.inv(self.preT),currentM)
        
        # angle = angle*math.pi/180
        m = hC0Cn[0:3,0:3].copy()
        # m = np.array([[1,0,0],
        #               [0,0,-1],
        #               [0,1,0]])
                      
        dx = (m[2,1]-m[1,2])
        dy = (m[0,2]-m[2,0])
        dz = (m[1,0]-m[0,1])
        px = (m[2,1]+m[1,2])
        py = (m[0,2]+m[2,0])
        pz = (m[1,0]+m[0,1])
        # 
        # SINGULARLITY CHECK
        # 
        
        ep1 = 0.01
        ep2 = 0.1
        
        mag = math.sqrt((dx)**2+dy**2+dz**2)
        if mag <ep1:
            mag = 1
        x = (dx)/mag
        y = (dy)/mag
        z = (dz)/mag
        if abs(dx)<ep1 and abs(dy)<ep1 and abs(dz)<ep1:
            if abs(px)<ep2 and abs(py)<ep2 and abs(pz)<ep2 and m[0,0]+m[1,1]+m[2,2] < ep2:
                return 0,1,0,0
            angle = math.pi
            xx = (m[0,0]+1)/2
            yy = (m[1,1]+1)/2
            zz = (m[2,2]+1)/2
            xy = pz/4
            xz = py/4
            yz = px/4
            
            if(xx>yy and xx>zz):
                if xx<ep1:
                    x = 0
                    y = 0.7071
                    z = 0.7071
                else:
                    x = math.sqrt(xx)
                    y = xy/x
                    z = xz/x
            elif(yy>zz):
                if yy<ep1:
                    x = 0.7071
                    y = 0
                    z = 0.7071
                else:
                    x = xy/y
                    y = math.sqrt(yy)
                    z = yz/y
            else:
                if zz<ep1:
                    x = 0.7071
                    y = 0.7071
                    z = 0
                else:
                    x = xz/z
                    y = yz/z
                    z = math.sqrt(zz)
            return 180,x,y,z
        
        
        # c = math.cos(angle)
        # s = math.sin(angle)
        # t = 1-c
        
        # newR = np.array([[t*x*x+c, t*x*y - z*s,t*x*z+y*s,0],
        #                  [t*x*y+z*s,t*y*y+c,t*y*z-x*s,0],
        #                  [t*x*z-y*s,t*y*z+x*s,t*z*z+c,0],
        #                  [0,0,0,1]])
        # m = self.addAngleMatrix
        # print(((m[0,0]+m[1,1]+m[2,2])/2))
        try:
            angle = math.acos(((m[0,0]+m[1,1]+m[2,2]-1)/2))*180/math.pi
            print(angle,mag)
            # if mag <= 0.1:
                
            #     return mag,-x,-y,-z
            # print(math.acos(-1)*180/math.pi)
            print(x,y,z)
            return angle,x,y,z
        except:
            
            # return -x,-y,-z
            pass
        
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
        
        if self.isImuInit:
            transform = np.array([[1,0,0,0],
                                [0,0,1,0],
                                [0,-1,0,0],
                                [0,0,0,1]])
            # print(self.imuTrans)
            newImu = np.dot(transform,self.imuTrans)
            newImu = np.dot(newImu,transform.T)
            # newImu = self.imuTrans
            
            if not self.imuinit:
                self.preT = newImu.copy()
                
                self.imuinit = True
                self.currentImu = newImu
                # self.currentModel = newImu
            else:
                self.currentImu = newImu
                ang,a,b,c = self.getAxis(self.currentImu)
                print(ang)
                if self.mode == 'relative':
                    # self.f = True
                    if ang>30 and ang!=180:
                        # ang,a,b,c = self.getAxis(self.imu.currentM)
                        print(ang)
                        # if self.axisRotation ==(None,None,None):
                        self.axisRotation =(a,b,c)
                        ang,x,y,z = 0,1,0,0
                        
                        # GL.glLoadMatrixf(self.imu.currentM.T)
                        # if ang+5 <180:
                        GL.glRotatef(5,self.axisRotation[0],self.axisRotation[1],self.axisRotation[2])
                        # else:
                            # GL.glRotatef(5,x,y,z)
                            
                        n = GL.glGetFloatv(GL.GL_MODELVIEW_MATRIX).T
                        # print(n)
                        # self.currentImu = np.dot(self.imu.currentM,n)
                        self.currentModel = np.dot(n,self.teapot.currentM)
                        # self.currentModel = np.dot(self.currentModel,n)
                        # self.currentImu = n
                        self.stopAdd = self.currentImu.copy()
                        # self.add=False
                    elif type(self.stopAdd) != type(None):
                        self.currentImu = self.stopAdd.copy()
                        self.stopAdd = None
                        self.axisRotation =(None,None,None)
                elif self.mode == 'absolute':
                    if self.f == True:
                        self.cimu = self.currentImu.copy()
                        self.cm = self.teapot.currentM.copy()
                        self.f = False
                    self.currentModel = np.dot(np.linalg.inv(self.cimu),self.cm)
                    self.currentModel = np.dot(self.currentImu,self.currentModel)
            
            # print(self.addAngleMatrix)
            # newImu = np.dot(transform,n)
            
            # newImu = self.imuTrans.copy()
        else:
            # self.currentImu = np.linalg.inv(self.hBaseToWorld.copy())
            self.currentImu = np.eye(4)
            
        
        self.imu.show = True
        self.imu.drawMatrixModel(self.currentImu,showFrame=True)
        self.teapot.drawMatrixModel(self.currentModel,showFrame=False)
        GL.glPopMatrix()
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
    port = '/dev/ttyUSB0' # arduino port
    
    # stylus = Stylus() # init stylus
    srec = StylusReciever(baudrate = 115200,port = port) # init serial
    
    # init GUI window
    ###########
    print("Start Program..\n")
    
    myGLWindow = MyGLWindow( 0, 0, 720, 480 )
    
    myGLWindow.show()
    
    while myGLWindow.shown():
        if srec.isActivate():
            command,rawData = srec.recieve()
            print(rawData)
            if command == 0xFF:
                # get joint states
                q,buttonStates = srec.readCommand(command,rawData)
                pose = stylus.getEndTransforms(q)
                
                print(buttonStates)
            if command == 0xFE:
                # get button states
                # print(bin(rawData[0]),bin(rawData[1]))
                # imuData,calibState = srec.readCommand(command,rawData)
                data = srec.encDataCvt(rawData[:-1])
                npData = np.array(data)/1000
                print(data)
                try:
                    myGLWindow.readImuQuat(npData)
                except:
                    pass
        myGLWindow.redraw()
        fltk.Fl_check()
        
    
    fltk.Fl_run()