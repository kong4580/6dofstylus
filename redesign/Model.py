
import sys
from math import sqrt
import math
import time

from OpenGL import GL, GLUT, GLU
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation as R
from Manipulator import Manipulator
import drawFunc 
from Obb import OBB
class Transform():
    def __init__(self):
        self.parent = []
        self.child = []
        self.parentToLocal = np.eye(4)
        self.startWorldToLocal = np.eye(4)
        self.manipulator = Manipulator()
        # self.tt = np.eye(4)
    @property
    def worldToLocal(self):
        if len(self.parent) > 0:
            # for i in self.parent:
            wTL = np.dot(self.parent[0].worldToLocal , self.parentToLocal )
            # wTL = np.dot(wTL ,self.tt)
                
        else:
            
            # wTL = np.dot(self.parentToLocal,self.tt )
            wTL = self.parentToLocal
            
        
        return wTL

    
    
    
    def apply(self,transform):
        
        rotM = np.eye(4)
        tranM = np.eye(4)
        
        
        rotM[0:3,0:3] = transform[0:3,0:3]
        tranM[0:3,3] = transform[0:3,3]
        
        oldttT = np.eye(4)
        oldttR = np.eye(4)
        
        oldttT[0:3,3] = self.parentToLocal[0:3,3]
        oldttR[0:3,0:3] = self.parentToLocal[0:3,0:3]
        
        # newT = np.dot(tranM,self.worldToLocal)
        # newT = np.dot(self.parentToLocal,transform)
        newT = np.eye(4)
        newttT = np.dot(oldttT,tranM)
        newttR = np.dot(oldttR,rotM)
        
        newT[0:3,3] = newttT[0:3,3]
        newT[0:3,0:3] = newttR[0:3,0:3]
        
        
        # newT = np.dot(newT,rotM)
        
        if len(self.parent) > 0:
            
            oldTParent =np.linalg.inv(self.parent[0].worldToLocal)
        else:
            oldTParent = np.eye(4)
            
        # self.parentToLocal = np.dot(oldTParent,newT)
        self.parentToLocal = newT
      
    
    def goUnder(self,transform):
        self.parent.append(transform)
        transform.child.append(self)
    
    def goTo(self, matrix):
        
        
        pT=np.eye(4)
        pT[0:3,3] = self.worldToLocal[0:3,3].copy()
        
        pR = np.eye(4)
        pR[0:3,0:3] = self.worldToLocal[0:3,0:3].copy()
        
        wT = np.eye(4)
        wT[0:3,3] = matrix[0:3,3].copy()
        wR = np.eye(4)
        wR[0:3,0:3] = matrix[0:3,0:3].copy()
        
        t = np.dot(np.linalg.inv(pT),wT)
        
        r = np.dot(np.linalg.pinv(pR),wR)
        
        newM = np.eye(4)
        newM[0:3,3] = t[0:3,3]
        newM[0:3,0:3] = r[0:3,0:3] 
        
        if len(self.parent) > 0:
            
            self.parentToLocal = np.dot(np.linalg.inv(self.parent[0].worldToLocal),matrix)

            return np.eye(4)
        else:
            return newM
        
class Model(Transform):
    
    # init model class
    def __init__(self,name,modelId,drawFunction = None,position=(0,0,0),rotation=(0,0,0),obj=None,m =np.eye(4)):
        super().__init__()
        # model name
        self.name = name
        self.modelId = modelId
        # model position
        self.centerPosition = position
        
        # model rotation
        self.rotation = rotation
        
        # model draw function
        self.drawFunction = drawFunction
        
        # model obj file
        self.obj = obj
        
        # model oriented bounding box
        self.obb = None
        
        # VARIABLE FOR CALCULATE MODEL TRANSFORM FOLLOW CURSOR
        
        # model pose and rotation use for calculate follow cursor
        self.realPose = None
        self.cursorPose = None
        self.currentRotation = None
        
        # model transform use for calculate follow cursor
        self.cursorM = None
        self.currentM = m.copy()
        self.startclickM = None
        self.offsetTran = None
        self.invOffsetTran = None
        # model opacity
        self.opacityValue = 1
        
        # model flags
        self.isSelected = False
        self.show = True
        self.flags = {
                      'showModel':True,
                      
                      
                      'showModelWireframe':False,
                      
                      'opacityMode':False,
                      
                      'enableLight': True,
                      
                      'showModelFrame':False,
                      'snapMode':False
                      }
        
    def updateFlags(self, flags,data):
        self.flags[flags] = data
    
    def resetFlags(self):
        self.flags = {
                      'showModel':True,
                      
                      
                      'showModelWireframe':False,
                      
                      'opacityMode':False,
                      
                      'enableLight': True,
                      
                      'showModelFrame':False,
                      'snapMode':False
                      }
        
    @property
    def argv(self):
        return 0
    
    # draw model with transform matrix
    def drawMatrixModel(self, showFrame=True, enableLight = True,wireFrame = False, opacity = False,mode = 'trans',selectedMode = False,coordinate = True,camera = [1,0,0]):
        # if model is obj firl
        if self.obj!=None:
            
            # if model show flags is true
            if self.show:
                # start transform matrix in model view
                GL.glMatrixMode(GL.GL_MODELVIEW)
                GL.glPushMatrix()
                GL.glLoadIdentity()

                # apply transform to model
                GL.glLoadMatrixf(self.currentM.T)
                if self.obj!=None:

                    GL.glTranslatef(-tuple(self.obb.centroid)[0],-tuple(self.obb.centroid)[1],-tuple(self.obb.centroid)[2])
                # if show OBB
                if self.obb.show:
                    
                    # set gl to draw obb in line mode
                    GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_LINE)
                    
                    # draw obb from list
                    GL.glCallList(self.obb.gl_list)
                
                # if wireFrame is enable
                if self.flags['showModelWireframe']:
                    
                    # turn off back face
                    GL.glEnable(GL.GL_CULL_FACE)
                    GL.glCullFace(GL.GL_BACK)
                    
                    # draw model in line mode
                    GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_LINE)
                
                # if wireFrame is disable
                else:
                    # turn off wireFrame
                    GL.glDisable(GL.GL_CULL_FACE)
                    
                    # draw model in solid mode
                    GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_FILL)
                
                # if model is selected
                if self.isSelected:
                    
                    # set model color to sky color
                    color = drawFunc.SkyColorVector
                    
                # if model is not selected
                else:
                    
                    # set model color to white
                    color = drawFunc.WhiteColorVector
                
                # if opacity is enable
                if self.flags['opacityMode']:
                    
                    # set model opacity to 0.75
                    self.opacityValue = 0.75
                    
                # if opacity is disable
                else:
                    
                    # set model opacity to 1
                    self.opacityValue = 1
                
                # add opacityValue to color vector
                color = list(color).copy()
                color.append(self.opacityValue)
                
                # set model color
                GL.glColor4fv(tuple(color))
                
                # set mode attribute
                GL.glPushAttrib(GL.GL_COLOR_BUFFER_BIT)
                GL.glEnable(GL.GL_BLEND)
                GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)
                
                # if disable lighting
                if not self.flags['enableLight']:
                    # turn off lighting
                    GL.glDisable(GL.GL_LIGHTING)
                    
                # draw model
                if selectedMode:
                    GL.glLoadName(self.modelId)
                GL.glCallList(self.obj.gl_list)
                
                # if disable lighting
                if not self.flags['enableLight']:
                    # turn on lighting
                    GL.glEnable(GL.GL_LIGHTING)
                
                # change draw mode to solid
                GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_FILL)
                
                # reset attribute to remain attribute
                GL.glDisable(GL.GL_BLEND)
                GL.glPopAttrib()
                GL.glPopMatrix()

            # draw model frame
                if self.flags['showModelFrame']:
            
                    if self.isSelected:
                        self.manipulator.drawManipulator(mode,coordinate,self.currentM,selectedMode,camera)
                        
        # if model is not obj file    
        else:
            pass

    def updateChild(self):
        if len(self.child) > 0:
            
            for c in self.child:
                c.currentM = c.worldToLocal
                
                c.updateChild()
        else:
            pass
    def drawFrame(self,drawFrameFunc,frameId,selectedMode=False):
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glPushMatrix()
        GL.glLoadIdentity()
        
        # apply transform to model
        GL.glLoadMatrixf(self.currentM.T)
        GL.glDisable(GL.GL_LIGHTING)
        if selectedMode:
            GL.glLoadName(frameId)
        drawFrameFunc()

        GL.glEnable(GL.GL_LIGHTING)
        GL.glPopMatrix()
        
    def moveModel(self,matrix,mode='absolute'):
        
        
        
        if mode == 'absolute':
            

            transform = self.goTo(matrix)
            
            self.apply(transform)
        else:
            self.apply(matrix)
            
        self.currentM = self.worldToLocal
            
        
        self.updateChild()
        # set current model matrix from modelview matrix
        # set model center position
        self.centerPosition = self.currentM[0:3,3]
                
    @property
    def renderM(self):
        return self.currentM.copy()
    # update model position
    ### now not use ###
    def __updatePosition(self,position=(0,0,0),rotation=(0,0,0)):
        self.centerPosition = position
        
            
        self.rotation = rotation

    # get model name
    def getName(self):
        return self.name
    
    # check if point is inside OBB
    def isPointInsideOBB(self,point):
        
        centroidToPoint = point - self.obb.current_centroid
        
        pX = np.absolute(np.dot(centroidToPoint,self.obb.current_homo[0:3,0]))
        pY = np.absolute(np.dot(centroidToPoint,self.obb.current_homo[0:3,1]))
        pZ = np.absolute(np.dot(centroidToPoint,self.obb.current_homo[0:3,2]))
        
        xLength = self.obb.xLength
        yLength = self.obb.yLength
        zLength = self.obb.zLength
        
        if 2*pX <= xLength and 2*pY <= yLength and 2*pZ <= zLength:
            return True
        else:
            return False
    
    # create OBB around model
    def createOBB(self,showOBB=False):
        
        # create buffer
        indices = []
        
        # add each face to buffer
        for face in self.obj.faces:
            indices.append(face[0][0] - 1)
            indices.append(face[0][1] - 1)
            indices.append(face[0][2] - 1)
            
        # create obb
        self.obb = OBB.build_from_triangles(self.obj.vertices, indices)
        
        # generate list to draw obb
        self.obb.gl_list = GL.glGenLists(1)
        GL.glNewList(self.obb.gl_list, GL.GL_COMPILE)
        
        # set obb show
        self.obb.show = showOBB
        
        # begin draw obb
        GL.glBegin(GL.GL_LINES)
        GL.glColor3fv((1, 0, 0))

        def inputVertex(x, y, z):
            GL.glVertex3fv(self.obb.transform((x, y, z)))

        inputVertex(*self.obb.max)
        inputVertex(self.obb.max[0], self.obb.min[1], self.obb.max[2])

        inputVertex(self.obb.max[0], self.obb.min[1], self.obb.max[2])
        inputVertex(self.obb.min[0], self.obb.min[1], self.obb.max[2])

        inputVertex(self.obb.min[0], self.obb.min[1], self.obb.max[2])
        inputVertex(self.obb.min[0], self.obb.max[1], self.obb.max[2])

        inputVertex(self.obb.min[0], self.obb.max[1], self.obb.max[2])
        inputVertex(*self.obb.max)

        inputVertex(self.obb.max[0], self.obb.max[1], self.obb.max[2])
        inputVertex(self.obb.max[0], self.obb.max[1], self.obb.min[2])

        inputVertex(self.obb.max[0], self.obb.min[1], self.obb.max[2])
        inputVertex(self.obb.max[0], self.obb.min[1], self.obb.min[2])

        inputVertex(self.obb.min[0], self.obb.max[1], self.obb.max[2])
        inputVertex(self.obb.min[0], self.obb.max[1], self.obb.min[2])

        inputVertex(self.obb.min[0], self.obb.min[1], self.obb.max[2])
        inputVertex(self.obb.min[0], self.obb.min[1], self.obb.min[2])

        inputVertex(self.obb.max[0], self.obb.max[1], self.obb.min[2])
        inputVertex(self.obb.max[0], self.obb.min[1], self.obb.min[2])

        inputVertex(self.obb.max[0], self.obb.min[1], self.obb.min[2])
        inputVertex(*self.obb.min)

        inputVertex(*self.obb.min)
        inputVertex(self.obb.min[0], self.obb.max[1], self.obb.min[2])

        inputVertex(self.obb.min[0], self.obb.max[1], self.obb.min[2])
        inputVertex(self.obb.max[0], self.obb.max[1], self.obb.min[2])
        
        GL.glEnd()
        GL.glEndList()
    
    # init model
    def initModel(self,matrixView):
        
        # init obj class
        self.obj.initOBJ()
        
        # update vertices
        self.obj.current_vertices = self.obj.vertices.copy()
        # self.obj.createShadow()
        
        # create OBB
        self.createOBB(showOBB=False)
        
        # update obb variable
        self.obb.current_point = self.obb.points
        self.obb.current_centroid = self.obb.centroid
        self.obb.current_homo = np.dot(matrixView,self.obb.homo)
        if self.obj != None:
            self.centerPosition -=self.obb.current_centroid
    
    def getSubModel(self):
        
        return [self]
        
        
        
class OBJ():
    def __init__(self, filename, swap_yz=False,scale=1):
        self.vertices = []
        self.normals = []
        self.texcoords = []
        self.faces = []
        self.current_vertices = []

        material = None
        start = time.time()
        for line in open(filename, "r"):
            if line.startswith('#'):
                continue
            values = line.split()
            if not values:
                continue
            if values[0] == 'v':
                v = list(map(float, values[1:4]))
                if scale!=1:
                    for i in range(len(v)):
                        v[i] = v[i]*scale
                if swap_yz:
                    v = v[0], v[2], v[1]
                self.vertices.append(v)
            elif values[0] == 'vn':
                vn = list(map(float, values[1:4]))
                if swap_yz:
                    vn = vn[0], vn[2], vn[1]
                self.normals.append(vn)
            elif values[0] == 'vt':
                vt = list(map(float, values[1:3]))
                self.texcoords.append(vt)
            
            elif values[0] == 'f':
                vertices = []
                texcoords = []
                norms = []
                face_elements = values[1:]
                if len(face_elements) > 3:
                    raise Exception('unsupported polygonal face')
                for face_element in face_elements:
                    w = face_element.split('/')
                    
                    vertices.append(int(w[0]))
                    if len(w) >= 2 and len(w[1]) > 0:
                        texcoords.append(int(w[1]))
                    else:
                        texcoords.append(0)
                    if len(w) >= 3 and len(w[2]) > 0:
                        norms.append(int(w[2]))
                    else:
                        norms.append(0)
                self.faces.append((vertices, norms, texcoords, material))
        end = time.time()
        
        
    def initOBJ(self):
        self.gl_list = GL.glGenLists(1)
        GL.glNewList(self.gl_list, GL.GL_COMPILE)
        
        GL.glFrontFace(GL.GL_CCW)
        start = time.time()
        for vertices in self.faces:
            vertices, normals, texcoords, material = vertices
            
            GL.glBegin(GL.GL_TRIANGLES)
            
            
            for i in range(len(vertices)):
                if normals[i] > 0:
                    GL.glNormal3fv(self.normals[normals[i] - 1])
                if texcoords[i] > 0:
                    GL.glTexCoord2fv(self.texcoords[texcoords[i] - 1])
                GL.glVertex3fv(self.vertices[vertices[i] - 1])
                
            GL.glEnd()
        end = time.time()
        
        GL.glEndList()
        
        
    def createShadow(self):
        self.shadow_gl_list = glGenLists(1)
        glNewList(self.shadow_gl_list, GL_COMPILE)
        
        glFrontFace(GL_CCW)
        for vertices in self.faces:
            vertices, normals, texcoords, material = vertices
            
            glBegin(GL_TRIANGLES)
            
            
            for i in range(len(vertices)):
                Px = self.current_vertices[vertices[i] - 1][0]
                Py = self.current_vertices[vertices[i] - 1][1]
                Pz = self.current_vertices[vertices[i] - 1][2]
                Lx = 0
                Ly = 0
                Lz = 10
                nx = 0
                ny = 0
                nz = 1
                d = -10
                planeLength = 30
                if(nx*(Px-Lx)+ny*(Py-Ly)+nz*(Pz-Lz) != 0):
                    tParameter = .999*(d-nx*Lx-ny*Ly-nz*Lz)/(nx*(Px-Lx)+ny*(Py-Ly)+nz*(Pz-Lz))#there was a float error here that was fixed with using 1.* (...)making the whole expression as a float
                    Sx = (Px-Lx)*tParameter + Lx
                    Sy = (Py-Ly)*tParameter + Ly
                    Sz = (Pz-Lz)*tParameter + Lz
                    glVertex3fv([Sx,Sy,Sz])
                
            glEnd()
        
        glEndList()
        

class Joint(Model):
    def __init__(self,name,modelId,drawFunction = None,position=(0,0,0),rotation=(0,0,0),obj=None,m =np.eye(4)):
        super().__init__(name,modelId,drawFunction,position,rotation,obj,m)
        
    def drawMatrixModel(self, showFrame=True, enableLight = True,wireFrame = False, opacity = False,mode = 'trans',selectedMode = False,camera = [1,0,0]):

        # if model is show
        if self.show:
            length = self.getDist
            # start transform matrix in model view
            GL.glMatrixMode(GL.GL_MODELVIEW)
            GL.glPushMatrix()
            GL.glLoadIdentity()

            # apply transform to model
            newRender = self.renderM.copy()
            d = np.linalg.det(newRender[0:3, 0:3])
            # GL.glLoadMatrixf(self.renderM.T)
            GL.glLoadMatrixf(self.worldToLocal.T)
            
            
            scale = 1
            
            color = drawFunc.WhiteColorVector
            
            # if opacity is enable
            if self.flags['opacityMode']:
                
                # set model opacity to 0.75
                self.opacityValue = 0.75
                
            # if opacity is disable
            else:
                
                # set model opacity to 1
                self.opacityValue = 1
            
            # add opacityValue to color vector
            color = list(color).copy()
            color.append(self.opacityValue)
            
            # set model color
            GL.glColor4fv(tuple(color))
            
            if not self.flags['showModelWireframe']:
                    color = drawFunc.YellowColorVector
                    color = list(color).copy()
                    color.append(self.opacityValue)
                    
                    # set model color
                    GL.glColor4fv(tuple(color))
                    
                    
            else:
                color = drawFunc.SkyColorVector
                color = list(color).copy()
                # color.append(self.opacityValue)
                
                # set model color
                GL.glColor3fv(tuple(color))
                    
                    
            
            # set mode attribute
            GL.glPushAttrib(GL.GL_COLOR_BUFFER_BIT)
            GL.glEnable(GL.GL_BLEND)
            GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)
            
            # if disable lighting
            
            if not self.flags['enableLight']:
                # turn off lighting
                GL.glDisable(GL.GL_LIGHTING)
            
            
            if selectedMode:
                GL.glLoadName(self.modelId)
            
            
            if not self.flags['showModelWireframe']:
                GL.glDisable(GL.GL_CULL_FACE)
                
                if self.drawFunction == 'sphere':
                    GLUT.glutSolidSphere(.74,10,10)
                elif self.drawFunction == 'torus':
                    GLUT.glutSolidTorus(0.2,0.74,10,10)
                elif self.drawFunction == 'teapot':
                    GLUT.glutSolidTeapot(1)
                    
            else:
                GL.glEnable(GL.GL_CULL_FACE)
                GL.glCullFace(GL.GL_BACK)
                if self.drawFunction == 'sphere':
                    GLUT.glutWireSphere(.74,10,10)
                elif self.drawFunction == 'torus':
                    GLUT.glutWireTorus(0.2,0.74,10,10)
                elif self.drawFunction == 'teapot':
                    GLUT.glutWireTeapot(1)
            if not self.flags['enableLight']:
                # turn on lighting
                GL.glEnable(GL.GL_LIGHTING)
            
            
            # reset attribute to remain attribute
            GL.glDisable(GL.GL_BLEND)
            GL.glPopAttrib()
            GL.glPopMatrix()
            
            if not np.equal(self.renderM, self.worldToLocal).all():
                GL.glMatrixMode(GL.GL_MODELVIEW)
                GL.glPushMatrix()
                GL.glLoadIdentity()
                
                GL.glLoadMatrixf(self.renderM.T)
                GL.glTranslatef(-length,0,0,1)
                
                GL.glRotatef(90,0,1,0)
                
                GL.glPushAttrib(GL.GL_COLOR_BUFFER_BIT)
                GL.glEnable(GL.GL_BLEND)
                GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)
                # change draw mode to solid
                GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_FILL)
                
                
                if selectedMode:
                    GL.glLoadName(9999)
                
                if not self.flags['showModelWireframe']:
                    color = drawFunc.YellowColorVector
                    color = list(color).copy()
                    color.append(self.opacityValue)
                    
                    # set model color
                    GL.glColor4fv(tuple(color))
                    GL.glDisable(GL.GL_CULL_FACE)
                    
                    GLUT.glutSolidCone(0.74,length,20,4)
                else:
                    color = drawFunc.SkyColorVector
                    color = list(color).copy()
                    
                    # set model color
                    GL.glColor3fv(tuple(color))
                    GL.glEnable(GL.GL_CULL_FACE)
                    GL.glCullFace(GL.GL_BACK)
                    GLUT.glutWireCone(0.74,length,20,4)
                    
                GL.glDisable(GL.GL_BLEND)
                GL.glEnable(GL.GL_LIGHTING)
                GL.glPopAttrib()
                
                GL.glPopMatrix()
            GL.glLineWidth(1)
            GL.glEnable(GL.GL_LIGHTING)
            GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_FILL)
            if showFrame and not self.flags['snapMode']:
                # disable light to draw model frame
                GL.glDisable(GL.GL_LIGHTING)
                GL.glEnable(GL.GL_LIGHTING)
                
    @property
    def argv(self):
        return self.getDist
    @property
    def pointTo(self):
        if len(self.parent) > 0:
            
            np.seterr(divide='ignore', invalid='ignore')
            vAB = self.worldToLocal[0:3,3]-self.parent[0].worldToLocal[0:3,3]
            
            newM = np.eye(4)
            
            newM[0:3,0] = (vAB.T.copy())/self.getDist
            
            newM[0:3,0] = newM[0:3,0]/np.linalg.norm(newM[0:3,0])
            if newM[0,0] > 0:
                sign0 = 1
            else:
                sign0 = -1
            
            if newM[1,0] > 0:
                sign1 = 1
            else:
                sign1 = -1
                
            newM[0,1] = sign0 * abs(newM[1,0])
            newM[1,1] = -1 *sign1 * abs(newM[0,0])
            newM[0:3,2] = np.cross(newM[0:3,0].copy(), newM[0:3,1].copy())
            newM[0:3,3] = self.worldToLocal[0:3,3].T.copy()
            
            newM[0:3,0] = newM[0:3,0]/np.linalg.norm(newM[0:3,0])
            newM[0:3,1] = newM[0:3,1]/np.linalg.norm(newM[0:3,1])
            newM[0:3,2] = newM[0:3,2]/np.linalg.norm(newM[0:3,2])
            
            if np.isnan(newM).any():
                newM = self.worldToLocal
            return newM
        else:
            print(self.name)
            return self.worldToLocal
    
    @property
    def getDist(self):
        np.seterr(divide='ignore', invalid='ignore')
        from scipy.spatial import distance
        if len(self.parent) > 0:
            for i in self.parent:
                return distance.euclidean( self.worldToLocal[0:3,3].T,i.worldToLocal[0:3,3].T)
        else:
            return 0
    
    @property
    def renderM(self):
        return self.pointTo
    
    def moveModel(self,matrix,mode='absolute'):
        if mode == 'absolute':
            transform = self.goTo(matrix)
            self.apply(transform)
        else:
            self.apply(matrix)
            
        self.currentM = self.worldToLocal
        
        self.updateChild()
        # set current model matrix from modelview matrix
        
        # set model center position
        self.centerPosition = self.currentM[0:3,3]
        
    def updateChild(self):
        if len(self.child) > 0:
            for c in self.child:
                c.currentM = c.worldToLocal
            
                c.updateChild()
        else:
            pass
        
    # init model
    def initModel(self,matrixView):
        
        pass
    
class ArticulateObj(Model):
    
    def drawMatrixModel(self, showFrame=True, enableLight = True,wireFrame = False, opacity = False,mode = 'trans',selectedMode = False,coordinate = True,camera = [1,0,0]):
        
        # if model is show
            
            if not self.flags['snapMode'] and self.show :
                
                # start transform matrix in model view
                GL.glMatrixMode(GL.GL_MODELVIEW)
                GL.glPushMatrix()
                GL.glLoadIdentity()

                # apply transform to model
                GL.glLoadMatrixf(self.renderM.T)
                # # draw model from drawFunction
               # if model is selected
                if self.isSelected:
                    
                    # set model color to sky color
                    color = drawFunc.SkyColorVector
                    
                # if model is not selected
                else:
                    
                    # set model color to white
                    color = drawFunc.WhiteColorVector
                
                # add opacityValue to color vector
                color = list(color).copy()
                color.append(self.opacityValue)
                
                # set model color
                GL.glColor4fv(tuple(color))
                
                # set mode attribute
                GL.glPushAttrib(GL.GL_COLOR_BUFFER_BIT)
                GL.glEnable(GL.GL_BLEND)
                GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)
                
                
                # if disable lighting
                # turn off lighting
                GL.glDisable(GL.GL_LIGHTING)
                    
                # draw model
                if selectedMode:
                    GL.glLoadName(self.modelId)
                    
                self.drawFunction(ratio = 1,selectedMode = selectedMode)
                if self.flags['showModelFrame']:
                    if self.isSelected:
                        self.manipulator.drawManipulator(mode,coordinate,self.currentM,selectedMode,camera)
                    
                
                GL.glEnable(GL.GL_LIGHTING)
                
                # change draw mode to solid
                GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_FILL)
                
                # reset attribute to remain attribute
                GL.glDisable(GL.GL_BLEND)
                GL.glPopAttrib()
                GL.glPopMatrix()
    
class ArticulateModel(Model):
    def __init__(self,name,modelId,listOfJoint,showTarget = False,showPole = False):
        super().__init__(name,modelId)
        self.listOfJoint = listOfJoint
        self.base = ArticulateObj('base',60,drawFunc.drawBase)
        self.target = ArticulateObj('target',61,drawFunc.drawTarget)
        self.poleVertex = ArticulateObj('pole',62,drawFunc.drawPole)
        self.showTarget = showTarget
        self.showPole = showPole
        
        if len(self.listOfJoint)>0:
            self.listOfJoint[0].goUnder(self.base)
            for idx in range(len(self.listOfJoint)-1):
                self.listOfJoint[idx+1].goUnder(self.listOfJoint[idx])
        
        
        self.base.moveModel(np.array([[1,0,0,0.],
                                      [0,1,0,0],
                                      [0,0,1,0],
                                      [0,0,0,1]]))
        self.target.parentToLocal = np.array([[1,0,0,-3.],
                                      [0,1,0,6],
                                      [0,0,1,0],
                                      [0,0,0,1]])
        self.poleVertex.parentToLocal = np.array([[1,0,0,3.],
                                      [0,1,0,6],
                                      [0,0,1,1],
                                      [0,0,0,1]])
        self.target.moveModel(np.eye(4),mode='relative')
        self.poleVertex.moveModel(np.eye(4),mode='relative')
        
        self.modelLists = [self.base,self.target,self.poleVertex]
        self.modelLists.extend(self.listOfJoint)
        for m in self.modelLists:
            m.startWorldToLocal = m.worldToLocal
    # init model
    def initModel(self,matrixView):
        for joint in self.listOfJoint:
            joint.initModel(matrixView)
    
    def drawMatrixModel(self, showFrame=True, enableLight = True,wireFrame = False, opacity = False,mode = 'trans',selectedMode = False,coordinate = True,camera = [1,0,0]):
        
        
        for model in self.getSubModel():
            model.flags = self.flags
            
            model.show = model.flags['showModel']
            
        self.base.drawMatrixModel(showFrame,enableLight,wireFrame,opacity,mode,selectedMode,coordinate,camera = camera)
        if self.showTarget:
            self.target.drawMatrixModel(showFrame,enableLight,wireFrame,opacity,mode,selectedMode,coordinate,camera = camera)
        if self.showPole:
            
            self.poleVertex.drawMatrixModel(showFrame,enableLight,wireFrame,opacity,mode,selectedMode,coordinate,camera = camera)
        
        for joint in self.listOfJoint:
            joint.drawMatrixModel(showFrame,enableLight,wireFrame,opacity,mode,selectedMode,camera = camera)
    
    def getSubModel(self):
        return self.modelLists
    
class Cursor(Model):
    
    def drawMatrixModel(self, showFrame=True, enableLight = True,wireFrame = False, opacity = False,mode = 'trans',selectedMode = False,coordinate = True,camera = [1,0,0]):
        
        # if model is show
            if self.show:
                # start transform matrix in model view
                GL.glMatrixMode(GL.GL_MODELVIEW)
                GL.glPushMatrix()
                GL.glLoadIdentity()

                # apply transform to model
                GL.glLoadMatrixf(self.renderM.T)
                # # draw model from drawFunction
                
                # draw model
                if selectedMode:
                    
                    GL.glLoadName(self.modelId)
                self.drawFunction()
                    
                if self.name == 'cursor':
                
                    if showFrame:
                        # disable light to draw model frame
                        GL.glDisable(GL.GL_LIGHTING)
                        self.drawFrame(drawFunc.coordinate,10,selectedMode)
                        GL.glEnable(GL.GL_LIGHTING)
                    
                        
                else:
                    if self.isSelected:
                        self.manipulator.drawManipulator(mode,coordinate,self.currentM,selectedMode,camera)
                    else:
                        pass
                    
                GL.glPopMatrix()
    

if __name__ == "__main__":
    c = Cursor('cursor',99,drawFunc.drawCursor,obj=None)
    print(c.name)