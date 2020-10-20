
import sys
from math import sqrt
import math
import time

from OpenGL import GL, GLUT, GLU
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation as R

import drawFunc 
from Obb import OBB
class Transform():
    def __init__(self):
        self.parent = []
        self.child = []
        self.parentToLocal = np.eye(4)
        self.startWorldToLocal = np.eye(4)
    
    @property
    def worldToLocal(self):
        if len(self.parent) > 0:
            for i in self.parent:
                wTL = np.dot(i.worldToLocal , self.parentToLocal)
        else:
            wTL = self.parentToLocal
        
        return wTL

    
    
    
    def apply(self,transform):
        
        rotM = np.eye(4)
        tranM = np.eye(4)
        
        
        rotM[0:3,0:3] = transform[0:3,0:3]
        tranM[0:3,3] = transform[0:3,3]
        
        
        # newT = np.dot(tranM,self.worldToLocal)
        # newT = np.dot(self.parentToLocal,transform)
        newT = np.dot(tranM,self.parentToLocal)
        newT = np.dot(newT,rotM)

        
        
        # newT = np.dot(newT,rotM)
        
        if len(self.parent) > 0:
            
            oldTParent =np.linalg.inv(self.parent[0].worldToLocal)
        else:
            oldTParent = np.eye(4)
            
        # self.parentToLocal = np.dot(oldTParent,newT)
        self.parentToLocal = newT
        
        # if self.parentToLocal[0,3] < 1e10:
        #     self.parentToLocal[0,3] = 0
        # # if self.parentToLocal[1,3] < 1e10:
        # #     self.parentToLocal[1,3] = 0
        # # if self.parentToLocal[2,3] < 1e10:
        # #     self.parentToLocal[2,3] = 0
    
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
        # print(pR,wR)
        r = np.dot(np.linalg.pinv(pR),wR)
        
        newM = np.eye(4)
        newM[0:3,3] = t[0:3,3]
        newM[0:3,0:3] = r[0:3,0:3] 
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
    
    @property
    def argv(self):
        return 0
    # draw model with transform matrix
    def drawMatrixModel(self, showFrame=True, enableLight = True,wireFrame = False, opacity = False,mode = 'trans',selectedMode = False):
        
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
                    # print(tuple(self.obb.current_centroid))
                    GL.glTranslatef(-tuple(self.obb.centroid)[0],-tuple(self.obb.centroid)[1],-tuple(self.obb.centroid)[2])
                # if show OBB
                if self.obb.show:
                    
                    # set gl to draw obb in line mode
                    GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_LINE)
                    
                    # draw obb from list
                    GL.glCallList(self.obb.gl_list)
                
                # if wireFrame is enable
                if wireFrame:
                    
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
                if opacity:
                    
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
                if not enableLight:
                    # turn off lighting
                    GL.glDisable(GL.GL_LIGHTING)
                    
                # draw model
                if selectedMode:
                    GL.glLoadName(self.modelId)
                GL.glCallList(self.obj.gl_list)
                
                # if disable lighting
                if not enableLight:
                    # turn on lighting
                    GL.glEnable(GL.GL_LIGHTING)
                
                # change draw mode to solid
                GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_FILL)
                
                # reset attribute to remain attribute
                GL.glDisable(GL.GL_BLEND)
                GL.glPopAttrib()
                GL.glPopMatrix()

            # draw model frame
                if showFrame:
                    if mode == 'trans':
                        self.drawFrame(drawFunc.drawAxisX,101,selectedMode)
                        self.drawFrame(drawFunc.drawAxisY,102,selectedMode)
                        self.drawFrame(drawFunc.drawAxisZ,103,selectedMode)
                    # print(mode)
                    # if mode == 'trans':
                    #     drawFunc.coordinate()
                    if mode == 'rot':
                        self.drawFrame(drawFunc.drawCircleX,201,selectedMode)

                        self.drawFrame(drawFunc.drawCircleY,202,selectedMode)

                        self.drawFrame(drawFunc.drawCircleZ,203,selectedMode)
                        
                        # drawFunc.drawCircleZ()
                    # GL.glEnable(GL.GL_LIGHTING)
                    # drawFunc.coordinate()
        # if model is not obj file    
        else:
            
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
                
                
                if showFrame:
                    # disable light to draw model frame
                    GL.glDisable(GL.GL_LIGHTING)
                    self.drawFrame(drawFunc.coordinate,10,selectedMode)
                    GL.glEnable(GL.GL_LIGHTING)
                
                    
                GL.glPopMatrix()
                
        
                
        # GL.glPopMatrix()
    def updateChild(self):
        if len(self.child) > 0:
            # print(self.child[0].renderM)
            for c in self.child:
                c.currentM = c.worldToLocal
            # print(self.child[0].renderM)
            
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
        
        # if len(self.parent) > 0:
        #     matrix[0:3,3] = self.currentM[0:3,3]
        # print(self.name,matrix)
        if mode == 'absolute':
            

            transform = self.goTo(matrix)
            
            self.apply(transform)
        else:
            self.apply(matrix)
            
        self.currentM = self.worldToLocal
        # print(self.name,self.currentM,self.parentToLocal)
        
        self.updateChild()
        # set current model matrix from modelview matrix
        
        # set model center position
        self.centerPosition = self.currentM[0:3,3]
        if self.obj!=None:
        # set obb transform matrix
            self.obb.current_homo = self.currentM
            tl = np.eye(4)
            tl[0:3,3] = np.array([-tuple(self.obb.centroid)[0],-tuple(self.obb.centroid)[1],-tuple(self.obb.centroid)[2]]).T
            off = np.dot(self.currentM,tl)
            # transform obb point to current model transform
            for idx in range(len(self.obb.points)):
                pointIdx = np.append(self.obb.points[idx].copy(),1)
                self.obb.current_point[idx] = np.dot(self.obb.current_homo.copy(),pointIdx.T)[0:3]
                
            # update obb centroid
            obbCentroid = np.append(self.obb.current_centroid,1)
            self.obb.current_centroid = np.dot(self.obb.current_homo,obbCentroid)[0:3]
            
            # update model vertices position
            for idx in range(len(self.obj.vertices)):
                verIdx = np.append(self.obj.vertices[idx].copy(),1)
                self.obj.current_vertices[idx] = np.dot(off,verIdx.T)[0:3]
                
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
            # print(self.name)
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
        
        # glEnable(GL_TEXTURE_2D)
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
        # glDisable(GL_TEXTURE_2D)
        
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
        
    def drawMatrixModel(self, showFrame=True, enableLight = True,wireFrame = False, opacity = False,mode = 'trans',selectedMode = False):

        # if model is show
        if self.show:
            length = self.getDist
            # start transform matrix in model view
            GL.glMatrixMode(GL.GL_MODELVIEW)
            GL.glPushMatrix()
            GL.glLoadIdentity()

            # apply transform to model
            GL.glLoadMatrixf(self.renderM.T)
            scale = 1
            
            GL.glColor4f(1,1,1,1)
            
            
            if selectedMode:
                GL.glLoadName(self.modelId)
            GLUT.glutSolidSphere(.74,10,10)
            # r,lats,longs = 0.74,10,10
            # #  i, j
            # for i in range(lats):
            #     lat0 = math.pi * (-0.5 + (i - 1) / lats)
            #     z0  = math.sin(lat0)
            #     zr0 =  math.cos(lat0)

            #     lat1 = math.pi * (-0.5 + i / lats)
            #     z1 = math.sin(lat1)
            #     zr1 = math.cos(lat1)

            #     GL.glBegin(GL.GL_QUAD_STRIP)
            #     for j in range(longs):
            #         lng = 2 * math.pi * (j - 1) / longs
            #         x = math.cos(lng)
            #         y = math.sin(lng)

            #         GL.glNormal3f(x * zr0, y * zr0, z0)
            #         GL.glVertex3f(r * x * zr0, r * y * zr0, r * z0)
            #         GL.glNormal3f(x * zr1, y * zr1, z1)
            #         GL.glVertex3f(r * x * zr1, r * y * zr1, r * z1)
                # GL.glEnd()
            GL.glPopMatrix()
            
            GL.glMatrixMode(GL.GL_MODELVIEW)
            GL.glPushMatrix()
            GL.glLoadIdentity()
            GL.glLoadMatrixf(self.renderM.T)
            
            bl = (-length*1*scale,0.5*scale,0.5*scale)
            br = (-length*1*scale,0.5*scale,-0.5*scale)
            fl = (-length*1*scale,-0.5*scale,0.5*scale)
            fr = (-length*1*scale,-0.5*scale,-0.5*scale)
            t = (0,0,0)
            if selectedMode:
                GL.glLoadName(9999)
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
            
            if showFrame:
                # disable light to draw model frame
                GL.glDisable(GL.GL_LIGHTING)
                self.drawFrame(drawFunc.coordinate,10,selectedMode)
                GL.glEnable(GL.GL_LIGHTING)
            # GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_FILL)
            # GL.glDisable(GL.GL_CULL_FACE)
                
            # GL.glPopMatrix()
    @property
    def argv(self):
        return self.getDist
    @property
    def pointTo(self):
        if len(self.parent) > 0:
            
            np.seterr(divide='ignore', invalid='ignore')
            vAB = self.worldToLocal[0:3,3]-self.parent[0].worldToLocal[0:3,3]
            
            newM = np.eye(4)
            # print(self.getDist)
            
            newM[0:3,0] = (vAB.T.copy())/self.getDist
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
            return newM
        else:
            return self.worldToLocal
    
    @property
    def getDist(self):
        np.seterr(divide='ignore', invalid='ignore')
        from scipy.spatial import distance
        if len(self.parent) > 0:
            for i in self.parent:
                # d = i.worldToLocal[0:3,3] - self.worldToLocal[0:3,3]
                return distance.euclidean( self.worldToLocal[0:3,3].T,i.worldToLocal[0:3,3].T)
        else:
            return 0
    
    @property
    def renderM(self):
        return self.pointTo
    
    def moveModel(self,matrix,mode='absolute'):
        # if len(self.parent) > 0:
        #     matrix[0:3,3] = self.currentM[0:3,3]
        # print(self.name,matrix)
        if mode == 'absolute':
            transform = self.goTo(matrix)
            self.apply(transform)
        else:
            self.apply(matrix)
            
        self.currentM = self.worldToLocal
        # print(self.name,self.currentM,self.parentToLocal)
        
        self.updateChild()
        # set current model matrix from modelview matrix
        
        # set model center position
        self.centerPosition = self.currentM[0:3,3]
        
    def updateChild(self):
        if len(self.child) > 0:
            # print(self.child[0].renderM)
            for c in self.child:
                c.currentM = c.worldToLocal
            # print(self.child[0].renderM)
            
                c.updateChild()
        else:
            pass
        
    # init model
    def initModel(self,matrixView):
        
        pass
    

class ArticulateModel(Model):
    def __init__(self,name,modelId,listOfJoint,showTarget = False,showPole = False):
        super().__init__(name,modelId)
        self.listOfJoint = listOfJoint
        self.base = Model('base',60,drawFunc.DrawCube)
        self.target = Model('target',61,drawFunc.DrawCube)
        self.poleVertex = Model('pole',62,drawFunc.DrawCube)
        self.showTarget = showTarget
        self.showPole = showPole
        # self.target.goUnder(self.base)
        # self.poleVertex.goUnder(self.base)
        if len(self.listOfJoint)>0:
            self.listOfJoint[0].goUnder(self.base)
            for idx in range(len(self.listOfJoint)-1):
                self.listOfJoint[idx+1].goUnder(self.listOfJoint[idx])
        
        
        self.base.moveModel(np.array([[1,0,0,0.],
                                      [0,1,0,0],
                                      [0,0,1,0],
                                      [0,0,0,1]]))
        self.target.moveModel(np.array([[1,0,0,-1.],
                                      [0,1,0,5],
                                      [0,0,1,0],
                                      [0,0,0,1]]))
        self.poleVertex.moveModel(np.array([[1,0,0,1.],
                                      [0,1,0,5],
                                      [0,0,1,1],
                                      [0,0,0,1]]))
        self.modelLists = [self.base,self.target,self.poleVertex]
        self.modelLists.extend(self.listOfJoint)
        for m in self.modelLists:
            m.startWorldToLocal = m.worldToLocal
    # init model
    def initModel(self,matrixView):
        for joint in self.listOfJoint:
            joint.initModel(matrixView)
    
    def drawMatrixModel(self, showFrame=True, enableLight = True,wireFrame = False, opacity = False,mode = 'trans',selectedMode = False):
        self.base.drawMatrixModel(showFrame,enableLight,wireFrame,opacity,mode,selectedMode)
        if self.showTarget:
            self.target.drawMatrixModel(showFrame,enableLight,wireFrame,opacity,mode,selectedMode)
        if self.showPole:
            
            self.poleVertex.drawMatrixModel(showFrame,enableLight,wireFrame,opacity,mode,selectedMode)
        
        # print(self.base.currentM)
        for joint in self.listOfJoint:
            joint.drawMatrixModel(showFrame,enableLight,wireFrame,opacity,mode,selectedMode)
    
    def getSubModel(self):
        return self.modelLists
    
    
    
    
    
if __name__ == "__main__":
    a = Transform()
    b = Transform()
    c = Transform()
    b.parentToLocal[0,3] = 2
    a.parentToLocal[0,3] = 0
    c.parentToLocal[0,3] = 9
    
    b.goUnder(a)
    # b.parent.append(a)
    # a.child.append(b)
    # a.parentToLocal[1,3] = 5
    b.apply(np.array([[0,-1,0,0],
                        [1,0,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]))
    a.apply(np.array([[0,-1,0,5],
                        [1,0,0,2],
                        [0,0,1,0],
                        [0,0,0,1]]))
    # a.parentToLocal = np.dot(np.array([[0,-1,0,0],
    #                                      [1,0,0,0],
    #                                      [0,0,1,0],
    #                                      [0,0,0,1]]),a.parentToLocal)
    c.goUnder(b)
    
    
    print(b.worldToLocal)
    
    print(a.worldToLocal)
    