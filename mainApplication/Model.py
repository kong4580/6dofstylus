from OpenGL import GL, GLUT, GLU

# from fltk import *
import sys
from math import sqrt
import drawFunc 
import time
import numpy as np
from scipy.spatial import ConvexHull

from Obb import OBB

class Model():
    def __init__(self,name,drawFunction = None,position=(0,0,0),rotation=(0,0,0),obj=None,m =np.eye(4)):
        self.name = name
        self.centerPosition = position
        self.rotation = rotation
        self.drawFunction = drawFunction
        self.obj = obj
        self.obb = None
        self.isSelected = False
        self.realPose = None
        self.cursorPose = None
        self.currentRotation = None
        self.show = True
        self.cursorM = None
        self.currentM = m
        self.startclickM = None
        
    def drawMatrixModel(self, matrix, showFrame=True, enableLight = True):
        
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glPushMatrix()
        GL.glLoadIdentity()
        
        if self.name == 'cursor':
            
            transform = np.array([[0,1,0,0],
                                [0,0,1,0],
                                [1,0,0,0],
                                [0,0,0,1]])
        else:
            transform = np.eye(4)
            
        nmatrix = np.dot(transform,matrix)
        GL.glLoadMatrixf(nmatrix.T)
        self.currentM = GL.glGetFloatv(GL.GL_MODELVIEW_MATRIX).T
        self.centerPosition = self.currentM[0:3,3]
        
        if self.obj!=None:
            if self.show:
                if self.obb.show:
                    GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_LINE)
                    GL.glCallList(self.obb.gl_list)
                GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_FILL)
                
                if self.isSelected:
                    GL.glColor3fv(drawFunc.SkyColorVector)
                else:
                    GL.glColor3fv(drawFunc.WhiteColorVector)
                
                if not enableLight:
                    GL.glDisable(GL.GL_LIGHTING)
                GL.glCallList(self.obj.gl_list)
                if not enableLight:
                    GL.glEnable(GL.GL_LIGHTING)
                # glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
                
                # glColor3fv(drawFunc.MagentaColorVector)
                # drawFunc.coordinate()
                # print(nmatrix)
                
                # glCallList(self.obj.shadow_gl_list)
            # self.obb.current_homo = np.dot(glGetFloatv(GL_MODELVIEW_MATRIX).T,self.obb.homo)
            self.obb.current_homo = self.currentM
            
            for idx in range(len(self.obb.points)):
                
                pointIdx = np.append(self.obb.points[idx].copy(),1)
                self.obb.current_point[idx] = np.dot(self.obb.current_homo.copy(),pointIdx.T)[0:3]
                
            # print("after",self.obb.current_point)
            obbCentroid = np.append(self.obb.current_centroid,1)
            self.obb.current_centroid = np.dot(self.obb.current_homo,obbCentroid)[0:3]
            
            for idx in range(len(self.obj.vertices)):
                verIdx = np.append(self.obj.vertices[idx].copy(),1)
        
                
                self.obj.current_vertices[idx] = np.dot(GL.glGetFloatv(GL.GL_MODELVIEW_MATRIX).T,verIdx.T)[0:3]
            
                
        else:
            if self.show:
                self.drawFunction()
                GL.glDisable(GL.GL_LIGHTING)
                drawFunc.coordinate()
                GL.glEnable(GL.GL_LIGHTING)
        
        if showFrame:
            drawFunc.coordinate()
                
        GL.glPopMatrix()
        
        
        
    # def drawModel(self,position=(0,0,0),rotation=(0,0,0),showFrame=False):
        
    #     if self.drawFunction != None:
    #         self.__updatePosition(position,rotation)
    #         # print("cen",self.centerPosition)
    #         # if self.obj == None:
            
            
    #         glMatrixMode(GL_MODELVIEW)
    #         glPushMatrix()
    #         glLoadIdentity()
            
    #         glTranslatef(self.centerPosition[0],self.centerPosition[1],self.centerPosition[2])
            
    #         # if self.name == 'cursor':
    #         #     glRotatef(self.rotation[0],0,1,0) #y#z
    #         #     glRotatef(self.rotation[2],1,0,0) #x#y
                
    #         #     glRotatef(self.rotation[1],0,0,1) #z#x
    #         # else:
    #         #     # glTranslatef(self.centerPosition[0],self.centerPosition[1],self.centerPosition[2])
                
    #         #     glRotatef(self.rotation[0],1,0,0) #y#z
            
    #         #     glRotatef(self.rotation[1],0,1,0) #x#y
    #         #     glRotatef(self.rotation[2],0,0,1) #z#
    #         # glTranslatef(-self.centerPosition[0],-self.centerPosition[1],-self.centerPosition[2])
    #         glRotatef(self.rotation[0],1,0,0) #y#z
    #         glRotatef(self.rotation[2],0,0,1) #z#x
    #         glRotatef(self.rotation[1],0,1,0) #x#y
            
    #         # glRotatef(self.rotation[0],1,0,0) #y#z
            
    #         # glRotatef(self.rotation[1],0,1,0) #x#y
    #         # glRotatef(self.rotation[2],0,0,1) #z#x
            
    #         glTranslatef(-self.centerPosition[0],-self.centerPosition[1],-self.centerPosition[2])
    #         glTranslatef(self.centerPosition[0],self.centerPosition[1],self.centerPosition[2])
            
    #         # drawFunc.coordinate()
            
    #         # self.realPose = glGetFloatv(GL_MODELVIEW_MATRIX).T[0:3,3].T
            
    #         if self.obj!=None:
    #             if self.show:
    #                 if self.obb.show:
    #                     glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
    #                     glCallList(self.obb.gl_list)
    #                 glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
                    
    #                 if self.isSelected:
    #                     glColor3fv(drawFunc.SkyColorVector)
    #                 else:
    #                     glColor3fv(drawFunc.WhiteColorVector)
                    
    #                 glCallList(self.obj.gl_list)
    #                 # glColor3fv(drawFunc.MagentaColorVector)
                    
    #                 # glCallList(self.obj.shadow_gl_list)
    #             # self.obb.current_homo = np.dot(glGetFloatv(GL_MODELVIEW_MATRIX).T,self.obb.homo)
    #             self.obb.current_homo = glGetFloatv(GL_MODELVIEW_MATRIX).T
    #             # print("ahomo",self.obb.current_homo)
    #             # print("before",self.name,self.obb.current_point)
    #             for idx in range(len(self.obb.points)):
                    
    #                 pointIdx = np.append(self.obb.points[idx].copy(),1)
                    
    #                 self.obb.current_point[idx] = dot(self.obb.current_homo.copy(),pointIdx.T)[0:3]
    #             # print("after",self.obb.current_point)
    #             obbCentroid = np.append(self.obb.current_centroid,1)
    #             self.obb.current_centroid = dot(self.obb.current_homo,obbCentroid)[0:3]
                
    #             for idx in range(len(self.obj.vertices)):
    #                 verIdx = np.append(self.obj.vertices[idx].copy(),1)
            
                    
    #                 self.obj.current_vertices[idx] = dot(glGetFloatv(GL_MODELVIEW_MATRIX).T,verIdx.T)[0:3]
    #             # self.centerPosition -=self.obb.current_centroid
                    
    #         else:
    #             if self.show:
    #                 self.drawFunction()
            
    #         if showFrame:
    #             drawFunc.coordinate()
            
    #         # if self.obj == None:
    #         glPopMatrix()
    #         # glPopMatrix()
    #     else:
    #         print("No model draw function")
    
    def __updatePosition(self,position=(0,0,0),rotation=(0,0,0)):
        self.centerPosition = position
        # if self.obj != None:
        #     # print(self.name)
        #     self.centerPosition -=self.obb.current_centroid
            
        self.rotation = rotation

    def getName(self):
        return self.name
    
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
        
    def isPointInsideConvexHull(self,point):
        
        hull = ConvexHull(self.obj.current_vertices,incremental = True)
        newHull = ConvexHull(np.concatenate((hull.points, [point])))
        
        if np.array_equal(newHull.vertices, hull.vertices): 
            return True
        
        return False
    
    def createOBB(self,showOBB=False):
        indices = []
        for face in self.obj.faces:
            indices.append(face[0][0] - 1)
            indices.append(face[0][1] - 1)
            indices.append(face[0][2] - 1)
        self.obb = OBB.build_from_triangles(self.obj.vertices, indices)
        self.obb.gl_list = GL.glGenLists(1)
        self.obb.show = showOBB
        GL.glNewList(self.obb.gl_list, GL.GL_COMPILE)
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
    
    def initModel(self,matrixView):
        self.obj.initOBJ()
        self.obj.current_vertices = self.obj.vertices.copy()
        # self.obj.createShadow()
        self.createOBB(showOBB=False)
        self.obb.current_point = self.obb.points
        self.obb.current_centroid = self.obb.centroid
        self.obb.current_homo = np.dot(matrixView,self.obb.homo)
        if self.obj != None:
            # print(self.name)
            self.centerPosition -=self.obb.current_centroid
        
    def followCursor(self,cursor):
        if self.cursorPose == None or type(self.cursorM) == type(None):
            self.cursorPose = cursor.rotation
            self.currentRotation = self.rotation
            
            self.cursorM = cursor.currentM.copy()
            self.startclickM = self.currentM.copy()
            
        deltaTransform = np.dot(cursor.currentM,np.linalg.inv(self.cursorM))
        newM = np.eye(4)
        
        # self.startclickM[0:3] = cursor.currentM[0:3]
        
        newM = np.dot(deltaTransform,self.startclickM)
        
        deltaRot = np.asarray(list(cursor.rotation)) - np.asarray(list(self.cursorPose))
        newRotation = self.currentRotation
        
        if deltaRot[0]**2+deltaRot[1]**2+deltaRot[2]**2 >=0.1: 
            newRotation = self.currentRotation + deltaRot
        
        return cursor.centerPosition,newRotation,newM
        
        
        
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
        
