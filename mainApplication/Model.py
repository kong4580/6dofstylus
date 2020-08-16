from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
# from PIL.Image import *
from fltk import *
import sys
from math import sqrt
from drawFunc import *
import time
import numpy as np
class Model():
    def __init__(self,name,drawFunction = None,position=(0,0,0),rotation=(0,0,0),obj=None):
        self.name = name
        self.centerPosition = position
        self.rotation = rotation
        self.drawFunction = drawFunction
        self.obj = obj
        self.obb = None
        self.isSelect = False
        
    def createModel(self,position=(0,0,0),rotation=(0,0,0),showFrame=False):
        
        if self.drawFunction != None:
            self.__updatePosition(position,rotation)
            # if self.obj == None:
            glPushMatrix()
            glLoadIdentity()
            
            glTranslatef(self.centerPosition[0],self.centerPosition[1],self.centerPosition[2])
            glRotatef(self.rotation[0],0,1,0) #y#z
            glRotatef(self.rotation[1],0,0,1) #z#x
            glRotatef(self.rotation[2],1,0,0) #x#y
            glTranslatef(-self.centerPosition[0],-self.centerPosition[1],-self.centerPosition[2])
            glTranslatef(self.centerPosition[0],self.centerPosition[1],self.centerPosition[2])
            if self.obj!=None:
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
                
                glCallList(self.obj.gl_list)
                
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
                glCallList(self.obb.gl_list)
                
                self.obb.current_homo = np.dot(glGetFloatv(GL_MODELVIEW_MATRIX).T,self.obb.homo)
                for idx in range(len(self.obb.points)):
                    
                    pointIdx = np.append(self.obb.points[idx],1)
                    print(pointIdx)
                    self.obb.current_point[idx] = dot(self.obb.current_homo,pointIdx.T)[0:3]
                obbCentroid = np.append(self.obb.centroid,1)
                self.obb.current_centroid = dot(self.obb.current_homo,obbCentroid)[0:3]
                # print(self.obb.current_point)
                # print(self.obb.extents)
                # print(np.linalg.norm(self.obb.current_point[0]-self.obb.current_point[3]))
                # print(self.isInside(np.array([1,-2,3])))
            else:
                self.drawFunction()
                
            if showFrame:
                coordinate()
            
            # if self.obj == None:
            glPopMatrix()
            # glPopMatrix()
        else:
            print("No model draw function")
    
    def __updatePosition(self,position=(0,0,0),rotation=(0,0,0)):
        self.centerPosition = position
        self.rotation = rotation

    def getName(self):
        return self.name
    
    def isPointInside(self,point):
        centroidToPoint = point - self.obb.current_centroid
        
        pX = np.absolute(dot(centroidToPoint,self.obb.current_homo[0:3,0]))
        pY = np.absolute(dot(centroidToPoint,self.obb.current_homo[0:3,1]))
        pZ = np.absolute(dot(centroidToPoint,self.obb.current_homo[0:3,2]))
        
        xLength = self.obb.extents[0]*2
        yLength = self.obb.extents[1]*2
        zLength = self.obb.extents[2]*2
        
        if 2*pX <= xLength and 2*pY <= yLength and 2*pZ <= zLength:
            return True
        else:
            return False
        
    def createOBB(self):
        indices = []
        for face in self.obj.faces:
            indices.append(face[0][0] - 1)
            indices.append(face[0][1] - 1)
            indices.append(face[0][2] - 1)
        self.obb = OBB.build_from_triangles(self.obj.vertices, indices)
        self.obb.gl_list = glGenLists(1)
        glNewList(self.obb.gl_list, GL_COMPILE)
        glBegin(GL_LINES)
        glColor3fv((1, 0, 0))

        def input_vertex(x, y, z):
            glVertex3fv(self.obb.transform((x, y, z)))

        input_vertex(*self.obb.max)
        input_vertex(self.obb.max[0], self.obb.min[1], self.obb.max[2])

        input_vertex(self.obb.max[0], self.obb.min[1], self.obb.max[2])
        input_vertex(self.obb.min[0], self.obb.min[1], self.obb.max[2])

        input_vertex(self.obb.min[0], self.obb.min[1], self.obb.max[2])
        input_vertex(self.obb.min[0], self.obb.max[1], self.obb.max[2])

        input_vertex(self.obb.min[0], self.obb.max[1], self.obb.max[2])
        input_vertex(*self.obb.max)

        input_vertex(self.obb.max[0], self.obb.max[1], self.obb.max[2])
        input_vertex(self.obb.max[0], self.obb.max[1], self.obb.min[2])

        input_vertex(self.obb.max[0], self.obb.min[1], self.obb.max[2])
        input_vertex(self.obb.max[0], self.obb.min[1], self.obb.min[2])

        input_vertex(self.obb.min[0], self.obb.max[1], self.obb.max[2])
        input_vertex(self.obb.min[0], self.obb.max[1], self.obb.min[2])

        input_vertex(self.obb.min[0], self.obb.min[1], self.obb.max[2])
        input_vertex(self.obb.min[0], self.obb.min[1], self.obb.min[2])

        input_vertex(self.obb.max[0], self.obb.max[1], self.obb.min[2])
        input_vertex(self.obb.max[0], self.obb.min[1], self.obb.min[2])

        input_vertex(self.obb.max[0], self.obb.min[1], self.obb.min[2])
        input_vertex(*self.obb.min)

        input_vertex(*self.obb.min)
        input_vertex(self.obb.min[0], self.obb.max[1], self.obb.min[2])

        input_vertex(self.obb.min[0], self.obb.max[1], self.obb.min[2])
        input_vertex(self.obb.max[0], self.obb.max[1], self.obb.min[2])
        glEnd()
        glEndList()
        
class OBJ():
    def __init__(self, filename, swap_yz=False):
        self.vertices = []
        self.normals = []
        self.texcoords = []
        self.faces = []
        

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
                # for i in range(len(v)):
                #     v[i] = v[i]*10
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
        self.gl_list = glGenLists(1)
        glNewList(self.gl_list, GL_COMPILE)
        glEnable(GL_TEXTURE_2D)
        glFrontFace(GL_CCW)
        start = time.time()
        for vertices in self.faces:
            vertices, normals, texcoords, material = vertices
            
            glBegin(GL_TRIANGLES)
            glColor3fv((1, 1, 1))
            
            for i in range(len(vertices)):
                if normals[i] > 0:
                    glNormal3fv(self.normals[normals[i] - 1])
                if texcoords[i] > 0:
                    glTexCoord2fv(self.texcoords[texcoords[i] - 1])
                glVertex3fv(self.vertices[vertices[i] - 1])
            glEnd()
        end = time.time()
        glDisable(GL_TEXTURE_2D)
        glEndList()
        # print("time =",end-start)
        
        
from numpy import ndarray, array, asarray, dot, cross, cov, array, finfo, min as npmin, max as npmax
from numpy.linalg import eigh, norm


########################################################################################################################
# adapted from: http://jamesgregson.blogspot.com/2011/03/latex-test.html
########################################################################################################################
class OBB:
    def __init__(self):
        self.rotation = None
        self.min = None
        self.max = None
        
        self.gl_list=None
        self.current_point = None
        self.current_centroid = None
        self.current_homo = None
        
    def transform(self, point):
        return dot(array(point), self.rotation)

    @property
    def centroid(self):
        return self.transform((self.min + self.max) / 2.0)

    @property
    def extents(self):
        return abs(self.transform((self.max - self.min) / 2.0))

    @property
    def points(self):
        return [
            # upper cap: ccw order in a right-hand system
            #       v1________v0
            #       /|       /|
            #      v2_______v3|
            #      | |      | |
            #      |v4______|_v5
            #      |/_______|/
            #      v7       v6
            # rightmost, topmost, farthest,v0
            self.transform((self.max[0], self.max[1], self.min[2])),
            # leftmost, topmost, farthest,v1
            self.transform((self.min[0], self.max[1], self.min[2])),
            # leftmost, topmost, closest,v2
            self.transform((self.min[0], self.max[1], self.max[2])),
            # rightmost, topmost, closest,v3
            self.transform(self.max),
            # lower cap: cw order in a right-hand system
            # leftmost, bottommost, farthest,v4
            self.transform(self.min),
            # rightmost, bottommost, farthest,v5
            self.transform((self.max[0], self.min[1], self.min[2])),
            # rightmost, bottommost, closest,v6
            self.transform((self.max[0], self.min[1], self.max[2])),
            # leftmost, bottommost, closest,v7
            self.transform((self.min[0], self.min[1], self.max[2])),
        ]
        
    @property
    def homo(self):
        homo = np.eye(4)
        homo[0:3,0:3] = self.rotation
        homo[0:3,3] = np.array([0,0,0]).T
        print(homo)
        return homo
    @classmethod
    def build_from_covariance_matrix(cls, covariance_matrix, points):
        if not isinstance(points, ndarray):
            points = array(points, dtype=float)
        assert points.shape[1] == 3

        obb = OBB()

        _, eigen_vectors = eigh(covariance_matrix)

        def try_to_normalize(v):
            n = norm(v)
            if n < finfo(float).resolution:
                raise ZeroDivisionError
            return v / n

        r = try_to_normalize(eigen_vectors[:, 0])
        u = try_to_normalize(eigen_vectors[:, 1])
        f = try_to_normalize(eigen_vectors[:, 2])

        obb.rotation = array((r, u, f)).T

        # apply the rotation to all the position vectors of the array
        # TODO : this operation could be vectorized with tensordot
        p_primes = asarray([obb.rotation.dot(p) for p in points])
        obb.min = npmin(p_primes, axis=0)
        obb.max = npmax(p_primes, axis=0)

        return obb

    @classmethod
    def build_from_triangles(cls, points, triangles):
        for point in points:
            if len(point) != 3:
                raise Exception('points have to have 3-elements')

        weighed_mean = array([0, 0, 0], dtype=float)
        area_sum = 0
        c00 = c01 = c02 = c11 = c12 = c22 = 0
        for i in range(0, len(triangles), 3):
            p = array(points[triangles[i]], dtype=float)
            q = array(points[triangles[i + 1]], dtype=float)
            r = array(points[triangles[i + 2]], dtype=float)
            mean = (p + q + r) / 3.0
            area = norm(cross((q - p), (r - p))) / 2.0
            weighed_mean += mean * area
            area_sum += area
            c00 += (9.0 * mean[0] * mean[0] + p[0] * p[0] + q[0] * q[0] + r[0] * r[0]) * (area / 12.0)
            c01 += (9.0 * mean[0] * mean[1] + p[0] * p[1] + q[0] * q[1] + r[0] * r[1]) * (area / 12.0)
            c02 += (9.0 * mean[0] * mean[2] + p[0] * p[2] + q[0] * q[2] + r[0] * r[2]) * (area / 12.0)
            c11 += (9.0 * mean[1] * mean[1] + p[1] * p[1] + q[1] * q[1] + r[1] * r[1]) * (area / 12.0)
            c12 += (9.0 * mean[1] * mean[2] + p[1] * p[2] + q[1] * q[2] + r[1] * r[2]) * (area / 12.0)

        weighed_mean /= area_sum
        c00 /= area_sum
        c01 /= area_sum
        c02 /= area_sum
        c11 /= area_sum
        c12 /= area_sum
        c22 /= area_sum

        c00 -= weighed_mean[0] * weighed_mean[0]
        c01 -= weighed_mean[0] * weighed_mean[1]
        c02 -= weighed_mean[0] * weighed_mean[2]
        c11 -= weighed_mean[1] * weighed_mean[1]
        c12 -= weighed_mean[1] * weighed_mean[2]
        c22 -= weighed_mean[2] * weighed_mean[2]

        covariance_matrix = ndarray(shape=(3, 3), dtype=float)
        covariance_matrix[0, 0] = c00
        covariance_matrix[0, 1] = c01
        covariance_matrix[0, 2] = c02
        covariance_matrix[1, 0] = c01
        covariance_matrix[1, 1] = c11
        covariance_matrix[1, 2] = c12
        covariance_matrix[2, 0] = c02
        covariance_matrix[1, 2] = c12
        covariance_matrix[2, 2] = c22

        return OBB.build_from_covariance_matrix(covariance_matrix, points)

    @classmethod
    def build_from_points(cls, points):
        if not isinstance(points, ndarray):
            points = array(points, dtype=float)
        assert points.shape[1] == 3, 'points have to have 3-elements'
        # no need to store the covariance matrix
        return OBB.build_from_covariance_matrix(cov(points, y=None, rowvar=0, bias=1), points)