import math
import numpy as np
from OpenGL import GL,GLU
class Ray:
    def __init__(self,P0 = np.asarray([0.,0.,0.]),P1=np.asarray([0.,0.,0.])):
        self.p0 = np.asarray([0.,0.,0.])
        self.p1 = np.asarray([0.,0.,0.])
        P0 = np.asarray(P0)
        P1 = np.asarray(P1)

        if P0.shape == (3,1) or P0.shape == (3,):
            self.p0[0] = P0[0]
            self.p0[1] = P0[1]
            self.p0[2] = P0[2]
            self.p1[0] = P1[0]
            self.p1[1] = P1[1]
            self.p1[2] = P1[2]
        elif P0.shape == () or P0.shape == (1,):
            self.generate3DRay(P0,P1)
        self.vec = self.p1 - self.p0
 
    def generate3DRay(self,x2D,y2D):
        x =x2D
        y =y2D	
        winZ0 = 0
        winZ1 = 1

        modelview = GL.glGetDoublev(GL.GL_MODELVIEW_MATRIX)
        projection = GL.glGetDoublev(GL.GL_PROJECTION_MATRIX)
        viewport = GL.glGetIntegerv(GL.GL_VIEWPORT)		

        winX = x
        winY = viewport[3] - y


        GL.glReadBuffer( GL.GL_BACK )

        posX0, posY0, posZ0 = GLU.gluUnProject( winX, winY, winZ0, modelview, projection, viewport)
        posX1, posY1, posZ1 = GLU.gluUnProject( winX, winY, winZ1, modelview, projection, viewport)
        # flip opengl co-ordinates to flight simulator co-ordinates
        self.p0[0] = posX0
        self.p0[1] = posY0
        self.p0[2] = posZ0
        #
        self.p1[0] = posX1
        self.p1[1] = posY1
        self.p1[2] = posZ1

    def intersects(self,p):
        w = self.p0 - p.Position
        norm = p.Normal
        # self.vec[2] += 0.000001
        checkParallel = np.dot(norm,w)
        # print(round(checkParallel,0),"intwersect")
        if round(checkParallel,0) ==0:
            # print(self.vec,norm,"no intersect")
            return np.asarray([None,None,None])
        # else:
            # print(self.vec,norm,"intersect")
        pNormal = p.Normal.copy()
        D = pNormal[0]*p.Position[0] + pNormal[1]*p.Position[1] + pNormal[2]*p.Position[2]



        t = -((pNormal[0]*self.p1[0] + pNormal[1]*self.p1[1] + pNormal[2]*self.p1[2] - D))/(pNormal[0]*self.vec[0]+pNormal[1]*self.vec[1]+pNormal[2]*self.vec[2])
        x = self.p1[0] + (self.vec[0]*t)
        y = self.p1[1] + (self.vec[1]*t)
        z = self.p1[2] + (self.vec[2]*t)

        # print(pNormal,self.vec)
        I = np.asarray([x,y,z])
        # print(I)
        # if round(y,8) == 5.00000026:
        # #     print("Noneeeee")
        #     print("Noneeeee")
        return I
class Plane:
    def __init__(self,norm,pos):
        self.Normal = np.asarray([0.,0.,0.])
        self.Position = np.asarray([0.,0.,0.])
        self.Normal[0] = norm[0]
        self.Normal[1] = norm[1]
        self.Normal[2] = norm[2]
        self.Position[0] = pos[0]
        self.Position[1] = pos[1]
        self.Position[2] = pos[2]

class Line:
    def __init__(self,direction,point):
        self.direction = np.asarray([0.,0.,0.])
        self.point = np.asarray([0.,0.,0.])
        self.direction[0] = direction[0]
        self.direction[1] = direction[1]
        self.direction[2] = direction[2]
        self.point[0] = point[0]
        self.point[1] = point[1]
        self.point[2] = point[2]

def lineIntersect(plane1,plane2):
    n1 = plane1.Normal
    n2 = plane2.Normal
    p1 = plane1.Position
    p2 = plane2.Position
    d1 = -(n1[0]*p1[0] + n1[1]*p1[1] + n1[2]*p1[2])
    d2 = -(n2[0]*p2[0] + n2[1]*p2[1] + n2[2]*p2[2])
    # find direction vector of the intersection line
    v = np.cross(n1,n2)                   # cross product

    # if |direction| = 0, 2 planes are parallel (no intersect)
    # return a line with NaN
    if(v[0] == 0 and v[1] == 0 and v[2] == 0):
        return Line(np.asarray([Nan,Nan,Nan]), np.asarray([Nan,Nan,Nan]))

    # find a point on the line, which is also on both planes
    # choose simplest plane where d=0: ax + by + cz = 0
    dot = np.dot(v,v)                       # V dot V
    u1 =  d2 * n1                      # d2 * N1
    u2 = -d1 * n2                      #-d1 * N2
    p = np.cross((u1 + u2),(v)) / dot       # (d2*N1-d1*N2) X V / V dot V

    return Line(v, p)
def pointProjectOnLine(line,point):
    dir = line.direction
    P = line.point
    Q = point
    PQ = Q - P
    projPQ = (np.dot(PQ,dir)/(dir[0]*dir[0]+dir[1]*dir[1]+dir[2]*dir[2]))*dir
    s = P + projPQ
    return s

plane1 = Plane([1,0,0],[0,0,0])
plane2 = Plane([0,1,0],[0,0,0])
line1 = lineIntersect(plane1,plane2)
point = pointProjectOnLine(line1,np.asarray([90,15,2]))
print(point)

