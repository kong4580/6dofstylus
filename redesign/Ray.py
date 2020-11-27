import math
import numpy as np
from OpenGL import GL,GLU
class Ray:
    def __init__(self,P0 = np.asarray([0.,0.,0.]),P1=np.asarray([0.,0.,0.])):

        # init point for ray
        # point0
        self.p0 = np.asarray([0.,0.,0.])

        # point1
        self.p1 = np.asarray([0.,0.,0.])
        P0 = np.asarray(P0)
        P1 = np.asarray(P1)

        # if giving 3D position for each point (x,y,z)
        if P0.shape == (3,1) or P0.shape == (3,):

            # set value for point0
            self.p0[0] = P0[0]
            self.p0[1] = P0[1]
            self.p0[2] = P0[2]

            # set value for point1
            self.p1[0] = P1[0]
            self.p1[1] = P1[1]
            self.p1[2] = P1[2]
        
        # if giving mouse position (x,y) on screen
        elif P0.shape == () or P0.shape == (1,):

            # generate point0 and point1 from mouse position 
            self.generate3DRay(P0,P1)

        # vector from point0 to point1
        self.vec = self.p1 - self.p0
 
    def generate3DRay(self,x2D,y2D):

        # mouse position
        x =x2D
        y =y2D

        # modelview matrix
        modelview = GL.glGetDoublev(GL.GL_MODELVIEW_MATRIX)

        # projection matrix
        projection = GL.glGetDoublev(GL.GL_PROJECTION_MATRIX)

        # viewport matrix
        viewport = GL.glGetIntegerv(GL.GL_VIEWPORT)		

        # specify the window coordinates to be mapped.
        winX = x
        winY = viewport[3] - y
        winZ0 = 0
        winZ1 = 1

        GL.glReadBuffer( GL.GL_BACK )

        # computed coordinate
        posX0, posY0, posZ0 = GLU.gluUnProject( winX, winY, winZ0, modelview, projection, viewport)
        posX1, posY1, posZ1 = GLU.gluUnProject( winX, winY, winZ1, modelview, projection, viewport)

        # set value for point0
        self.p0[0] = posX0
        self.p0[1] = posY0
        self.p0[2] = posZ0
        # set value for point1
        self.p1[0] = posX1
        self.p1[1] = posY1
        self.p1[2] = posZ1

    def intersects(self,p):

        # vector w from point on plane to point0
        w = self.p0 - p.Position

        # plane normal
        norm = p.Normal

        # dot product plane normal and vector w
        checkParallel = np.dot(norm,w)

        # if dot product equal 0 it is parallel
        if round(checkParallel,0) ==0:
            return np.asarray([None,None,None])
        pNormal = p.Normal.copy()

        # Plane equation
        # Ax + By + Cz = D
        # D = Aa + Bb + Cc
        # where <x,y,z> is position we want to find on plane
        # <A,B,C> is plane normal
        # <a,b,c> is init point on plane  

        # Line equation
        # x = x0 + Qt
        # y = y0 + Rt
        # z = z0 + St
        # where <x,y,z> is position we want to find on line
        # <x0,y0,z0> is init point on line
        # <Q,R,S> is line vector/direction of line

        # combine together
        # A(x0 + Qt) + B(y0 + Rt) + C(z0 + St) - D = 0
        # find t for the equation
        # t = -((Ax0 + By0 + Cz0 - D)/(AQ + BR + CS))

        D = pNormal[0]*p.Position[0] + pNormal[1]*p.Position[1] + pNormal[2]*p.Position[2]
        
        t = -((pNormal[0]*self.p1[0] + pNormal[1]*self.p1[1] + pNormal[2]*self.p1[2] - D))/(pNormal[0]*self.vec[0]+pNormal[1]*self.vec[1]+pNormal[2]*self.vec[2])

        # get the coordinate from line equation
        x = self.p1[0] + (self.vec[0]*t)
        y = self.p1[1] + (self.vec[1]*t)
        z = self.p1[2] + (self.vec[2]*t)

        # plane and line intersecting coordinate
        I = np.asarray([x,y,z])

        # return coordinate
        return I

    def sphereIntersect(self,center,radius):

        # Sphere equation
        # || x - c ||^2 = r^2
        # x is 3D position that we want to find
        # c is 3D position of the center
        # r is circle radius

        # Line equation
        # x = o + du
        # x is 3D position that we want to find
        # o is init point online
        # u is vector of line

        # direction of line
        u = self.vec

        # origin of line
        o = self.p1

        # center of sphere
        c = np.asarray(center)

        # radius of sphere
        r = radius

        # combine line equation and sphere equation
        # || o + du -c||^2 = r^2
        # (o + du - c) * (o + du - c) = r^2
        # d^2(u*u) + 2d(u*(o-c)) + (o-c)*(o-c) - r^2 = 0
        # find d
        # if a = u*u
        #    b = 2(u*(o-c))
        #    c = (o-c)*(o-c) - r^2
        # d = (-b + sqrt(b^2 - 4ac))/2a
        # (get only positive value)

        a = np.dot(u,u)

        b = 2*(np.dot(u,o-c))

        c = np.dot(o-c,o-c) - (r*r)

        # check if the line intersect the sphere or not
        if (b*b) - 4*a*c < 0:
            return np.asarray([None,None,None])

        d = (-b + math.sqrt((b*b) - 4*a*c))/(2*a)

        # get coordinate from line equation
        x = self.p1[0] + (self.vec[0]*d)
        y = self.p1[1] + (self.vec[1]*d)
        z = self.p1[2] + (self.vec[2]*d)

        # line and sphere intersection
        I = np.asarray([x,y,z])

        return I

class Plane:

    def __init__(self,norm,pos):
        # set plane init normal
        self.Normal = np.asarray([0.,0.,0.])

        # set plane init position
        self.Position = np.asarray([0.,0.,0.])

        # set value
        self.Normal[0] = norm[0]
        self.Normal[1] = norm[1]
        self.Normal[2] = norm[2]
        self.Position[0] = pos[0]
        self.Position[1] = pos[1]
        self.Position[2] = pos[2]

class Line:
    def __init__(self,direction,point):
        # set line init direction 
        self.direction = np.asarray([0.,0.,0.])

        #set line init point
        self.point = np.asarray([0.,0.,0.])

        #set value
        self.direction[0] = direction[0]
        self.direction[1] = direction[1]
        self.direction[2] = direction[2]
        self.point[0] = point[0]
        self.point[1] = point[1]
        self.point[2] = point[2]