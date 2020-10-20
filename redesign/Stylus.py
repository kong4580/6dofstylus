import numpy as np
from math import sin, cos,pi
from scipy.spatial.transform import Rotation as R

class Stylus():
    def __init__(self):
        self.h1 = 53.5
        self.h2 = 80
        
        self.l1 = 60
        self.l2 = 56
        
        
        self.dh = np.matrix([[0 ,self.h1, 0, pi/2],
                             [pi/2, 0, self.h2, 0],
                             [0, 0, 0, pi/2],
                             [0, self.l1+self.l2, 0, -pi/2],
                             [pi/2, 0, 0, -pi/2],
                             [pi/2,0, 0, -pi/2]])
        
        self.rho = [1,1,1,1,1,1]
    
    def transl(self, dist, axis):
        h = np.matrix(np.eye(4))
        if axis == 'x':
            idx = 0
        elif axis == 'y':
            idx = 1
        elif axis == 'z':
            idx = 2
        else:
            print('incorrect axis of translation')
        h[idx,3] = dist
        return h
    
    def rot(self,ang,axis):
        h = np.matrix(np.eye(4))
        c = cos(ang)
        s = sin(ang)
        if c > 0:
            if c <= 1e-10:
                c=0
        else:
            if c >= -1e-10:
                c=0
                
        if s > 0:
            if s <= 1e-10:
                s=0
        else:
            if s >= -1e-10:
                s=0
                
        if axis == 'x':
            h[0:3,0:3] = np.matrix([[1, 0, 0 ],[0, c, -s], [0, s, c]])
        elif axis == 'y':
            h[0:3,0:3] = np.matrix([[c, 0, s] , [0, 1, 0, ], [-s, 0, c]])
        elif axis == 'z':
            h[0:3,0:3] = np.matrix([[c, -s, 0], [s, c, 0] , [0, 0, 1]])
        else:
            print('incorrect axis of rotation')
        
        return h
    
    def forwardKinematics(self, q,wrt=(0,5)):
        rho = self.rho
        dh=self.dh
        n = len(q)
        tformPrev = np.matrix(np.eye(4))
        tforms = np.zeros((n,4,4))
        
        for i in range(wrt[0],wrt[1]+1):
            if rho[i]:
                jointTrans = self.rot(q[i],'z')
            else:
                jointTrans = self.transl(q[i],'z')
            
            dhTrans = jointTrans*self.rot(dh[i,0],'z')*self.transl(dh[i,1],'z')*self.transl(dh[i,2],'x')*self.rot(dh[i,3],'x')
            
            tformPrev = tformPrev * dhTrans
            tforms[i,:,:] = tformPrev
        return tforms
    
    def getEndTransforms(self,q):
        if len(q) != 6:
            return np.eye(4)
        else:
            q= np.array(q)
            
            q[0] = q[0] -pi 
            q[1] = q[1] - pi 
            q[2] = q[2] - pi 
            q[3] = q[3] - pi 
            q[4] = q[4] - pi 
            q[5] = q[5] - pi 
            
            q = q - np.asarray([-0.02147573, -0.02300971, -0.01994175,  0.04141748,  0.02454369, -0.12885439]).T
            
            
            fk = self.forwardKinematics(q.tolist())[-1,:,:]
            
            offsetZ = 0
            x,y,z = fk[0,3],fk[1,3],fk[2,3]+offsetZ
            r = R.from_matrix(fk[0:3,0:3])
            # print(r.as_matrix())
            rx = r.as_euler('xyz')[0]
            ry = r.as_euler('xyz')[1]
            rz = r.as_euler('xyz')[2]
            return fk
    
class Stylus2(Stylus):
    
    def __init__(self):
        super().__init__()
        self.h1 = 80
        self.h2 = 165
        self.l1 =30
        self.l2 = 230
        self.dh = np.matrix([[0 ,self.h1, 0, pi/2],
                             [90*pi/180, 0, self.h2, 0],
                             [0,0,self.l1, 0],
                             [-90*pi/180,0,self.l2, 0]])
                             
    def getEndTransforms(self,q):
        if len(q) != 3:
            return np.eye(4)
        else:
            q= np.array(q)
            q[0] = q[0] -pi 
            q[1] = q[1] - pi 
            q[2] = q[2] - pi 

            q = q - np.asarray([0.16260196, -0.19634954, -0.22089323])
            q = np.append(q,[0])
            print(q)
            fk = self.forwardKinematics(q.tolist(),wrt=(0,3))[-1,:,:]
            print(fk)
            return fk
        
if __name__ == '__main__':
    stylus = Stylus()
    q = [0,0,0,0,0,0]
    print(stylus.forwardKinematics(q))
# a1 =53.5 
# a2 = 80 
# a3 = 0 
# l1 = 0
# l2 = 110 
# le = -48.5 
# # stylus DH table
# # theta d a alpha beta
# dh = np.matrix([[0 ,a1, l1, pi/2,0],
#                 [pi/2, 0, a2, 0,0],
#                 [0, 0, a3, pi/2,0],
#                 [0, l2, 0, -pi/2,0],
#                 [0, 0, 0, pi/2,0],
#                 [0,le, 0, 0,0]])
# dh = np.matrix([[0 ,53.5, 0, -pi/2],
#                 [-pi/2, 0, 80, 0],
#                 [0, 0, 0, pi/2],
#                 [0, 60+50, 0, pi/2],
#                 [pi/2, 0, 0, pi/2],
#                 [0, -48.5, 0, 0]])
# rho = [1,1,1,1,1,1]
# q = [0,0,pi/2,0,0,0]

# def transl(dist,axis):
#     h = np.matrix(np.eye(4))
#     if axis == 'x':
#         idx = 0
#     elif axis == 'y':
#         idx = 1
#     elif axis == 'z':
#         idx = 2
#     else:
#         print('incorrect axis of translation')
#     h[idx,3] = dist
#     return h
# def rot(ang,axis):
#     h = np.matrix(np.eye(4))
#     c = cos(ang)
#     s = sin(ang)
#     if c > 0:
#         if c <= 1e-10:
#             c=0
#     else:
#         if c >= -1e-10:
#             c=0
            
#     if s > 0:
#         if s <= 1e-10:
#             s=0
#     else:
#         if s >= -1e-10:
#             s=0
            
#     if axis == 'x':
#         h[0:3,0:3] = np.matrix([[1, 0, 0 ],[0, c, -s], [0, s, c]])
#     elif axis == 'y':
#         h[0:3,0:3] = np.matrix([[c, 0, s] , [0, 1, 0, ], [-s, 0, c]])
#     elif axis == 'z':
#         h[0:3,0:3] = np.matrix([[c, -s, 0], [s, c, 0] , [0, 0, 1]])
#     else:
#         print('incorrect axis of rotation')
    
#     return h
# def forwardKinematics(q,rho,dh,wrt=(0,5)):
#     n = len(q)
#     tformPrev = np.matrix(np.eye(4))
#     tforms = np.zeros((n,4,4))
    
#     for i in range(wrt[0],wrt[1]+1):
#         if rho[i]:
#             jointTrans = rot(q[i],'z')
#         else:
#             jointTrans = transl(q[i],'z')
        
#         dhTrans = jointTrans*rot(dh[i,0],'z')*transl(dh[i,1],'z')*transl(dh[i,2],'x')*rot(dh[i,3],'x')
        
#         tformPrev = tformPrev * dhTrans
#         tforms[i,:,:] = tformPrev
#     return tforms
# fk = forwardKinematics(q,rho,dh)
# print("forward kinematic:\n",fk)

# def jacobian(q,rho,dh):
#     n = len(q)
#     tform = forwardKinematics(q,rho,dh)
#     jac = np.zeros((n,6,n))
#     for j in range(n):
#         pn = tform[j,0:3,3:4]
#         zPre = np.matrix('0;0;1')
#         pPre = np.matrix('0;0;0')
        
#         for i in range(j+1):
            
#             jacVi = np.cross(rho[i]*zPre,pn-pPre,axis=0) + (1-rho[i])*zPre
#             jacWi = rho[i]*zPre
            
#             jac[j:j+1,:,i:i+1] = np.matrix(np.vstack((jacWi,jacVi)))
#             zPre = tform[i,0:3,2:3]
#             pPre = tform[i,0:3,3:4]
#     return jac
# jac = jacobian(q,rho,dh)
# print("jacobian:\n",jac)

# def inverseJacobian(jac):
    
#     j = np.dot(jac,jac.T)
#     invJac = np.linalg.pinv(j) 
#     invJac = np.dot(jac.T,invJac)
#     return invJac
# # print("inverseJacobian:\n",inverseJacobian(jac[2]))

# # endEfFK = fk[-1]
# # endEfRot = R.from_matrix(endEfFK[0:3,0:3])

# # chi = np.matrix([endEfRot.as_euler('zyx')[0],endEfRot.as_euler('zyx')[1],endEfRot.as_euler('zyx')[2],endEfFK[0,3],endEfFK[1,3],endEfFK[2,3]]).T


# qGoal= [0,0,0,0,0,0]
# def getChi(q,rho,dh):
#     fk = forwardKinematics(q,rho,dh)
#     endEfFK = fk[-1]
#     endEfRot = R.from_matrix(endEfFK[0:3,0:3])

#     # chi = np.matrix([endEfRot.as_euler('zyx')[0],endEfRot.as_euler('zyx')[1],endEfRot.as_euler('zyx')[2],endEfFK[0,3],endEfFK[1,3],endEfFK[2,3]]).T
#     chi = np.matrix([endEfFK[0,3],endEfFK[1,3],endEfFK[2,3],endEfRot.as_euler('xyz')[0],endEfRot.as_euler('xyz')[1],endEfRot.as_euler('xyz')[2]]).T
    
#     return chi
# def numericalIK(qGoal,rho,dh):
#     count=0
#     alpha = 0.01
#     qb = qGoal.copy()
#     jac = jacobian(qb,rho,dh)
#     dchi = getChi(qb,rho,dh)
#     chigoal = np.matrix([[-48.5],[0],[243.5],[0],[pi/2],[pi]])
#     # chigoal = np.matrix([[0],[0],[0],[-48.5],[0],[248.5]])
    
    
#     qbM = np.matrix(qb).T
#     qip1 = np.matrix([0,0,0])
#     dq = 1
#     while(np.linalg.norm(dq)>=1e-2 and count <3000):
#         print("Iter : ",count)
#         if type(qb)==type([]):
#             qbM = np.matrix(qb).T
#         else:
#             qbM = np.matrix(qb)
        
#         qip1 = qbM + alpha*inverseJacobian(jac[-1])*dchi
        
#         jac = jacobian(qip1,rho,dh)
#         dchi = chigoal-getChi(qip1,rho,dh)
#         dq = qip1-qbM
#         qb = qip1.copy()
#         count+=1
#         print('qi:\n',np.round(qb,decimals=5))
#         print('norm of qGoal - qi:\n',np.linalg.norm(dq),'\n\n')
#     return qb
# qi = numericalIK(qGoal,rho,dh)
# print("qGoal = \n",np.round(np.matrix(qGoal).T,decimals=3),'\n')
# print("q from numericalIK = \n",np.round(np.matrix(qi),decimals=3),'\n')
# print('error qGoal - qi = \n',np.round(np.matrix(qGoal).T-qi,decimals=5),'\n')

# print('cartesian qGoal = \n',np.round(forwardKinematics(qGoal,rho,dh)[5,:,:],decimals = 4),'\n')
# print('cartesian from numericalIK = \n',np.round(forwardKinematics(qi,rho,dh)[5,:,:],decimals = 1))
# print('cartesian error = \n',np.round(forwardKinematics(qGoal,rho,dh)[5,:,:]-forwardKinematics(qi,rho,dh)[5,:,:],decimals = 2))

# np.random.uniform(low=0.01, high=10, size=(8,))
