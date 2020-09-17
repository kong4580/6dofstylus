import numpy as np
from math import pi, cos, sin,sqrt,atan,atan2



def findM(dh,fk=False):
    theta = dh[:,0]
    d = dh[:,1]
    a = dh[:,2]
    alpha = dh[:,3]
    beta = dh[:,4]
    def findFK(wrt,f):
        tf= np.matrix(np.eye(4))
        if wrt == 5 and f == 5:
            return tf
        for i in range(wrt,f+1):
            Ai = np.matrix([[cos(theta[i])*cos(beta[i])-sin(theta[i])*sin(alpha[i])*sin(beta[i]), -sin(theta[i])*cos(alpha[i]), cos(theta[i])*sin(beta[i])+sin(theta[i])*sin(alpha[i])*cos(beta[i]), a[i].A1[0]*cos(theta[i])],
                            [sin(theta[i])*cos(beta[i])+cos(theta[i])*sin(alpha[i])*sin(beta[i]), cos(theta[i])*cos(alpha[i]), sin(theta[i])*sin(beta[i])-cos(theta[i])*sin(alpha[i])*cos(beta[i]), a[i].A1[0]*sin(theta[i])],
                            [-cos(alpha[i])*sin(beta[i]), sin(alpha[i]), cos(alpha[i])*cos(beta[i]), d[i].A1[0]],
                            [0,0,0,1]])
            tf = tf*Ai
            
        return np.round(tf,decimals=4)
    def findB(gamma,i):
        if gamma == 0: # theta
            B = np.matrix([[-cos(beta[i])*sin(theta[i])-sin(alpha[i])*sin(beta[i])*cos(theta[i]), -cos(alpha[i])*cos(theta[i]), cos(beta[i])*sin(alpha[i])*cos(theta[i])-sin(beta[i])*sin(theta[i]), -a[i].A1[0]*sin(theta[i])],
                        [cos(beta[i])*cos(theta[i])+sin(alpha[i])*sin(beta[i])*sin(theta[i]), -cos(alpha[i])*sin(theta[i]), cos(beta[i])*sin(alpha[i])*sin(theta[i])+sin(beta[i])*cos(theta[i]), -a[i].A1[0]*cos(theta[i])],
                        [0,0,0,0],
                        [0,0,0,0]])
        elif gamma == 1: # d
            B = np.matrix([[0,0,0,0],
                        [0,0,0,0],
                        [0,0,0,1],
                        [0,0,0,0]])
        elif gamma == 2: # a
            B = np.matrix([[0,0,0,cos(theta[i])],
                        [0,0,0,sin(theta[i])],
                        [0,0,0,0],
                        [0,0,0,0]])
        elif gamma == 3: # alpha
            B = np.matrix([[-cos(alpha[i])*sin(beta[i])*sin(theta[i]), sin(alpha[i])*sin(theta[i]), cos(alpha[i])*cos(beta[i])*sin(theta[i]),0],
                        [-cos(alpha[i])*sin(beta[i])*cos(theta[i]), -sin(alpha[i])*cos(theta[i]),-cos(alpha[i])*cos(beta[i])*cos(theta[i]),0],
                        [sin(alpha[i])*sin(beta[i]),cos(alpha[i]),-cos(beta[i])*sin(alpha[i]),0],
                        [0,0,0,0]])
        elif gamma == 4:# beta
            B = np.matrix([[-sin(beta[i])*cos(theta[i])-sin(alpha[i])*cos(beta[i])*sin(theta[i]), 0, -sin(beta[i])*sin(alpha[i])*sin(theta[i])+cos(beta[i])*cos(theta[i]), 0],
                        [-sin(beta[i])*sin(theta[i])-sin(alpha[i])*cos(beta[i])*cos(theta[i]), 0, sin(beta[i])*sin(alpha[i])*cos(theta[i])+cos(beta[i])*sin(theta[i]), 0],
                        [-cos(alpha[i])*cos(beta[i]),0,-cos(alpha[i])*sin(beta[i]),0],
                        [0,0,0,0]])
        return B

    
    Mtheta=np.matrix(np.zeros((3,6)))
    Md=np.matrix(np.zeros((3,6)))
    Ma=np.matrix(np.zeros((3,6)))
    Malpha=np.matrix(np.zeros((3,6)))
    Mbeta=np.matrix(np.zeros((3,6)))
    for i in range(0,6):
        Mtheta[:,i] = np.matrix([[np.dot(np.dot(findFK(0,i-1),findB(gamma = 0,i=i)),findFK(i+1,5))[0,3]],
                            [np.dot(np.dot(findFK(0,i-1),findB(gamma = 0,i=i)),findFK(i+1,5))[1,3]],
                            [np.dot(np.dot(findFK(0,i-1),findB(gamma = 0,i=i)),findFK(i+1,5))[2,3]]])
        Md[:,i] = np.matrix([[np.dot(np.dot(findFK(0,i-1),findB(gamma = 1,i=i)),findFK(i+1,5))[0,3]],
                            [np.dot(np.dot(findFK(0,i-1),findB(gamma = 1,i=i)),findFK(i+1,5))[1,3]],
                            [np.dot(np.dot(findFK(0,i-1),findB(gamma = 1,i=i)),findFK(i+1,5))[2,3]]])
        Ma[:,i] = np.matrix([[np.dot(np.dot(findFK(0,i-1),findB(gamma = 2,i=i)),findFK(i+1,5))[0,3]],
                            [np.dot(np.dot(findFK(0,i-1),findB(gamma = 2,i=i)),findFK(i+1,5))[1,3]],
                            [np.dot(np.dot(findFK(0,i-1),findB(gamma = 2,i=i)),findFK(i+1,5))[2,3]]])
        Malpha[:,i] = np.matrix([[np.dot(np.dot(findFK(0,i-1),findB(gamma = 3,i=i)),findFK(i+1,5))[0,3]],
                            [np.dot(np.dot(findFK(0,i-1),findB(gamma = 3,i=i)),findFK(i+1,5))[1,3]],
                            [np.dot(np.dot(findFK(0,i-1),findB(gamma = 3,i=i)),findFK(i+1,5))[2,3]]])
        Mbeta[:,i] = np.matrix([[np.dot(np.dot(findFK(0,i-1),findB(gamma = 4,i=i)),findFK(i+1,5))[0,3]],
                            [np.dot(np.dot(findFK(0,i-1),findB(gamma = 4,i=i)),findFK(i+1,5))[1,3]],
                            [np.dot(np.dot(findFK(0,i-1),findB(gamma = 4,i=i)),findFK(i+1,5))[2,3]]])
    if fk:
        # print("fk: ",np.round(findFK(0,5)))
        return findFK(0,5)
    return Mtheta,Md,Ma,Malpha,Mbeta
def computeIK(tf, offset = (0,0,0)):
    global a1
    global a2
    global a3
    global l1
    global l2
    global le
    
    gr06 = tf[0:3, 0:3]
    gw = tf[0:3,3]+np.dot(gr06,np.matrix([0+offset[0], 0+offset[1], - le+offset[2]]).T)
    x = gw[0]
    y = gw[1]
    z = gw[2]
    sign = np.matrix('1 1 1;1 1 -1;1 -1 1;1 -1 -1;-1 1 1;-1 1 -1;-1 -1 1;-1 -1 -1')
    p_q = np.zeros((6,8))
    for i in range(8):
        q = np.zeros(6)
        sign1 = sign[i,0]
        sign2 = sign[i,1]
        sign3 = sign[i,2]
        try:
            R = sign1*sqrt(x**2+y**2)
            k = ((R-  l1)**2+(z- a1)**2-  l2**2- a3**2- a2**2)/(2* a2)
            q[2] = 2*atan((-2*  l2+sign2*sqrt(4*  l2**2+4*( a3+k)*( a3-k)))/(-2*( a3+k)))
            inv = np.matrix([[-(  l2*sin(q[2])+ a3*cos(q[2])+ a2),   l2*cos(q[2])- a3*sin(q[2]) ],[   l2*cos(q[2])- a3*sin(q[2]),   l2*sin(q[2])+ a3*cos(q[2])+ a2]])
            s2c2 = np.matrix(np.linalg.inv(inv)*np.matrix([R-  l1, z- a1]).T)
            q[1] = atan2(s2c2[0], s2c2[1])
            q[0] = atan2(y/(  l1+  l2*cos(q[1]+q[2])- a3*sin(q[1]+q[2])- a2*sin(q[1])), x/(  l1+  l2*cos(q[1]+q[2])- a3*sin(q[1]+q[2])- a2*sin(q[1])))
            
            
            gr03 = np.matrix([[-sin(q[1]+q[2])*cos(q[0]), sin(q[0]), cos(q[1]+q[2])*cos(q[0])],
                    [-sin(q[1]+q[2])*sin(q[0]), -cos(q[0]), cos(q[1]+q[2])*sin(q[0])],
                    [cos(q[1]+q[2]), 0, sin(q[1]+q[2])]])
            gr = gr03.T*gr06
            c5 = gr[2, 2]
            s5 = sign3*sqrt(gr[0,2]**2+gr[1,2]**2)
            q[4] = atan2(s5, c5)
            if s5 != 0:
                s6 = gr[2,1]/s5
                c6 = -gr[2,0]/s5
                q[5] = atan2(s6, c6)
            
                c4 = gr[0,2]/s5
                s4 = gr[1,2]/s5
                q[3] = atan2(s4, c4)
            else:
                q[3] = 0
                q[5] = 0
        except:
            nan = np.nan
            q = np.matrix([nan, nan, nan, nan, nan, nan]).T
        
        # for j in range(6):
        #     if (q[j]< joint_limit[j,0] or q[j]> joint_limit[j,1]):
        #         state = 0
        #         break
        #     else:
        #         state = 1
        state = 1
        if state == 1:
            p_q[:,i]=q.T
        else:
            nan = np.nan
            nanm = np.matrix([nan, nan, nan, nan, nan, nan])
            p_q[:,i]=nanm
    
    lastq = p_q[:,~np.all(np.isnan(p_q), axis=0)]
    if type(lastq).__name__ is not 'NoneType':
        
        lastq = lastq.reshape(6, len(lastq[0]))
        lastq = lastq.T
        return lastq
    else:
        return None

a1 =53.5 
a2 = 80 
a3 = 0 
l1 = 0
l2 = 110 
le = -48.5 
# stylus DH table
# theta d a alpha beta
dh = np.matrix([[0 ,a1, l1, pi/2,0],
                [pi/2, 0, a2, 0,0],
                [0, 0, a3, pi/2,0],
                [0, l2, 0, -pi/2,0],
                [0, 0, 0, pi/2,0],
                [0,le, 0, 0,0]])
# error matrix
# theta;d;a;alpha;beta
err = np.matrix([[0.05,0.05,0.05,0.05,0.05,0.05],
                 [1,1,2,1,1,2],
                 [0,0,0,0,0,0],
                 [0,0,0,0,0,0],
                 [0,0,0,0,0,0]]).T

dht1 = dh.copy()
dht2 = dh.copy()

# add error
dhe = dh + err


tf1 = np.matrix([[0,0,1,61.5],
                [0,-1,0,0],
                [1,0,0,133.5],
                [0,0,0,1]])
tf2 = np.matrix([[0,0,1,11.5],
                [0,-1,0,0],
                [1,0,0,133.5],
                [0,0,0,1]])
q1 = np.matrix(computeIK(tf1)[0])
q2 = np.matrix(computeIK(tf2)[0])
dht1[:,0] = dht1[:,0]+q1.T
dht2[:,0] = dht2[:,0]+q1.T
count = 0
# # Input q
# q1 = np.matrix([0,0,0,0,0,0])
# # Input q
# q2 = np.matrix([0,0,0,0,0,0])
# assume no error from reading sensor
# add q col0
while(count < 10):
    dh1 = dhe.copy()
    dh2 = dhe.copy()

    dh1[:,0] = dh1[:,0]+q1.T

    # add q col0
    dh2[:,0] = dh2[:,0]+q2.T


    Mtheta1,Md1,Ma1,Malpha1,Mbeta1 = findM(dh1)
    Mtheta2,Md2,Ma2,Malpha2,Mbeta2 = findM(dh2)


    # print(phi.shape)
    Y = (findM(dh1,fk=True)[0:3,3].reshape(3,1)-findM(dh2,fk=True)[0:3,3].reshape(3,1)) - (tf1[0:3,3]-tf2[0:3,3])
    # print(Y)
    for joint in range(6):
        phi = np.array([Mtheta1[:,joint]-Mtheta2[:,joint],
                    Md1[:,joint]-Md2[:,joint],
                    Ma1[:,joint]-Ma2[:,joint],
                    Malpha1[:,joint]-Malpha2[:,joint],
                    Mbeta1[:,joint]-Mbeta2[:,joint]])
        

        phi = phi.squeeze().T
        ch = np.dot(phi.T,phi)
        chinv = np.linalg.pinv(ch)
        chdotT = np.dot(chinv,phi.T)
        X = np.dot(chdotT,Y)
        # print(joint,X)
        dhe[joint,0] = -dhe[joint,0]+X[0]
        dhe[joint,1] = -dhe[joint,1]+X[1]
        dhe[joint,2] = -dhe[joint,2]+X[2]
        dhe[joint,3] = -dhe[joint,3]+X[3]
        dhe[joint,4] = -dhe[joint,4]+X[4]
    print("\nIter ",count)
    count+=1
    print("fk",findM(dhe,fk=True))
