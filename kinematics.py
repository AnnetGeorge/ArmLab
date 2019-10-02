import numpy as np
#expm is a matrix exponential function
#from scipy.linalg import expm

""" 
TODO: Here is where you will write all of your kinematics functions 
There are some functions to start with, you may need to implement a few more
"""


def DH(joint_angles):
    # Dh = [a alpha d theta]
    Dh = np.zeros((len(joint_angles),4))
    d6 = 108.4 # need to check
    Dh = [[0,-np.pi/2,117.75,joint_angles[0]],
          [98.84,0,0,joint_angles[1]-np.pi/2],
          [0,np.pi/2,0,joint_angles[2]+np.pi/2],
          [0,-np.pi/2,113.1,joint_angles[3]],
          [0,np.pi/2,0,joint_angles[4]],
          [0,0,d6,joint_angles[5]]
          ]

    return Dh

def calH(Dh):
    # Dh = [a alpha d theta]
    cos = np.cos
    sin = np.sin
    a = Dh[0]
    alpha = Dh[1]
    d = Dh[2]
    theta = Dh[3]
    A = np.array([[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
                 [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                 [0, sin(alpha), cos(alpha), d],
                 [0,0,0,1]])
    return A

np.set_printoptions(precision=3)

def FK_dh(joint_angles, link):
    """
    TODO: implement this function

    Calculate forward kinematics for rexarm using DH convention

    return a transformation matrix representing the pose of the 
    desired link

    note: phi is the euler angle about the y-axis in the base frame

    """
    # joint_angles = [base,shld,elbw,wrst,wrst2]
    # joint_angles = [base,shld,elbw,wrst,wrst2,wrst3] after 0929
    H = np.identity(4)
    Dh = DH(joint_angles)
    for i in range(link):
        Hc = calH(Dh[i])
        H = np.matmul(H,Hc)
        # print(link-i,Hc)
    # print ('----------')
    ZYZ = get_euler_angles_from_T(H)[0]
    return H,ZYZ


def FK_pox(joint_angles):
    """
    TODO: implement this function

    Calculate forward kinematics for rexarm
    using product of exponential formulation

    return a 4-tuple (x, y, z, phi) representing the pose of the 
    desired link

    note: phi is the euler angle about y in the base frame

    """
    pass


def get_euler_angles_from_T(T):
    """
    TODO: implement this function
    return the Euler angles from a T matrix
    
    """
    #(phi,theta,psi)
    cos=np.cos
    sin=np.sin
    theta = np.arctan2((np.sqrt(T[2][0]**2+T[2][1]**2)),T[2][2])
    if theta == 0:
        phi = 0
        psi = np.arctan2(-T[0][1],T[0][0])
    elif theta == np.pi:
        phi = 0
        psi = np.arctan2(T[0][1],-T[0][0])
    else:
        phi = np.arctan2(T[1][2]/sin(theta),T[0][2]/sin(theta))
        psi = np.arctan2(T[2][1]/sin(theta),-T[2][0]/sin(theta))
    toReturn = []
    toReturn.append(phi)
    toReturn.append(theta)
    toReturn.append(psi)

    theta2 = np.arctan2((-np.sqrt(1-T[2][2]**2)),T[2][2])
    if theta2 == 0:
        phi2 = 0
        psi2 = np.arctan2(-T[0][1],T[0][0])
    elif theta2 == np.pi:
        phi2 = 0
        psi2 = np.arctan2(T[0][1],-T[0][0])
    else:
        phi2 = np.arctan2(T[1][2]/sin(theta2),T[0][2]/sin(theta2))
        psi2 = np.arctan2(T[2][1]/sin(theta2),-T[2][0]/sin(theta2))
    toReturn2 = []
    toReturn2.append(phi2)
    toReturn2.append(theta2)
    toReturn2.append(psi2)

    return toReturn,toReturn2

# Q = list(FK_dh([0,0,0,0,0],4)[1])
# R2D = 180.0/np.pi
# Q = [R2D * q_i for q_i in Q]
# print Q

def get_pose_from_T(T):
    """
    TODO: implement this function
    return the joint pose from a T matrix
    of the form (x,y,z,phi) where phi is rotation about base frame y-axis
    
    """
    pass

def Gripperpose(H,a,b,c):
    a=a*np.pi/180
    b=b*np.pi/180
    c=c*np.pi/180
    cos=np.cos
    sin=np.sin
    R = np.array([[cos(a)*cos(b)*cos(c)-sin(a)*sin(c), -cos(a)*cos(b)*sin(c)-sin(a)*cos(c), cos(a)*sin(b), 0.0],
                  [sin(a)*cos(b)*cos(c)+cos(a)*sin(c), -sin(a)*cos(b)*sin(c)+cos(a)*cos(c), sin(a)*sin(b), 0.0],
                  [-sin(b)*cos(c), sin(b)*sin(c), cos(b), 0.0],
                  [0.0, 0.0, 0.0, 1]])
    # print H
    # print R
    Hp = np.matmul(H,R)
    return Hp

def IK(pose):
    
    """
    TODO: implement this function

    Calculate inverse kinematics for rexarm

    return the required joint angles

    """
    #pose = R06 = R
    d6 = 108.4 # gripper length
    H = pose
    R = np.zeros((3,3))
    for i in range(3):
        for j in range(3):
            R[i][j] = H[i][j]
    o=[H[i][3] for i in range(3)] # desire postion
    # print R
    # print o
    # o06 =np.matmul(H,[0,0,0,1])
    o04 = o - np.matmul(R,[0,0,d6])
    # print 'o04',o04
    x4 = o04[0]
    y4 = o04[1]
    z4 = o04[2]
    t11 = np.arctan2(y4,x4) #yc/xc
    t12 = np.pi+np.arctan2(y4,x4)
    "cal theta 3 2 sols"
    o01z = 117.75
    do01o04 = np.sqrt(x4**2+y4**2+(z4-o01z)**2) #distance o01 o04
    # print 'd14',do01o04
   
    L2 = 98.84
    L3 = 113.1
    fix31 = (do01o04**2-L2**2-L3**2)/(2*L2*L3)
    # print 'beforefix',fix31
    
    if fix31 > 1.0+0.0000001:
        fix31 = 1.0
        print "=======unreachable========1"
        print 'afterfix',fix31
        IK_angle = None
        return IK_angle
    elif fix31 <-1.0-0.0000001:
        fix31 = -1.0
        print "=======unreachable========2"
        print 'afterfix',fix31
        IK_angle = None
        return IK_angle
    if fix31>1:
        fix31 = 1.0
    t31 = np.arccos(fix31)
    t32 = -np.arccos(fix31)
    # print "t31",t31*180/np.pi,"t32",t32*180/np.pi
    # convert to real t31 *-1
    t31r = -np.arccos(fix31)
    t32r = +np.arccos(fix31)
    # print "t31r",t31r*180/np.pi,"t32r",t32r*180/np.pi

    "cal theta 2 sol depends on theta3 2 sols"
    if np.abs(t12)>=np.abs(t11):
        t1f = t11
    else:
        t1f = t12

    #theta 2 = gamma - psi
    angleslink = [t1f,0,0,0,0,0]
    H01 = FK_dh(angleslink,1)[0]
    # print 'H01',H01
    o14 = np.append (o04,[1.0])
    o14 = np.matmul(np.linalg.inv(H01),o14)
    # print 'o14',o14
    gamma = np.arctan2(o14[1],o14[0]) 
    # print "gamma",gamma*180/np.pi
    fixpsi = (L2**2+do01o04**2-L3**2)/(2*L2*do01o04)
    # print "beforefixpsi",fixpsi
    if fixpsi > 1.0+0.000001:
        fixpsi = 1.0
        print "=======unreachable========3"
        # print 'afterfixpsi',fixpsi
        IK_angle = None
        return IK_angle
    elif fixpsi <-1.0-0.000001:
        fixpsi = -1.0
        print "=======unreachable========4"
        # print 'afterfixpsi',fixpsi
        IK_angle = None
        return IK_angle
    psi = np.arccos(fixpsi)
    # print "psi",psi*180/np.pi
    if t31 < 0.0:
        t21 = gamma-psi #check!~
        t21r = 0.5*np.pi+t21
        # print "t21",t21*180/np.pi,"t21r",t21r*180/np.pi
    elif t31 >= 0.0:
        t21 = gamma+psi
        t21r = 0.5*np.pi+t21
        # print "t22",t22*180/np.pi,"t21r",t21r*180/np.pi

    if t32 < 0.0:
        t22 = gamma-psi #check!~
        t22r = 0.5*np.pi+t22
        # print "t21",t21*180/np.pi,"t21r",t21r*180/np.pi
    elif t32 >= 0.0:
        t22 = gamma+psi
        t22r = 0.5*np.pi+t22
    
    "t31 "
    angles_1 = [t1f,t21r,t31r,0,0,0]
    H03 = FK_dh(angles_1,3)[0]
    R03 = np.zeros((3,3))
    for i in range(3):
        for j in range(3):
            R03[i][j] = H03[i][j]
    # print "H03"
    # print np.matmul(H03,[0,0,0,1])
    R36 = np.matmul(np.linalg.inv(R03),R)
    "ZYZ has 2 sol here, use if to chose"
    t46 = get_euler_angles_from_T(R36)[0]
    if (t46[0]>2.62) or (t46[0]<-2.62) or (t46[2]>2.62) or (t46[2]<-2.62):
        t46 = get_euler_angles_from_T(R36)[1]

    t41 = t46[0]
    t51 = t46[1]
    t61 = t46[2]

    # IK_angle = [t11,t21r,t32r,t41,t51,t61]
    IK_angle = [t1f,t21r,t31r,t41,t51,t61]
    P=[0,0,0,1]
    Hq = FK_dh(IK_angle,6)[0]
    worldf = np.matmul(Hq,P)
    

    "t32"
    angles_1 = [t1f,t22r,t32r,0,0,0]
    H032 = FK_dh(angles_1,3)[0]
    R032 = np.zeros((3,3))
    for i in range(3):
        for j in range(3):
            R032[i][j] = H032[i][j]
    # print "H03"
    # print np.matmul(H03,[0,0,0,1])
    R362 = np.matmul(np.linalg.inv(R032),R)
    "ZYZ has 2 sol here, use if to chose"
    t462 = get_euler_angles_from_T(R362)[0]
    if (t462[0]>2.62) or (t462[0]<-2.62) or (t462[2]>2.62) or (t462[2]<-2.62):
        t462 = get_euler_angles_from_T(R362)[1]

    t42 = t462[0]
    t52 = t462[1]
    t62 = t462[2]

    IK_angle2 = [t1f,t22r,t32r,t42,t52,t62]
    P=[0,0,0,1]
    Hq2 = FK_dh(IK_angle2,6)[0]
    worldf2 = np.matmul(Hq2,P)

    
    for i in IK_angle2:
        if np.abs(i)>2.62:
            print ("IK2*************************")
            print ("IK2 in ",worldf2)
            De = [i*180/np.pi for i in IK_angle2]
            print De
            IK_angle = [t1f,t21r,t31r,t41,t51,t61]
             #[[]]
            return IK_angle
    print ("IK in ",worldf)
    De = [i*180/np.pi for i in IK_angle]
    print De
    IK_angle2 = [t1f,t22r,t32r,t42,t52,t62]
     #[[]]
    return IK_angle2
cos=np.cos
sin=np.sin
# A = np.array([[cos(np.pi),0,-sin(np.pi),100],[0,1,0,100],[sin(np.pi),0,cos(np.pi),200],[0,0,0,1]])
# B = np.array([[1,0,0,174],[0,1,0,-2],[0,0,1,75],[0,0,0,1]])
# B = Gripperpose(B,-8,-165,-10)
B = np.array([[1,0,0,78],[0,1,0,-191],[0,0,1,88],[0,0,0,1]])
B = Gripperpose(B,-102,154,99)

# # A = np.array([[cos(np.pi),0,-sin(np.pi),100],[0,1,0,100],[sin(np.pi),0,cos(np.pi),120],[0,0,0,1]])
IK(B)
# aa=IK(B)
# B=[i*180/np.pi for i in B]
# print B

def to_s_matrix(w,v):
    """
    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)
    """
    pass