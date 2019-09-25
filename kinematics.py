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
    d6 = 100 # need to check
    Dh = [[0,-np.pi/2,117.75,joint_angles[0]],
          [98.84,0,0,joint_angles[1]-np.pi/2],
          [0,np.pi/2,0,joint_angles[2]+np.pi/2],
          [0,-np.pi/2,113.1,joint_angles[3]],
          [0,np.pi/2,0,joint_angles[4]],
        #   [0,np.pi/2,d6,joint_angles[5]]
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
  
    H = np.identity(4)
    Dh = DH(joint_angles)
    for i in range(link):
        Hc = calH(Dh[i])
        H = np.matmul(H,Hc)
        # print(link-i,Hc)
    # print ('----------')
    ZYZ = get_euler_angles_from_T(H)
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
    theta = np.arctan2((np.sqrt(T[2][0]**2+T[2][1]**2)),T[2][2])
    if theta == 0:
        phi = 0
        psi = np.arctan2(-T[0][1],T[0][0])
    elif theta == np.pi:
        phi = 0
        psi = np.arctan2(T[0][1],-T[0][0])
    else:
        phi = np.arctan2(T[1][2],T[0][2])
        psi = np.arctan2(T[2][1],-T[2][0])
    toReturn = []
    toReturn.append(phi)
    toReturn.append(theta)
    toReturn.append(psi)
    return toReturn

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

def IK(pose):
    
    """
    TODO: implement this function

    Calculate inverse kinematics for rexarm

    return the required joint angles

    """
    #pose = R06 = R
    d6 = 0 # gripper length
    H = pose
    R = np.zeros((3,3))
    for i in range(3):
        for j in range(3):
            R[i][j] = H[i][j]
    o=[H[i][3] for i in range(3)] # desire postion
    print R
    print o
    # o06 =np.matmul(H,[0,0,0,1])
    o04 = o - np.matmul(R,[0,0,d6])
    x4 = o04[0]
    y4 = o04[1]
    z4 = o04[2]
    t11 = np.arctan2(y4,x4) #yc/xc
    t12 = np.pi+np.arctan2(y4,x4)

    ""
    L2 = 98.84
    L3 = 113.1
    t31 = np.arccos((x4**2+y4**2-L2**2-L3**2)/(2*L2*L3))
    t32 = -np.arccos((x4**2+y4**2-L2**2-L3**2)/(2*L2*L3))
    print "theta 3"
    print t31*180/np.pi,t32*180/np.pi

    ""
    gamma1 = np.arctan2(L3*np.sin(t31),L2+L3*np.cos(t31))
    gamma2 = np.arctan2(L3*np.sin(t32),L2+L3*np.cos(t32))
    print "gamma1"
    print gamma1*180/np.pi
    print "gamma2"
    print gamma2*180/np.pi

    t21 = -np.arctan2(z4-117.75,x4)+gamma1
    t22 = -np.arctan2(z4-117.75,x4)+gamma2
    print 'angle'
    print np.arctan2(z4-117.75,x4)*180/np.pi    
    print z4
    print "theta 2"
    print t21*180/np.pi,t22*180/np.pi

    angles_1 = [t11,t21,t31,0,0,0]
    H03 = FK_dh(angles_1,3)[0]
    R03 = np.zeros((3,3))
    for i in range(3):
        for j in range(3):
            R03[i][j] = H03[i][j]
    # print "H03"
    # print np.matmul(H03,[0,0,0,1])
    R36 = np.matmul(np.linalg.inv(R03),R)
    t46 = get_euler_angles_from_T(R36)
    # print t46
    t41 = t46[0]
    t51 = t46[1]
    t61 = t46[2]
    # IK_angle = [[t11,t21,t31,t41,t51]]
    IK_angle = [t11,t21,t31,t41,-np.pi/2]
    De = [i*180/np.pi for i in IK_angle]
    print De
    IK_angle = [[t11,t21,t31,t41,-np.pi/2]]
    return IK_angle
cos=np.cos
sin=np.sin
# A = np.array([[cos(np.pi),0,-sin(np.pi),100],[0,1,0,100],[sin(np.pi),0,cos(np.pi),200],[0,0,0,1]])
# B = np.array([[1,0,0,100],[0,1,0,100],[0,0,1,200],[0,0,0,1]])
A = np.array([[cos(np.pi),0,-sin(np.pi),100],[0,1,0,100],[sin(np.pi),0,cos(np.pi),120],[0,0,0,1]])
print IK(A)
# B=IK(A)
# B=[i*180/np.pi for i in B]
# print B

def to_s_matrix(w,v):
    """
    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)
    """
    pass