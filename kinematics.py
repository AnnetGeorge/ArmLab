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
    Dh = [[0,-np.pi/2,117.75,joint_angles[0]],
          [98.84,0,0,joint_angles[1]-np.pi/2],
          [0,np.pi/2,0,joint_angles[2]+np.pi/2],
          [0,-np.pi/2,113.1,joint_angles[3]]
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




def IK(pose):
    """
    TODO: implement this function

    Calculate inverse kinematics for rexarm

    return the required joint angles

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
        phi = np.arctan2(T[1][2]/np.sin(theta),T[0][2]/np.sin(theta))
        psi = np.arctan2(T[2][1]/np.sin(theta),-T[2][0]/np.sin(theta))
    toReturn = []
    toReturn.append(phi)
    toReturn.append(theta)
    toReturn.append(psi)
    return toReturn

print FK_dh([1.22,np.pi/3,0.2,0.3,0,4],4)[0]
def get_pose_from_T(T):
    """
    TODO: implement this function
    return the joint pose from a T matrix
    of the form (x,y,z,phi) where phi is rotation about base frame y-axis
    
    """
    pass




def to_s_matrix(w,v):
    """
    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)
    """
    pass