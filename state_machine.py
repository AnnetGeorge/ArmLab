import time
import numpy as np
import kinematics as kp
import math
import cv2

"""
TODO: Add states and state functions to this class
        to implement all of the required logic for the armlab
"""
class StateMachine():
    def __init__(self, rexarm, planner, kinect):
        self.rexarm = rexarm
        self.tp = planner
        self.kinect = kinect
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.waypoints = []
        self.IKtest = []
        

    def set_next_state(self, state):
        self.next_state = state

    """ This function is run continuously in a thread"""

    def run(self):
        if(self.current_state == "manual"):
            if (self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()                
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "task4"):
                self.task4()
            if(self.next_state == "limp"):
                self.current_state = "limp"

        if(self.current_state == "idle"):
            if(self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "calibrate"):
                self.calibrate()
            if(self.next_state == "execute"):
                self.execute()
            if(self.next_state == "task4"):
                self.task4()
            if(self.next_state == "limp"):
                self.current_state = "limp"
            if (self.next_state == "teaching"):
                self.waypoints = []
                self.teaching()

        if(self.current_state == "limp"):
            if(self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "calibrate"):
                self.calibrate()
            if(self.next_state == "execute"):
                self.execute()
            if(self.next_state == "task4"):
                self.task4()
            if (self.next_state == "teaching"):
                self.waypoints = []
                self.teaching()
            self.rexarm.disable_torque()
            self.status_message = "State: Limp(idle) - motors have been disbled"

        if(self.current_state == "estop"):
            self.next_state = "estop"
            self.estop()  

        if(self.current_state == "execute"):
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "execute"):
                self.execute()
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "limp"):
                self.current_state = "limp"

        if(self.current_state == "calibrate"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "limp"):
                self.current_state = "limp"

        if (self.current_state == "teaching"):
            if (self.next_state == "teaching"):
                self.teaching()
            if(self.next_state == "execute"):
                self.execute()
            if(self.next_state == "limp"):
                self.current_state = "limp"
               

    """Functions run for each state"""


    def manual(self):
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"
        self.rexarm.send_commands()
        self.rexarm.get_feedback()

    def idle(self):
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"
        self.rexarm.get_feedback()

    def estop(self):
        self.status_message = "EMERGENCY STOP - Check Rexarm and restart program"
        self.current_state = "estop"
        self.rexarm.disable_torque()
        self.rexarm.get_feedback()
    
    def execute(self):
        self.status_message = "State: Running Click to Grab"
        self.current_state = "execute"
        self.rexarm.enable_torque()
        sourcePoint = None
        destPoint = None
        self.rexarm.get_feedback()

        self.kinect.new_click = False
        while(not (self.kinect.new_click == True)):
            self.rexarm.get_feedback()  
        self.kinect.new_click = False
        sourcePoint = self.kinect.last_click.copy()
        x_w_src,y_w_src,z_w_src,_ = self.kinect.ijToXyz(sourcePoint[0],sourcePoint[1])
        # x_w_src,y_w_src,z_w_src = (100, 100, 70)

        while(not (self.kinect.new_click == True)):
            self.rexarm.get_feedback()
        self.kinect.new_click = False
        destPoint = self.kinect.last_click.copy()
        x_w_dest,y_w_dest,z_w_dest,_ = self.kinect.ijToXyz(destPoint[0],destPoint[1])
        # x_w_dest,y_w_dest,z_w_dest = (-100, -100, 70)

        if(z_w_dest < 50):
            z_w_dest = 50
        if(z_w_src < 50):
            z_w_src = 50
        sourceA_prime = np.array([[1,0,0,x_w_src],[0,1,0,y_w_src],[0,0,1,z_w_src+80],[0,0,0,1]])
        sourceA = np.array([[1,0,0,x_w_src],[0,1,0,y_w_src],[0,0,1,z_w_src],[0,0,0,1]])
        angleA = np.arctan2(y_w_src, x_w_src)*(180/np.pi)
        Y_rot_component = 180
        while(kp.IK(kp.Gripperpose(sourceA_prime,angleA,Y_rot_component,0)) is None and Y_rot_component >= -180):
            Y_rot_component -= 1
        sourceA_prime = kp.Gripperpose(sourceA_prime,angleA,Y_rot_component,0)
        Y_rot_component = 180
        while(kp.IK(kp.Gripperpose(sourceA,angleA,Y_rot_component,0)) is None and Y_rot_component >= -180):
            Y_rot_component -= 1
        sourceA = kp.Gripperpose(sourceA,angleA,Y_rot_component,0)
        IK_A_prime = kp.IK(sourceA_prime)
        IK_A = kp.IK(sourceA)

        sourceB_prime = np.array([[1,0,0,x_w_dest],[0,1,0,y_w_dest],[0,0,1,z_w_dest+80],[0,0,0,1]])
        sourceB = np.array([[1,0,0,x_w_dest],[0,1,0,y_w_dest],[0,0,1,z_w_dest],[0,0,0,1]])
        angleB = np.arctan2(y_w_dest, x_w_dest)*(180/np.pi)
        Y_rot_component = 180
        while(kp.IK(kp.Gripperpose(sourceB_prime,angleB,Y_rot_component,0)) is None and Y_rot_component >= -180):
            Y_rot_component -= 1
        sourceB_prime = kp.Gripperpose(sourceB_prime,angleB,Y_rot_component,0)
        Y_rot_component = 180
        while(kp.IK(kp.Gripperpose(sourceB,angleB,Y_rot_component,0)) is None and Y_rot_component >= -180):
            Y_rot_component -= 1
        sourceB = kp.Gripperpose(sourceB,angleB,Y_rot_component,0)
        IK_B_prime = kp.IK(sourceB_prime)
        IK_B = kp.IK(sourceB)

        self.rexarm.open_gripper()
        time.sleep(3.0)
        spd = 1
        if(IK_A is not None and IK_A_prime is not None):
            self.tp.execute_plan([[0.0,0.0,0.0,0.0,0.0,0.0],IK_A_prime, IK_A], max_speed = spd, look_ahead = 8)
        self.rexarm.close_gripper()
        time.sleep(0.75)
        if(IK_B is not None):
            print ("=================")
            print ("IK_prime",IK_B_prime)
            self.tp.execute_plan([[0.0,0.0,0.0,0.0,0.0,0.0],IK_B_prime, IK_B], max_speed = spd, look_ahead = 8)
        self.rexarm.open_gripper()
        time.sleep(1.5)
        if(IK_B is not None):
            print ("=================")
            print ("IK",IK_B)
            self.tp.execute_plan([IK_B_prime, [0.0,0.0,0.0,0.0,0.0,0.0]], max_speed = spd, look_ahead = 8)

        self.next_state = "idle"

    def addPoints(self):
        if self.current_state == "teaching":
            self.waypoints.append(list(self.rexarm.joint_angles_fb))

    def teaching(self):
        self.status_message = "State: Teaching - Add waypoints using button 2"
        self.current_state = "teaching"
        self.rexarm.disable_torque()
        self.rexarm.get_feedback()
        self.next_state = "teaching"

    def task4(self):
        self.current_state = "task4"
        self.next_state = "idle"
        initialPose = [[1,0,0,200],[0,1,0,200],[0,0,1,50],[0,0,0,1]]
        Y_rot_component = 180
        while(kp.IK(kp.Gripperpose(initialPose,45,Y_rot_component,0)) is None and Y_rot_component >= -180):
            Y_rot_component -= 1
        curPoseOffset = np.eye(4)
        plan1=kp.Gripperpose(initialPose,45,Y_rot_component,0)
        path = np.array([kp.IK(plan1)])
        # step_size_mm = 5
        for i in range(40):
            curPoseOffset = [[1,0,0,0],[0,1,0,-5*(i+1)],[0,0,1,0],[0,0,0,1]]
            curPose = np.matmul(curPoseOffset,initialPose)
            newIK = kp.IK(kp.Gripperpose(curPose,45,Y_rot_component,0))
            toAppend = np.array([newIK])
            print(path)
            print(toAppend)
            if(newIK is not None):
                path = np.concatenate((path,toAppend), axis=0)
            else:
                Y_rot_component = 180
                while(kp.IK(kp.Gripperpose(curPose,-45,Y_rot_component,0)) is None and Y_rot_component >= -180):
                    Y_rot_component -= 1
                newIK = kp.IK(kp.Gripperpose(curPose,-45,Y_rot_component,0))
                print(newIK)
                toAppend = np.array([newIK])
                path = np.concatenate((path,toAppend), axis=0)
        print(path)
        self.tp.execute_plan(path,max_speed=1.0)

    def calibrate(self):
        self.current_state = "calibrate"
        self.next_state = "idle"
        self.kinect.kinectCalibrated = False
        self.kinect.new_click = False
        self.kinect.depth2rgb_affine = np.float32([[1,0,0],[0,1,0]])
        location_strings = ["lower left corner of board",
                            "upper left corner of board",
                            "upper right corner of board",
                            "lower right corner of board",
                            "center of shoulder motor"]
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in RGB image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.rgb_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False
        
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in depth image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.depth_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False
   
        

        """TODO Perform camera calibration here"""
        self.kinect.depth2rgb_affine = \
            self.kinect.getAffineTransform(self.kinect.depth_click_points, \
                self.kinect.rgb_click_points)

        model_points = np.array([\
            [-305.0,-305.0,0.0],\
                [-305.0,305.0,0.0],\
                    [305.0,305.0,0.0],\
                        [305.0,-305.0,0.0],\
                            [0.0,0.0,128.75]])

        image_points = np.float32(self.kinect.rgb_click_points[0:4])

        (success, rot_vec, trans_vec) = cv2.solvePnP(model_points[0:4], \
                        image_points,\
                    self.kinect.loadCameraCalibration(),\
                    None)
        if(success):
            self.kinect.registerExtrinsicMatrix(rot_vec, trans_vec)
            self.kinect.kinectCalibrated = True
            self.status_message = "Calibration - Completed Calibration"
            time.sleep(1)
        else:
            self.status_message = "Calibration - failed! See terminal for error msg."        