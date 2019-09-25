import time
import numpy as np
import kinematics as kp
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
        self.status_message = "State: Executing waypoint path"
        self.current_state = "execute"
        self.rexarm.enable_torque()
        cos = np.cos
        sin = np.sin
        self.A = np.array([[cos(np.pi),0,-sin(np.pi),100],[0,1,0,100],[sin(np.pi),0,cos(np.pi),120],[0,0,0,1]])
        self.B = np.array([[1,0,0,150],[0,1,0,150],[0,0,1,115],[0,0,0,1]])
        self.IKtest = kp.IK(self.A)
        print self.IKtest
        self.tp.execute_plan(self.IKtest)
        # self.tp.execute_plan(self.waypoints)
        #for waypoint in self.waypoints:
        #    self.rexarm.set_positions(waypoint)
        #    self.rexarm.pause(2.0)
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


    def calibrate(self):
        self.current_state = "calibrate"
        self.next_state = "idle"
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
            np.linalg.inv(self.kinect.getAffineTransform(self.kinect.rgb_click_points, \
                self.kinect.depth_click_points))
        self.kinect.kinectCalibrated = True
        print(self.kinect.depth2rgb_affine)
        self.status_message = "Calibration - Completed Calibration"
        time.sleep(1)