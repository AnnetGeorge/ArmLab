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
            if(self.next_state == "task1"):
                self.task1()
            if(self.next_state == "task2"):
                self.task2()
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
            if(self.next_state == "task1"):
                self.task1()
            if(self.next_state == "task2"):
                self.task2()
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
            if(self.next_state == "task1"):
                self.task1()
            if(self.next_state == "task2"):
                self.task2()
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

        if(self.current_state == "task4"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "task1"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "task2"):
            if(self.next_state == "idle"):
                self.idle()

               

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
        sourceB = np.array([[1,0,0,x_w_dest],[0,1,0,y_w_dest],[0,0,1,z_w_dest+20],[0,0,0,1]])
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

    def getFinalNaivePoseForXYZ(self, x, y, z):
        pose = np.array([[1,0,0,x],[0,1,0,y],[0,0,1,z],[0,0,0,1]])
        angle = np.arctan2(y, x)*(180/np.pi)
        Y_rot_component = 180
        while(kp.IK(kp.Gripperpose(pose, angle, Y_rot_component,0)) is None and Y_rot_component >= -180):
            Y_rot_component -= 1
        return kp.Gripperpose(pose, angle, Y_rot_component,0)

    def task1Done(self, blocks):
        maxz = 0
        for block in blocks:
            if block[0][0] > 0:
                return False
            if block[0][2] > maxz:
                maxz = block[0][2]
        return maxz > 90
    
    def moveBlockFromSrcToDst(self, src, dest):
        x_w_src,y_w_src,z_w_src = src
        if(z_w_src  < 50):
            z_w_src = 50
        sourcePosePrimed = self.getFinalNaivePoseForXYZ(x_w_src,y_w_src,z_w_src+90)
        sourcePose = self.getFinalNaivePoseForXYZ(x_w_src,y_w_src,z_w_src)
        destPosePrimed = self.getFinalNaivePoseForXYZ(dest[0],dest[1],dest[2]+90)
        destPose = self.getFinalNaivePoseForXYZ(dest[0],dest[1],dest[2])
        sourcePrimedIK = kp.IK(sourcePosePrimed)
        sourceIK = kp.IK(sourcePose)
        destPrimedIK = kp.IK(destPosePrimed)
        destIK = kp.IK(destPose)
        if(sourcePrimedIK is None):
            print("source prime None")
        if(sourceIK is None):
            print("source IK None")
        if(destPrimedIK is None):
            print("dest prime None")
            print destPosePrimed
        if(destIK is None):
            print("dest IK None")
            print destPose
        if(not(sourcePrimedIK is None or sourceIK is None or destPrimedIK is None or destIK is None)):
            self.tp.execute_plan([sourcePrimedIK,sourceIK],max_speed=1.0)
            time.sleep(.35)
            self.rexarm.close_gripper()
            time.sleep(.35)
            self.tp.execute_plan([[self.rexarm.get_positions()[0],0.0,0.0,0.0,0.0,0.0],destPrimedIK,destIK],max_speed=1.0)
            time.sleep(.35)
            self.rexarm.open_gripper()
            time.sleep(.35)
            self.tp.execute_plan([destPrimedIK,[self.rexarm.get_positions()[0],0.0,0.0,0.0,0.0,0.0]],max_speed=1.0)

    def task1(self):
        self.current_state = "task1"
        self.next_state = "idle"

        self.status_message = "Performing task 1"
        self.rexarm.enable_torque()
        self.rexarm.get_feedback()

        #send the rexarm straight up so it doens't block cubes
        self.tp.execute_plan([[0.0,-0.2,0.0,0.0,0.0,0.0]])
        self.rexarm.open_gripper()
        dest = np.array([-100.0, 100, 10.0])
        #get the list of cubes
        cubes = self.kinect.blockDetector()
        while(not self.task1Done(cubes)):
            leftBlocks = []
            rightBlocks = []
            for cube in cubes:
                if(cube[0][0] < 0):
                    leftBlocks.append(cube)
                else:
                    rightBlocks.append(cube)
            # find dest block if it exists
            destBlock = None
            for leftBlock in leftBlocks:
                if(destBlock is None or destBlock[0][2] < leftBlock[0][2]):
                    destBlock = leftBlock
            # get destination from dest block
            if(destBlock is None):
                dest = [-160, 160, 40]
            else:
                dest = [destBlock[0][0]*1.00, destBlock[0][1]*1.00, destBlock[0][2]]

            for leftBlock in leftBlocks:
                if destBlock is None or not (leftBlock[0][0] == destBlock[0][0] and leftBlock[0][1] == destBlock[0][1] and leftBlock[0][2] == destBlock[0][2]):
                    src = (leftBlock[0][0]*1.04, leftBlock[0][1]*1.04, leftBlock[0][2])
                    dest[2] += 43
                    dest[0] *= 0.98
                    dest[1] *= 0.98
                    self.moveBlockFromSrcToDst(src,dest)
            
            for rightBlock in rightBlocks:
                src = (rightBlock[0][0]*1.04, rightBlock[0][1]*1.04, rightBlock[0][2])
                dest[2] += 43
                dest[0] *= 0.98
                dest[1] *= 0.98
                self.moveBlockFromSrcToDst(src,dest)
            
            self.tp.execute_plan([[self.rexarm.get_positions()[0],0.0,0.0,0.0,0.0,0.0]],max_speed=1.0)
            cubes = self.kinect.blockDetector()

    def task2bDone(self, blocks):
        for block in blocks:
            if block[0][1] > 0:
                return False
        return True

    def task2aDone(self, blocks):
        maxz = 55
        for block in blocks:
            if block[0][2] > maxz:
                return False
        return True

    def task2(self):
        self.current_state = "task2"
        self.next_state = "idle"

        self.status_message = "Performing task 2"
        self.rexarm.enable_torque()
        self.rexarm.get_feedback()

        self.tp.execute_plan([[0.0,0.0,0.0,0.0,0.0,0.0]])
        self.rexarm.open_gripper()

        cubes = self.kinect.blockDetector()
        while(not (self.task2aDone(cubes) and self.task2bDone(cubes))):
            while(not self.task2aDone(cubes)):
                stackedBlocks = []
                for cube in cubes:
                    if(cube[0][2] > 55.0):
                        stackedBlocks.append(cube)
                for stackedBlock in stackedBlocks:
                    src = (stackedBlock[0][0]*1.03, stackedBlock[0][1]*1.03, stackedBlock[0][2]+5)
                    dest = [src[0] + 80, src[1], src[2]]
                    if dest[1] > 0:
                        dest[1] = -dest[1]
                    if(dest[0] > 200):
                        dest[0] = 200
                    self.moveBlockFromSrcToDst(src,dest)
                self.tp.execute_plan([[self.rexarm.get_positions()[0],0.0,0.0,0.0,0.0,0.0]],max_speed=1.0)
                self.tp.execute_plan([[0.0,0.0,0.0,0.0,0.0,0.0]],max_speed=1.0)
                cubes = self.kinect.blockDetector()
            
            while(not self.task2bDone(cubes)):
                upperBlocks = []
                for cube in cubes:
                    if(cube[0][1] > 0.0):
                        upperBlocks.append(cube)
                for upperBlock in upperBlocks:
                    src = (upperBlock[0][0]*1.03, upperBlock[0][1]*1.03, upperBlock[0][2]+5)
                    dest = (src[0], -src[1], src[2]+40)
                    self.moveBlockFromSrcToDst(src,dest)
                self.tp.execute_plan([[self.rexarm.get_positions()[0],0.0,0.0,0.0,0.0,0.0]],max_speed=1.0)
                self.tp.execute_plan([[0.0,0.0,0.0,0.0,0.0,0.0]],max_speed=1.0)
                cubes = self.kinect.blockDetector()


    def task4(self):
        self.current_state = "task4"
        self.next_state = "idle"
        self.rexarm.enable_torque()
        sourcePoint = None
        destPoint = None
        self.rexarm.get_feedback()

        # self.kinect.new_click = False
        # while(not (self.kinect.new_click == True)):
        #     self.rexarm.get_feedback()  
        # self.kinect.new_click = False
        # sourcePoint = self.kinect.last_click.copy()
        # x_w_src,y_w_src,z_w_src,_ = self.kinect.ijToXyz(sourcePoint[0],sourcePoint[1])
        x_w_src,y_w_src,z_w_src = (200, 200, 80)

        # while(not (self.kinect.new_click == True)):
        #     self.rexarm.get_feedback()
        # self.kinect.new_click = False
        # destPoint = self.kinect.last_click.copy()
        # x_w_dest,y_w_dest,z_w_dest,_ = self.kinect.ijToXyz(destPoint[0],destPoint[1]
        # if(z_w_dest < 50):
        #     z_w_dest = 50
        if(z_w_src < 50):
            z_w_src = 50
        sourceA_prime = np.array([[1,0,0,x_w_src],[0,1,0,y_w_src],[0,0,1,z_w_src+80],[0,0,0,1]])
        sourceA = np.array([[1,0,0,x_w_src],[0,1,0,y_w_src],[0,0,1,z_w_src],[0,0,0,1]])
        angleA = np.arctan2(y_w_src, x_w_src)*(180/np.pi)
        angleB = -45
        angleC = np.arctan2(y_w_src, -x_w_src)*(180/np.pi)

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


        self.rexarm.open_gripper()
        time.sleep(3.0)
        spd = 1
        if(IK_A is not None and IK_A_prime is not None):
            self.tp.execute_plan([[0.0,0.0,0.0,0.0,0.0,0.0],IK_A_prime, IK_A], max_speed = spd, look_ahead = 8)
        self.rexarm.close_gripper()
        time.sleep(0.75)
        plan1 = []
        for move in range(21):
            sourceA = np.array([[1,0,0,x_w_src],[0,1,0,y_w_src-10*(move+1)],[0,0,1,z_w_src],[0,0,0,1]])
            while(kp.IK(kp.Gripperpose(sourceA,angleA,Y_rot_component,0)) is None and Y_rot_component >= -180):
                Y_rot_component -= 1
            sourceA_next = kp.Gripperpose(sourceA,angleA,Y_rot_component,0)
            if(kp.IK(sourceA_next) is None):
                while(kp.IK(kp.Gripperpose(sourceA,angleA-90,Y_rot_component,0)) is None and Y_rot_component >= -180):
                    Y_rot_component -= 1
                sourceA_next = kp.Gripperpose(sourceA,angleA-90,Y_rot_component,0)
            IK_A = kp.IK(sourceA_next)
            plan1.append(IK_A)
            # if(IK_A is not None):
                # self.tp.execute_plan([IK_A], max_speed = spd, look_ahead = 8)
            # time.sleep(0.2)
        plan1 = np.array(plan1)
        self.tp.execute_plan(plan1, max_speed = spd, look_ahead = 8)
        # print (plan1)
        self.rexarm.open_gripper()
        time.sleep(3.0)
        sourceA_prime = kp.FK_dh(plan1[-1],6)[0]
        P=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-70],[0,0,0,1]])
        sourceA_prime = np.matmul(sourceA_prime,P)
        sourceA_prime = kp.IK(sourceA_prime)
        self.tp.execute_plan([sourceA_prime,[0.0,0.0,0.0,0.0,0.0,0.0]], max_speed = spd, look_ahead = 8)

        plan = []
        for move in range(24):
            sourceB = np.array([[1,0,0,x_w_src-10],[0,1,0,y_w_src-200-10*(move+1)],[0,0,1,z_w_src],[0,0,0,1]])
            Y_rot_component=180
            while(kp.IK(kp.Gripperpose(sourceB,angleB,Y_rot_component,0)) is None and Y_rot_component >= -180):
                Y_rot_component -= 1
            sourceB_next = kp.Gripperpose(sourceB,angleB,Y_rot_component,0)
            IK_B = kp.IK(sourceB_next)
            plan.append(IK_B)
        plan = np.array(plan)
        print(plan)
        self.tp.execute_plan([[0.0,0.0,0.0,0.0,0.0,0.0],plan[1]], max_speed = spd, look_ahead = 8)
        self.rexarm.close_gripper()
        # time.sleep(2.0)
        self.tp.execute_plan(plan, max_speed = spd, look_ahead = 8)
        # self.rexarm.open_gripper()
        # sourceB_prime = kp.FK_dh(plan[-1],6)[0]
        # P=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-70],[0,0,0,1]])
        # sourceB_prime = np.matmul(sourceB_prime,P)
        # sourceB_prime = kp.IK(sourceB_prime)
        # self.tp.execute_plan([sourceB_prime,[0.0,0.0,0.0,0.0,0.0,0.0]], max_speed = spd, look_ahead = 8)
        # time.sleep(3.0)


        plan = []
        for move in range(19):
            sourceB = np.array([[1,0,0,x_w_src+10-10*(move+1)],[0,1,0,-y_w_src],[0,0,1,z_w_src],[0,0,0,1]])
            Y_rot_component=180
            while(kp.IK(kp.Gripperpose(sourceB,angleB,Y_rot_component,0)) is None and Y_rot_component >= -180):
                Y_rot_component -= 1
            sourceB_next = kp.Gripperpose(sourceB,angleB,Y_rot_component,0)
            IK_B = kp.IK(sourceB_next)
            if (IK_B is not None):
                plan.append(IK_B)
        print(len(plan))
        self.tp.execute_plan([[0.0,0.0,0.0,0.0,0.0,0.0],plan[1]], max_speed = spd, look_ahead = 8)
        plan = np.array(plan)
        print plan
        self.rexarm.close_gripper()
        self.tp.execute_plan(plan, max_speed = spd, look_ahead = 8)
        # self.rexarm.open_gripper()
        # time.sleep(3.0)
        sourceB_prime = kp.FK_dh(plan[-1],6)[0]
        P=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,50],[0,0,0,1]])
        sourceB_prime = np.matmul(sourceB_prime,P)
        sourceB_prime = kp.IK(sourceB_prime)
        self.tp.execute_plan([sourceB_prime,[0.0,0.0,0.0,0.0,0.0,0.0]], max_speed = spd, look_ahead = 8)
        time.sleep(3.0)


        plan = []
        for move in range(10):
            sourceB = np.array([[1,0,0,x_w_src-190-10*(move+1)],[0,1,0,-y_w_src],[0,0,1,z_w_src],[0,0,0,1]])
            Y_rot_component=180
            while(kp.IK(kp.Gripperpose(sourceB,angleB,Y_rot_component,0)) is None and Y_rot_component >= -180):
                Y_rot_component -= 1
            sourceB_next = kp.Gripperpose(sourceB,angleB,Y_rot_component,0)
            IK_B = kp.IK(sourceB_next)
            if (IK_B is not None):
                plan.append(IK_B)
        print(len(plan))
        self.tp.execute_plan([[0.0,0.0,0.0,0.0,0.0,0.0],plan[1]], max_speed = spd, look_ahead = 8)
        plan = np.array(plan)
        print plan
        self.rexarm.close_gripper()
        self.tp.execute_plan(plan, max_speed = spd, look_ahead = 8)

        plan = []
        Y_rot_component=180
        for move in range(11):
            sourceB = np.array([[1,0,0,-90-10*(move+1)],[0,1,0,-y_w_src],[0,0,1,z_w_src],[0,0,0,1]])     
            while(kp.IK(kp.Gripperpose(sourceB,angleA,Y_rot_component,0)) is None and Y_rot_component >= -180):
                Y_rot_component -= 1
            sourceB_next = kp.Gripperpose(sourceB,angleA,Y_rot_component,0)
            IK_B = kp.IK(sourceB_next)
            if (IK_B is not None):
                plan.append(IK_B)
        print(plan)
        print(len(plan))
        plan = np.array(plan) 
        self.tp.execute_plan(plan, max_speed = spd, look_ahead = 8)
        # self.tp.execute_plan([[0.0,0.0,0.0,0.0,0.0,0.0]], max_speed = spd, look_ahead = 8)


        # "Lower Left to Upper Left"
        plan = []
        Y_rot_component=180
        for move in range(14):
            sourceB = np.array([[1,0,0,-x_w_src],[0,1,0,-y_w_src+10*(move+1)],[0,0,1,z_w_src],[0,0,0,1]])     
            while(kp.IK2(kp.Gripperpose(sourceB,angleA,Y_rot_component,0)) is None and Y_rot_component >= -180):
                Y_rot_component -= 1
            sourceB_next = kp.Gripperpose(sourceB,angleA,Y_rot_component,0)
            IK_B = kp.IK2(sourceB_next)
            if (IK_B is not None):
                plan.append(IK_B)
        
        print(len(plan))
        plan = np.array(plan)
        print(plan)
        self.tp.execute_plan(plan, max_speed = spd, look_ahead = 8)


        plan = []
        Y_rot_component=180
        for move in range(2):
            Y_rot_component=180
            sourceB = np.array([[1,0,0,-x_w_src],[0,1,0,-y_w_src+150+10*(move+1)],[0,0,1,z_w_src],[0,0,0,1]])     
            while(kp.IK(kp.Gripperpose(sourceB,angleC,Y_rot_component,0)) is None and Y_rot_component >= -180):
                Y_rot_component -= 1
            sourceB_next = kp.Gripperpose(sourceB,angleC,Y_rot_component,0)
            IK_B = kp.IK(sourceB_next)
            if (IK_B is not None):
                plan.append(IK_B)
        
        print(len(plan))
        plan = np.array(plan)
        print(plan)
        self.tp.execute_plan(plan, max_speed = spd, look_ahead = 8)  

        
        plan = []
        Y_rot_component=180
        for move in range(24):
            Y_rot_component=180
            sourceB = np.array([[1,0,0,-x_w_src],[0,1,0,-y_w_src+170+10*(move+1)],[0,0,1,z_w_src],[0,0,0,1]])     
            while(kp.IK2(kp.Gripperpose(sourceB,angleC,Y_rot_component,0)) is None and Y_rot_component >= -180):
                Y_rot_component -= 1
            sourceB_next = kp.Gripperpose(sourceB,angleC,Y_rot_component,0)
            IK_B = kp.IK2(sourceB_next)
            if (IK_B is not None):
                plan.append(IK_B)
        
        print(len(plan))
        plan = np.array(plan)
        print(plan)
        self.tp.execute_plan(plan, max_speed = spd, look_ahead = 8)       

        plan = []
        Y_rot_component=180
        for move in range(20):
            Y_rot_component=180
            sourceB = np.array([[1,0,0,-x_w_src+10*(move+1)],[0,1,0,y_w_src],[0,0,1,z_w_src],[0,0,0,1]])     
            while(kp.IK2(kp.Gripperpose(sourceB,angleC,Y_rot_component,0)) is None and Y_rot_component >= -180):
                Y_rot_component -= 1
            sourceB_next = kp.Gripperpose(sourceB,angleC,Y_rot_component,0)
            IK_B = kp.IK2(sourceB_next)
            if (IK_B is not None):
                plan.append(IK_B)
        
        print(len(plan))
        plan = np.array(plan)
        print(plan)
        self.tp.execute_plan(plan, max_speed = spd, look_ahead = 8)

        plan = []
        Y_rot_component=180
        for move in range(20):
            Y_rot_component=180
            sourceB = np.array([[1,0,0,-x_w_src+200+10*(move+1)],[0,1,0,y_w_src],[0,0,1,z_w_src],[0,0,0,1]])     
            while(kp.IK(kp.Gripperpose(sourceB,angleA,Y_rot_component,0)) is None and Y_rot_component >= -180):
                Y_rot_component -= 1
            sourceB_next = kp.Gripperpose(sourceB,angleA,Y_rot_component,0)
            IK_B = kp.IK(sourceB_next)
            if (IK_B is not None):
                plan.append(IK_B)
        
        print(len(plan))
        plan = np.array(plan)
        print(plan)
        self.tp.execute_plan(plan, max_speed = spd, look_ahead = 8)     


        self.next_state = "idle"


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
            self.kinect.SaveMatrices()
            time.sleep(1)
        else:
            self.status_message = "Calibration - failed! See terminal for error msg."        