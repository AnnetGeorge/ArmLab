import numpy as np 
import time

"""
TODO: build a trajectory generator and waypoint planner 
        so it allows your state machine to iterate through
        the plan at the desired command update rate
"""

class TrajectoryPlanner():
    def __init__(self, rexarm):
        self.idle = True
        self.rexarm = rexarm
        self.num_joints = rexarm.num_joints
        self.initial_wp = [0.0]*self.num_joints
        self.final_wp = [0.0]*self.num_joints 
        self.dt = 0.05 # command rate
    
    def set_initial_wp(self):
        self.initial_wp = self.rexarm.get_positions()

    def set_final_wp(self, waypoint):
        self.final_wp = waypoint

    def go(self, max_speed = 2.5, look_ahead = 8.0):
        T_exp = self.calc_time_from_waypoints(self.initial_wp, self.final_wp, max_speed)
        startT = time.time()
        curT = startT
        running = True
        spline = self.generate_cubic_spline(self.initial_wp, self.final_wp, T_exp)
        while(running):
            curT = time.time()
            deltaT = (curT - startT)
            look_ahead_deltaT = deltaT + look_ahead/1000
            if(look_ahead_deltaT > startT + T_exp):
                look_ahead_deltaT = T_exp - curT
            joint_positions = [0.0] * self.num_joints
            for i in range(self.num_joints):
                A = spline[i]
                joint_pos = A[0] + A[1]*look_ahead_deltaT + (A[2])*(look_ahead_deltaT**2) + (A[3])*(look_ahead_deltaT**3) 
                joint_positions[i] = joint_pos
            self.rexarm.set_positions(joint_positions)
            # self.rexarm.pause(self.dt)
            if(curT > (startT + T_exp)):
                running = False

    def stop(self):
        self.rexarm.set_positions(self.rexarm.get_positions())

    def calc_time_from_waypoints(self, initial_wp, final_wp, max_speed):
        delta = np.array(final_wp) - np.array(initial_wp)
        times = np.abs(delta / max_speed)
        print(np.amax(times))

        return np.amax(times)

    def generate_cubic_spline(self, initial_wp, final_wp, T):
        M = [[1, 0, 0, 0], [0, 1, 0, 0], [1, T, T**2, T**3], [0, 1, 2*T, 3*T**2]]
        M_inv = np.linalg.inv(M)
        A = np.zeros((self.num_joints, 4))
        for i in range(self.num_joints):
            q0 = initial_wp[i]
            q1 = final_wp[i]
            v0 = 0
            vf = 0
            b = np.transpose([q0, v0, q1, vf])
            A_i = np.matmul(M_inv,b)
            A[i] = A_i
        return A


    def execute_plan(self, plan, look_ahead=8, max_speed=2.5):
        for waypoint in plan:
            self.set_initial_wp()
            self.set_final_wp(waypoint)
            self.go(max_speed = max_speed, look_ahead = look_ahead)