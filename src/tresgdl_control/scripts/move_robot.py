#!/usr/bin/env python

import sys
import rospy
import numpy as np
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3

class RobotController:

    goal_states = [[0, 0, 0, 0, 0],
                   [np.pi/2, 0, 0, 0],
                   [0, np.pi/4, np.pi/4, 0.03, 0.03]]

    jointnames = ['shoulder_pan', 'shoulder_tilt', 'elbow',
                  'arm_to_gripper_one', 'arm_to_gripper_two']

    link0_length = 0.1 # From floor to top of base
    link1_length = 0.15 # From center of joint shoulder_pan to joint shoulder_tilt
    link2_length = 0.8 # From joint shoulder_tilt to elbow
    link3_length = 0.9 # From elbow to tool center point
    
    def __init__(self):
        self.sh_pan = rospy.Publisher('/tresgdl/shoulder_pan_position_controller/command',
                                      Float64, queue_size=10)
        self.sh_tilt = rospy.Publisher('/tresgdl/shoulder_tilt_position_controller/command',
                                      Float64, queue_size=10)
        self.elbow = rospy.Publisher('/tresgdl/elbow_position_controller/command',
                                      Float64, queue_size=10)
        movecommand_sub = rospy.Subscriber('/movecommand', Int32,  self._move_callback)
        move2point_sub = rospy.Subscriber('/move_to_point', Vector3,  self._move_to_point_callback)
        jointstate_sub = rospy.Subscriber('/tresgdl/joint_states', JointState,  self._js_callback,
                                          queue_size=1)
        self.goal_point = np.zeros(len(RobotController.jointnames))
        self.initial_state = np.zeros(len(RobotController.jointnames))
        self.current_state = np.zeros(len(RobotController.jointnames))
        self.goal = 0 # Variable set by movecommand. Determines where the robot moves
        self.at_goal = False

        self.control_state = 0 # Used for implementing a simple finite state machine
        
        rospy.init_node("Controller", anonymous=True)
        self.rate = rospy.Rate(20)

    def _move_callback(self, data):
        self.goal = data.data # Index into the list of goal states
        self.initial_state[:] = self.current_state # Copy current state to initial state
        self.control_state = 0
        self.at_goal = False
                  
    def _move_to_point_callback(self, data):
        self.goal_point[:] = [data.x, data.y, data.z]
        self.control_state = 1
                  
    def _js_callback(self, data):
        for i,name in enumerate(data.name):
            self.current_state[RobotController.jointnames.index(name)] = data.position[i]
        #print(self.current_state)
    def run_controller(self):
        while not rospy.is_shutdown():
            if self.control_state == 0:
                self.move_smoothly()
            elif self.control_state == 1:
                self.move_tcp_to_point()

            self.rate.sleep()

    def move_to_goal(self):
        try:
            jointangles = RobotController.goal_states[self.goal]
        except IndexError:
            pass

        if self.at_goal:
            return

        
        n = 5 # Number of intermediate points
        delta_theta = (jointangles - self.initial_state)/n

        
        wait = 0.2
        for i in range(n):
            self.sh_pan.publish(self.initial_state[0] + (i+1)*delta_theta[0])
            self.sh_tilt.publish(self.initial_state[1] + (i+1)*delta_theta[1])
            self.elbow.publish(self.initial_state[2] + (i+1)*delta_theta[2])
            # Wait for wait seconds
            now = rospy.get_time()
            while rospy.get_time() < (now + wait):
                self.rate.sleep()
        self.at_goal = True
        
    def move_tcp_to_point(self):
        '''
        Solves inverse kinematics problem, then moves the robot arm so the tool center point
        is at the desired position given by self.goal_point
        '''

        # Your code here

    def move_smoothly(self):
        '''
        Uses interpolation to move the arm smoothly from the initial state to the goal state.
        '''
        try:
            jointangles = RobotController.goal_states[self.goal]
        except IndexError:
            pass

        if self.at_goal:
            return

        T = 1
        # Find cubic interpolation parameters by solving
        # A * a = b
        
        A = np.array([[1, 0, 0, 0],
                      [1, T, T**2, T**3],
                      [0, 1, 0, 0],
                      [0, 1, 2*T, 3*T**2]])
        b = np.array([0, 1, 0, 0])
        a = np.linalg.solve(A, b)
        #print(a)
        
        delta_theta = (jointangles - self.initial_state)

        now = rospy.get_time()
        while rospy.get_time() < (now + T):
            # Find interpolated value, then publish it
            t = rospy.get_time() - now
            tvec = np.array([1, t, t**2, t**3])
            beta = float(np.dot(a, tvec))
            #print(beta)
            self.sh_pan.publish(self.initial_state[0] + beta*delta_theta[0])
            self.sh_tilt.publish(self.initial_state[1] + beta*delta_theta[1])
            self.elbow.publish(self.initial_state[2] + beta*delta_theta[2])

            self.rate.sleep()
        
        self.at_goal = True
        

        # Your code here

        
if __name__ == '__main__':
    try:
        fb = RobotController()
        fb.run_controller()
        
    except rospy.ROSInterruptException:
        pass
