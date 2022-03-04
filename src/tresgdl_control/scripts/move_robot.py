#!/usr/bin/env python

import sys
import rospy
import numpy as np
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3

class RobotController:

    goal_states = [[0, 0, 0],
                   [np.pi/2, 0, 0],
                   [0, np.pi/4, np.pi/4]]

    jointnames = ['shoulder_pan', 'shoulder_tilt', 'elbow']

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
        jointstate_sub = rospy.Subscriber('/tresgdl/jointstate', JointState,  self._js_callback,
                                          queue_size=1)
        self.goal_point = np.zeros(3)
        self.initial_state = np.zeros(3)
        self.current_state = np.zeros(3)
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

    def run_controller(self):
        while not rospy.is_shutdown():
            if self.control_state == 0:
                self.move_to_goal()
            elif self.control_state == 1:
                self.move_tcp_to_point()

            self.rate.sleep()

    def move_to_goal(self):
        try:
            jointangles = RobotController.goal_states[self.goal]
        except IndexError:
            pass

        n = 5 # Number of intermediate points
        delta_theta = (jointangles - self.initial_state)/n

        
        wait = 0.2
        for i in range(n):
            self.sh_pan.publish(jointangles[0] + (i+1)*delta_theta[0])
            self.sh_tilt.publish(jointangles[1] + (i+1)*delta_theta[1])
            self.elbow.publish(jointangles[2] + (i+1)*delta_theta[2])
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

        # Your code here

        
if __name__ == '__main__':
    try:
        fb = RobotController()
        fb.run_controller()
        
    except rospy.ROSInterruptException:
        pass
