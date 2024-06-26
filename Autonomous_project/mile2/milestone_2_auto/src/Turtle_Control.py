#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
import numpy as np

## Identify Global Variables ##
global p
global alpha
global beta
global X_pos
global Y_pos
global Theta
global linear_v
global angular_v
global x_desired
global y_desired
global theta_desired
global Kp
global Kalpha
global Kbeta
# Polar coordinates
p = 0
alpha = 0
beta = 0
# Desired positions we require our robot to reach
x_desired = 0
y_desired = 0
theta_desired = 0
# Current positions our robot is at
X_pos = 0
Y_pos = 0
Theta = 0
# Control parameters
linear_v = 0
angular_v = 0
Kp = 0.3
Kalpha = 1
Kbeta = -0.5


def Callback(Pose):

    global X_pos
    global Y_pos
    global Theta

    # Find the turtle's index in the ModelStates message
    turtle_index = Pose.name.index('turtlebot3_waffle_pi')

    orientation = Pose.pose[turtle_index].orientation
    _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w]) #only yaw (third) is used. first & second are dummies
    
    yaw_degrees = yaw * 180.0 / np.pi  # convert radians to degrees
    if yaw_degrees < 0:
        yaw_degrees += 360

    X_pos = round(Pose.pose[turtle_index].position.x, 3)
    Y_pos = round(Pose.pose[turtle_index].position.y, 3)
    if yaw > 0 :
        Theta = round (yaw, 3)
    else:
        Theta = round ((2*(np.pi) + yaw), 3)
    
    print("x_curr: ",X_pos)
    print("y_curr: ",Y_pos)
    print("theta_curr: ",yaw_degrees)



def Polar_Coordinates():
    global p
    global beta
    global alpha
    global x_desired
    global y_desired
    global theta_desired
    global X_pos
    global Y_pos
    global Theta

    x_delta = x_desired - X_pos
    y_delta = y_desired - Y_pos

    p = np.sqrt((np.square(x_delta)) + (np.square(y_delta)))
   
    gamma = np.arctan2(y_delta, x_delta)

    if gamma < 0:
        gamma = gamma + (2 * np.pi)

    alpha = gamma - Theta
    
    beta = -alpha - Theta + theta_desired
    

def Control_Law():
    global p
    global beta
    global alpha
    global linear_v
    global angular_v
    global Kp
    global Kalpha
    global Kbeta

    if np.absolute(alpha) < np.pi / 2:
        linear_v = Kp * p
    else:
        linear_v = -Kp * p
   
    angular_v = Kalpha * alpha + Kbeta * beta


if __name__ == '__main__':
    # Input your desired coordinates
    x_desired = float(input("Desired X goal: "))
    y_desired = float(input("Desired Y goal: "))
    theta_desired_degree = float(input("Desired Theta goal: "))
    
    theta_desired = theta_desired_degree*np.pi/180
    # Node, Publisher and Subscriber Setup
    rospy.init_node('Turtle_Control', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/gazebo/model_states', ModelStates, Callback)
    rate = rospy.Rate(10)
    vel_msg = Twist()

    while not rospy.is_shutdown():
        Polar_Coordinates()
        Control_Law()

        v = round(linear_v, 2)
        w = round(angular_v, 2)

        
        if abs(X_pos - x_desired) < 0.01 and abs(Y_pos - y_desired) < 0.01 and abs(Theta - theta_desired) < 0.1: # If within threshold range, set linear and angular velocities to 0 to stop the robot
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
        else:
            vel_msg.linear.x = linear_v
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = angular_v

        pub.publish(vel_msg)
        rate.sleep()

