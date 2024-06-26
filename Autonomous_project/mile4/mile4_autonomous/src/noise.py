#!/usr/bin/env python3


# Imported Packages
import rospy 
import numpy as np 
import matplotlib.pyplot as plt
import random
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray




#Initialization
position = np.zeros((1,3))	#Identify array of six elements all initialized by zero
position_Noise_msg = Float32MultiArray()
# Standard Deviation Inputs
STD_DEV = [0.3,0.3,0.1]

XReal = [] #Array to hold value of x obtained by the gazebo
YReal = [] #Array to hold value of y obtained by the gazebo
ThetaReal = [] #Array to hold value of theta obtained by the gazebo

XNoisy = [] #Array to hold noisy value of x coordinates
YNoisy = [] #Array to hold noisy value of y coordinates
ThetaNoisy = [] #Array to hold noisy value of theta coordinates
	

def Callback(Pose):

    global X_pos
    global Y_pos
    global Theta
    global position
    
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
    
    position = [X_pos, Y_pos, Theta]
    Position_Noise = add_Noise(position)
    position_Noise_msg.data = Position_Noise

    XReal.append(position[0]) 	    #Array to hold value of x obtained by gazebo
    YReal.append(position[1])  	    #Array to hold value of y obtained by  gazebo
    ThetaReal.append(position[2]) 	#Array to hold value of theta obtained by  gazebo
    
    XNoisy.append(position_Noise[0]) 			#Array to hold noisy value of x coordinates
    YNoisy.append(position_Noise[1]) 			#Array to hold noisy value of y coordinates
    ThetaNoisy.append(position_Noise[2])		#Array to hold noisy value of theta coordinates

    pub.publish(position_Noise_msg)	#Publish msg



def add_Noise(position):
	global SD
	global position_Noise

	x_noise = position[0] + random.uniform(-STD_DEV[0],STD_DEV[0])		#Noisy data calculated at x
	y_noise = position[1] + random.uniform(-STD_DEV[1],STD_DEV[1])		#Noisy data calculated at y
	theta_noise = position[2] + random.uniform(-STD_DEV[2],STD_DEV[2])	#Noisy data calculated at theta

	position_Noise = [x_noise, y_noise, theta_noise]   #Store the noisy position in array
 
	return(position_Noise)



if __name__ == '__main__':     # Main function that is executed 

	# Initialize ROS Node 
	rospy.init_node('Turtle_Noise', anonymous = True) #Identify Ros Node
	# Initialize The Subscriber
	rospy.Subscriber('/gazebo/model_states', ModelStates, Callback) #Subscribe to gazebo
	# Initialize The Publisher
	pub = rospy.Publisher('/Noise_data', Float32MultiArray, queue_size=10) #Publish the New Noisy message
	rospy.spin()
    
    ##Plotting of signals from sensor and noisy signals
	plt.figure(1)
	line_1 = plt.plot(XReal, 'r-', label='X-Real')
	line_2 = plt.plot(XNoisy, 'b-', label='X-Noisy')
	plt.legend()


	plt.figure(2)
	line_1 = plt.plot(YReal, 'r-', label='Y-Real')
	line_2 = plt.plot(YNoisy, 'b-', label='Y-Noisy')
	plt.legend()

	plt.figure(3)
	line_1 = plt.plot(ThetaReal, 'r-', label='Theta-Real')
	line_2 = plt.plot(ThetaNoisy, 'b-', label='Theta-Noisy')
	plt.legend()

	
	plt.show(block=True)



