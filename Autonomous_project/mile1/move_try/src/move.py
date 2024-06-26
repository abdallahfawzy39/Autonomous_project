#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from move_try.srv import ang, angResponse

def set_angular_velocity(req):
    global angular_vel
    angular_vel = req.angular_velocity
    return angResponse("Angular Velocity Set: %s"%(req.angular_velocity))

def Move_Robot():
    global angular_vel

    rospy.init_node('Move_Robot', anonymous=True) 
    vel_msg = Twist()

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    # Define the robot parameters required for simulation:
    r = 0.1 #the wheel radius in meters
    l = 0.2 #the wheel base distance in meters (width of robot)

    # Define the initial wheel velocities:
    Wr = 2
    Wl = 2
    angular_vel = 0.0

    # Define the service to set the angular velocity:
    rospy.Service('/set_angular_velocity', ang, set_angular_velocity)

    while not rospy.is_shutdown():
        # Update the wheel velocities based on the current angular velocity:
        Wr = 2 + angular_vel
        Wl = 2 - angular_vel

        # Calculate the linear and angular velocities    
        v = (r/2) * (Wr + Wl) # Linear Velocity
        w = (r/l) * (Wr - Wl) # Angular Velocity

        vel_msg.linear.x = v  # Linear Velocity
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = w  # Angular Velocity

        pub.publish(vel_msg)
        rate.sleep()		

		
if __name__ == '__main__':
    try:
        Move_Robot()
    except rospy.ROSInterruptException:
        pass
