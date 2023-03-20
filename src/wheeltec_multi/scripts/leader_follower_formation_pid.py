#!/usr/bin/env python3

import rospy
import tf
from tf import transformations
from tf import broadcaster
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import math
import numpy as np
# from simple_pid import PID

# from dynamic_reconfigure.server import Server # http://wiki.ros.org/dynamic_reconfigure/Tutorials/HowToWriteYourFirstCfgFile
# from turtlebot_sim.cfg import tf_pidConfig   
    
e_linear_x = 0
e_linear_y = 0
e_angular_z = 0
angular_turn = 0

slave_x = 0
slave_y = 0

max_vel_x=3.0
min_vel_x=0.05
max_vel_theta=3.0
min_vel_theta=0.05

k_v=1
k_l=1
k_a=1

# Master robot's velocity
odom_linear_x=0
odom_linear_y=0
odom_angular_z=0

def odom_cb(msg):
    global odom_linear_x, odom_linear_y, odom_angular_z
    odom_linear_x = msg.twist.twist.linear.x
    odom_linear_y = msg.twist.twist.linear.y
    odom_angular_z = msg.twist.twist.angular.z
    
    if(math.fabs(odom_linear_x) < min_vel_x):
        odom_linear_x = 0

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('turtlebot_tf_listener')

    # Get the parameters
    # target_frame = rospy.get_param('~target_frame',leader_robot_name+"/base_footprint")
    # theta0 = rospy.get_param('~expected_theta', np.pi)
    # L0 = rospy.get_param('~expected_distance', 1)
    # d = rospy.get_param('~front_distance', 0.5)


    leader_robot_name = rospy.get_param('~leader_robot_name', "")
    follower_robot_name = rospy.get_param('~follower_robot_name', "robot_1")

    base_frame = rospy.get_param("~base_frame", "base_footprint")
    base_to_slave = rospy.get_param("~base_to_slave", "slave1")
    slave_x = rospy.get_param("~slave_x", -1.5)
    slave_y = rospy.get_param("~slave_y", 1.5)
    max_vel_x = rospy.get_param("~max_vel_x", 1.0)
    min_vel_x = rospy.get_param("~min_vel_x", 0.05)
    max_vel_theta = rospy.get_param("~max_vel_theta", 1.0)
    min_vel_theta = rospy.get_param("~min_vel_theta", 0.05)
    k_v = rospy.get_param("~k_v", 1)
    k_l = rospy.get_param("~k_l", 1)
    k_a = rospy.get_param("~k_a", 1)
    # print all parameters
    print("leader_robot_name: ", leader_robot_name)
    print("follower_robot_name: ", follower_robot_name)
    print("base_frame: ", base_frame)
    print("base_to_slave: ", base_to_slave)
    print("slave_x: ", slave_x)
    print("slave_y: ", slave_y)
    print("max_vel_x: ", max_vel_x)
    print("min_vel_x: ", min_vel_x)
    print("max_vel_theta: ", max_vel_theta)
    print("min_vel_theta: ", min_vel_theta)
    print("k_v: ", k_v)
    print("k_l: ", k_l)
    print("k_a: ", k_a)
    rospy.sleep(3)

    # Subscribe to the leader robot's odometry
    leader_vel = rospy.Subscriber(leader_robot_name + "/odom", Odometry, odom_cb)

    # Publish to the follower robot's velocity
    print(follower_robot_name + '/cmd_vel')
    slave_vel = rospy.Publisher(follower_robot_name + '/cmd_vel', Twist, queue_size=1)

    # Create a transform listener
    listener = tf.TransformListener()

    # Create a transform broadcaster
    rate = rospy.Rate(10.0)

    # Get tf prefix

    base_frame = tf.TransformListener()
    base_to_slave = tf.TransformListener()

    listner = tf.TransformListener()
    vel_msg = Twist()

    e_angular_z = 0
    e_angular_y = 0

    # Swap coordinates, slave_x and slave_y use leader's coordinate system, 右邊為x軸，前面為y軸
    slave_x = slave_x + slave_y
    slave_y = slave_x - slave_y
    slave_x = -(slave_x - slave_y)
    

    rospy.on_shutdown(lambda: slave_vel.publish(Twist()))

    while not rospy.is_shutdown():
        # Get the transform from the base frame to the slave frame
        try:
            # 追隨者座標與期望座標的相對位置
            ## Ex: (追隨者座標) /follower/base_footprint -> (期望座標) /leader/follower
            frame_1 = f"{follower_robot_name}/base_footprint"
            frame_2 = f"{leader_robot_name}/{follower_robot_name}"
            # print(frame_1, frame_2)
            (trans, rot) = listener.lookupTransform(frame_1, frame_2, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Waiting for TF data or no TF data available")
            rospy.sleep(1)
            continue
            
        # Quaternion to Euler angles
        rpy = tf.transformations.euler_from_quaternion(rot)

        e_angular_x, e_angular_y, e_angular_z = rpy
        e_linear_x, e_linear_y, e_linear_z = trans

        # Assuming rpy, trans and e_angular_x, e_angular_y, e_angular_z, e_linear_x, e_linear_y, e_linear_z are defined
        # print("Euler Angles (RPY):")
        # print("Roll: ", e_angular_x)
        # print("Pitch: ", e_angular_y)
        # print("Yaw: ", e_angular_z)
        
        # print("Translation:")
        # print("X: ", e_linear_x)
        # print("Y: ", e_linear_y)
        # print("Z: ", e_linear_z)
        


        if math.fabs(e_linear_x) < 5:
            try:
                angular_turn = math.atan2(slave_y, odom_linear_x/odom_angular_z)
            except ZeroDivisionError:
                angular_turn = math.atan2(slave_y, slave_x)
                print("ZeroDivisionError: angular_turn = math.atan2(slave_y, odom_linear_x/odom_angular_z)")
        else:
            angular_turn = math.atan2(slave_y, slave_x)

        if (math.fabs(odom_angular_z) > min_vel_theta):
            if slave_x > 0 and slave_y > 0:
                if odom_angular_z > 0:
                    e_angular_z = e_angular_z+angular_turn
                else:
                    e_angular_z=e_angular_z- np.pi + angular_turn
            elif slave_x <= 0 and slave_y >= 0:
                if odom_angular_z > 0:
                    e_angular_z = e_angular_z - angular_turn
                else:
                    e_angular_z = e_angular_z+angular_turn- np.pi 
            elif slave_x < 0 and slave_y < 0:
                if odom_angular_z > 0:
                    e_angular_z = e_angular_z - angular_turn
                else:
                    e_angular_z = e_angular_z+ np.pi + angular_turn
            else:
                if odom_angular_z > 0:
                    e_angular_z = e_angular_z - angular_turn
                else:
                    e_angular_z = e_angular_z+ np.pi + angular_turn
        

            if(e_angular_z > 4.71 and e_angular_z < 7.85): e_angular_z = e_angular_z-6.28
            if(e_angular_z < -4.71 and e_angular_z > -7.85): e_angular_z = e_angular_z+6.28
        # print("Deviation: \n e_linear_x = {}\n e_linear_y = {}\n e_angular_z = {}\n".format(e_linear_x, e_linear_y, e_angular_z))

        if abs(odom_linear_x/odom_angular_z) < 5.0: # odom_linear_x/odom_angular_z < 5.0 means the car is not moving in a straight line
            d_r = 0.0
            if slave_x + odom_linear_x/odom_angular_z >= 0:
                d_r = math.sqrt(pow(slave_x + odom_linear_x/odom_angular_z, 2) + pow(slave_y, 2)) - odom_linear_x/odom_angular_z
            else:
                d_r = -math.sqrt(pow(slave_x + odom_linear_x/odom_angular_z, 2) + pow(slave_y, 2)) - odom_linear_x/odom_angular_z
            vel_msg.linear.x = (odom_linear_x + odom_angular_z * d_r) * abs(math.cos(e_angular_z)) + k_v * e_linear_x # According to v/w=r, where w_slave=w_master, so
                                                                                                                    # v_slave/w_slave=(odom_linear_x+odom_angular_z*d_r) /odom_angular_z=r_master+d_r=r_slave, which complies with kinematic rules
        else:
            vel_msg.linear.x = odom_linear_x + k_v * e_linear_x
        _k_a = k_a
        _k_l = k_l

        if vel_msg.linear.x < -min_vel_x: # When the slave car is reversing, correct the signs of the parameters to be opposite to those when it is moving forward
            if abs(odom_angular_z) > min_vel_theta:
                _k_a = -k_a # _k_a parameter is positive for forward motion
                _k_l = -k_l

        vel_msg.angular.z = odom_angular_z + _k_l * e_linear_y + _k_a * math.sin(e_angular_z)

        if vel_msg.linear.x > max_vel_x:
            vel_msg.linear.x = max_vel_x # Velocity limit
        elif vel_msg.linear.x < -max_vel_x:
            vel_msg.linear.x = -max_vel_x
        if vel_msg.angular.z > max_vel_theta:
            vel_msg.angular.z = max_vel_theta # Velocity limit
        elif vel_msg.angular.z < -max_vel_theta:
            vel_msg.angular.z = -max_vel_theta
        # vel_msg.linear.x = -vel_msg.linear.x 
        # vel_msg.angular.z = -vel_msg.angular.z 
        # print(vel_msg.linear.x, vel_msg.angular.z)
        slave_vel.publish(vel_msg)
        rate.sleep()
    