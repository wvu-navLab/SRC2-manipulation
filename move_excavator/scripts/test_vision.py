#!/usr/bin/env python3
import roslib
import rospy
import math

from move_excavator.srv import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import tf


global curr_odom_hauler
global curr_odom_excavator
global curr_target
curr_odom_hauler = Odometry()
curr_odom_excavator = Odometry()
curr_target = PointStamped()

def odometryCb_hauler(msg):
    global curr_odom_hauler 
    curr_odom_hauler = msg    
    #print("Hauler", msg.pose.pose)

def odometryCb_excavator(msg):
    global curr_odom_excavator
    curr_odom_excavator = msg 
    #print("Excavator",  msg.pose.pose)  
    
def targetCb(msg):
    global curr_target
    curr_target = msg 
    #print("Target",  msg)  

if __name__ == "__main__":
    rospy.init_node('test_vis', anonymous=True) #make node 
    rospy.Subscriber('small_hauler_1/localization/odometry/truth',Odometry,odometryCb_hauler)
    rospy.Subscriber('small_excavator_1/localization/odometry/truth',Odometry,odometryCb_excavator)
    rospy.Subscriber('small_excavator_1/manipulation/target_bin',PointStamped,targetCb)
    tl = tf.TransformListener()

    rate = rospy.Rate(1)
    
    print("Before")
    rospy.wait_for_service('small_excavator_1/manipulation/find_hauler')
    #FindHauler()
    print("After")
    
    while (not rospy.is_shutdown()):
    	rate.sleep()
    	#print("Hauler", curr_odom_hauler.pose.pose.position)
    	#print("Excavator", curr_odom_excavator.pose.pose.position)
    	#transf_target = tl.transformPoint("small_excavator_1_base_footprint", curr_target)
    	#print("Target", transf_target)
    	#hx = transf_target.point.x
    	#hy = transf_target.point.y
    	#d1=math.sqrt(hx**2+hy**2)
    	#d2=math.sqrt((curr_odom_hauler.pose.pose.position.x-curr_odom_excavator.pose.pose.position.x)**2+(curr_odom_hauler.pose.pose.position.y-curr_odom_excavator.pose.pose.position.y)**2)
    	#print("Estimated Hauler:", math.sqrt(hx**2+hy**2), " Actual Hauler", math.sqrt((curr_odom_hauler.pose.pose.position.x-curr_odom_excavator.pose.pose.position.x)**2+(curr_odom_hauler.pose.pose.position.y-curr_odom_excavator.pose.pose.position.y)**2), "Error", d1-d2)
    	
    
    #rospy.spin()
