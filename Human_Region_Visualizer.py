#!/usr/bin/env python

# General imports
import rospy
import math
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import time
import array as arr
from datetime import datetime

# ROS messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from pedsim_msgs.msg import AgentState, AgentStates
from pedsim_msgs.msg import AgentGroup, AgentGroups

# custom functions
import libs.maths.quaternions as custom_quat
import libs.models.farida_model as farida

# Casadi 
import casadi as ca
from casadi import sin, cos, pi


#Human Variables
human_detected = False
human_data = None

#Visualization Variables
safety_distance = 2
frontal = 1 #shifts the ellipse in front of a human

def ellipse_region():

    #initiate node at 10 Hz
    rospy.init_node('ellipse_region', anonymous = True)
    rate = rospy.Rate(10) #10 Hz
    
    #Subscribers
    rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, human_detect)  #human detection
    
    #Publishers
    marker_array = rospy.Publisher('/marker_array', MarkerArray, queue_size = 10)
    
    while not rospy.is_shutdown():
        if(human_detected == True):
            num_human = len(human_data.agent_states) #number of people
            for i in range(num_human):
                obstacle = [human_data.agent_states[i].pose.position.x, human_data.agent_states[i].pose.position.y,
                        custom_quat.euler_from_quaternion(human_data.agent_states[i].pose.orientation.x,
                        human_data.agent_states[1].pose.orientation.y,
                        human_data.agent_states[i].pose.orientation.z,
                        human_data.agent_states[i].pose.orientation.w)[2]]
                #print("Obstacle at: ", obstacle)
                marker_ar = []
                marker_ar = DrawEllipse(obstacle, safety_distance, i)
                marker_array.publish(marker_ar)
        else:
            print(human_detected, "false!")


def DrawEllipse(obstacle, safety_distance, human_num):
    num_points = 20 #total number of points in one dimension (20 is good)
    side = safety_distance*4
    resolution = side/num_points 
    
    #Construct x,y array of points around human position
    x =  []
    y = []
    
    for i in range(num_points):
        x.append(obstacle[0] - side/2 + resolution*i)
        y.append(obstacle[1] - side/2 + resolution*i)
    
    #Publish Markers for each point
    marker_ar = []
    id = 0 + (human_num - 1)*num_points**2 #generates unique id for every human

    for i in x:
        for j in y:
            #shift origin, then rotate frame with ellipse, and print marker within region
            [x_sh, y_sh] = [i - obstacle[0], j - obstacle[1]] #shift origin
            func = (x_sh*ca.cos(obstacle[2])+y_sh*ca.sin(obstacle[2]) - frontal)**2 + (2*(-x_sh*ca.sin(obstacle[2])+y_sh*ca.cos(obstacle[2])))**2 - safety_distance**2 
            if(func < 0.0):
                marker_ar.append(Marker_Append(i, j, id)) #store, for each point, a marker
            else:
                marker_ar.append(Marker_Append(0, 0, id)) #store the point at origin if outside of ellipse
            id += 1
    return marker_ar
            
            
            
def Marker_Append(i, j, id):
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.id = id
    marker.pose.position.x = i
    marker.pose.position.y = j
    marker.pose.position.z = 0
    marker.pose.orientation.w = 1
    marker.color.r = 0
    marker.color.g = 1
    marker.color.b = 0
    marker.color.a = 1
    return marker

def human_detect(data):
    global human_data, human_detected #modify original variables
    human_data = data
    human_detected = True
    #print(data.agent_states[0])
    #print(data.agent_states[0].pose.orientation.x)
    """ print(np.rad2deg(custom_quat.euler_from_quaternion(data.agent_states[0].pose.orientation.x,
    data.agent_states[0].pose.orientation.y,
    data.agent_states[0].pose.orientation.z,
    data.agent_states[0].pose.orientation.w)[2])) #for human angle"""
    print(len(data.agent_states))

if __name__ == '__main__':
    try:
        ellipse_region()
    except rospy.ROSInterruptException:
        pass


