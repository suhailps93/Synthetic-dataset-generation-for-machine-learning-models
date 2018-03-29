#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
import random
import math
import time
from std_srvs.srv import Empty as EmptySrv
count_angle=1

camera_state = ModelState()
camera_state.model_name = 'camera'
camera_state.pose.position.x = 1
camera_state.pose.position.y = 1
camera_state.pose.position.z = 0.5
camera_state.reference_frame = 'world'

model_state = ModelState()
model_state.model_name = 'tide'
model_state.pose.position.x = 0
model_state.pose.position.y = 0
model_state.pose.position.z = 0.5
model_state.reference_frame = 'world'


def set_state(x,y,z,w):
    camera_state.pose.position.x = x
    camera_state.pose.position.y = y
    camera_state.pose.orientation.z =z
    camera_state.pose.orientation.w = w
    set_model_state(camera_state)
    time.sleep(1)

def set_state_angle(x,y,count):
    camera_state.pose.position.x = x
    camera_state.pose.position.y = y
    camera_state.pose.orientation.z =math.sin((3.925-count*0.785)/2)
    camera_state.pose.orientation.w = math.cos((3.925-count*0.785)/2)
    set_model_state(camera_state)
    time.sleep(1)

def set_state_model():
    global count_angle

    if count_angle<8:
        model_state.pose.orientation.y =math.sin(count_angle*0.785/2)
        model_state.pose.orientation.w =math.cos(count_angle*0.785/2)
    elif count_angle <16 :
        model_state.pose.orientation.x =math.sin(count_angle*0.785/2)
        model_state.pose.orientation.w =math.cos(count_angle*0.785/2)
    count_angle=count_angle+1

    # else:
    #     model_state.pose.orientation.x=0.382*count
    #     model_state.pose.orientation.y =0.382*count


    set_model_state(model_state)
    time.sleep(1)

def move_cameras():

    set_state_angle(1,1,0)
    set_state_angle(1,0,1)
    set_state_angle(1,-1,2)
    set_state_angle(0,-1,3)
    set_state_angle(-1,-1,4)
    set_state_angle(-1,0,5)
    set_state_angle(-1,1,6)
    set_state_angle(0,1,7)
    set_state_model()







if __name__=='__main__':


    rospy.init_node('set_model_state')
    g_pause = rospy.ServiceProxy("/gazebo/pause_physics", EmptySrv)
    rospy.wait_for_service('/gazebo/pause_physics')
    g_pause()

    time.sleep(2)
    while not rospy.is_shutdown():
        rospy.wait_for_service('/gazebo/set_model_state')
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_model_state(model_state)
        time.sleep(1)

        move_cameras()


