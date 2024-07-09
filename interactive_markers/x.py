#!/usr/bin/env python
# -*- coding: UTF-8 -*-
#Author ：LQL
import rospy
import copy
import time

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point,Pose,Twist
from tf.broadcaster import TransformBroadcaster
from tf.listener import TransformListener
from tf.transformations import *
from std_msgs.msg import String
from random import random
from math import *

server = None
menu_handler = MenuHandler()
br = None
counter = 0
global  pos
pos=Pose()

def processFeedback(feedback):
    global pos
    pos = feedback.pose
    server.applyChanges()

def makeArrow( msg ):
    marker = Marker()

    marker.type = Marker.ARROW
    marker.scale.x = msg.scale * 2.0
    marker.scale.y = msg.scale * 0.5
    marker.scale.z = msg.scale * 0.5
    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 0
    marker.color.a = 0.8

    return marker

def makeAdd( msg ):
    marker = Marker()
    marker.type = Marker.CYLINDER
    marker.scale.x = msg.scale * 0.5
    marker.scale.y = msg.scale * 0.5
    marker.scale.z = msg.scale * 3
    marker.color.r = 0
    marker.color.g = 0
    marker.color.b = 1
    marker.color.a = 1.0

    return marker

def makeMark():
    marker=Marker()
    marker.header.frame_id='base_link'
    marker.header.stamp=rospy.Time.now()
    marker.ns="basic_shapes"
    marker.id=10
    marker.type=Marker.ARROW
    marker.scale.x = 0.1
    marker.scale.y = 0.5
    marker.scale.z = 5
    marker.color.r = 0
    marker.color.g = 1
    marker.color.b = 0
    marker.color.a = 0.3
    marker.action=Marker.ADD
    marker.lifetime=rospy.Duration()
    return marker


def makeArrowControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeArrow(msg) )
    msg.controls.append( control )
    return control

def saveMarker( int_marker ):
  server.insert(int_marker, processFeedback)

def makeGoalMarker(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = position
    int_marker.scale = 0.5
    int_marker.name = "goal"
    int_marker.description = "goal"
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.always_visible = True
    control.markers.append(makeArrow(int_marker))
    int_marker.controls.append(copy.deepcopy(control))
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    control.markers.append(makeAdd(int_marker))
    int_marker.controls.append(copy.deepcopy(control))
    del control
    server.insert(int_marker, processFeedback)
    menu_handler.apply( server, int_marker.name )
if __name__ == "__main__":
    rospy.init_node("interactive_controls")
    listener= TransformListener()
    markerpub=rospy.Publisher('visualization_marker',Marker,queue_size=1)
    server = InteractiveMarkerServer("interactive_controls")
    while True:
        time.sleep(1)
        try:
            (trans, rot) = listener.lookupTransform('base_link', 'base_link', rospy.Time(0))
            position = Point(trans[0], trans[1], trans[2])
            print(position)
        except Exception:
            print(e)
        else:
            break
    makeGoalMarker(position)
    server.applyChanges()
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        markerpub.publish(makeMark())
        rate.sleep()