#!/usr/bin/env python

#/*******************************************************************************
# * Copyright (c) 2011 Martin Frassl, Michael Lichtenstern
# * All rights reserved.
# * 
# * Redistribution and use in source and binary forms, with or without modification, 
# * are permitted provided that the following conditions are met:
# * 
# *     * Redistributions of source code must retain the above copyright notice, this 
# *       list of conditions and the following disclaimer.
# *     * Redistributions in binary form must reproduce the above copyright notice, this 
# *       list of conditions and the following disclaimer in the documentation and/or 
# *       other materials provided with the distribution.
# * 
# * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
# * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
# * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
# * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
# * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED 
# * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
# * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY 
# * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ******************************************************************************/

import roslib; roslib.load_manifest('rosworldwind')
import rospy
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose as MarkerPose
from turtlesim.msg import Pose as TurtlePose

from std_msgs.msg import ColorRGBA

from tf import transformations

pub = 0

def talk(data):
    
    global pub
    
    msg = Marker()
    
    msg.header.frame_id = "testid";
    msg.header.stamp = rospy.Time.now()

    
    msg.id = 1
    msg.type = 1
    
    q = transformations.quaternion_from_euler(0, 0, -1 * data.theta, 'rxyz')
    
    pose = MarkerPose()
    
    pose.position.x = data.x * 10000
    pose.position.y = data.y * 10000
    pose.position.z = 10000
    
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    
    msg.pose = pose
    
    msg.scale.x = 40000
    msg.scale.y = 40000
    msg.scale.z = 40000
    
    msg.color = ColorRGBA(0, 0, 1, 1)
    
    rospy.loginfo(msg)
    pub.publish(msg)
    

def callback(data):
    talk(data)

def talker():
    global pub 
    pub = rospy.Publisher('turtleMarker', Marker)
    rospy.init_node('turtleTransformator')

def listener():
    talker()
    #rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/turtle1/pose", TurtlePose, callback)
    rospy.spin()

if __name__ == '__main__':
    
    listener();

