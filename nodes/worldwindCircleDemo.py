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
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

from std_msgs.msg import ColorRGBA

from tf import transformations

def talker():
    pub = rospy.Publisher('marker01', Marker)
    rospy.init_node('testtalker')
    
    i = 0
    steps = 50
    
    while not rospy.is_shutdown():
        
        msg = Marker()
        
        msg.header.frame_id = "testid";
        msg.header.stamp = rospy.Time.now()
        
        radius = 100000
        
        msg.id = 1
        msg.type = 1
        
        pose = Pose()
        
        pose.position.x = radius* math.sin(1.0/steps * i * 2 * math.pi)
        pose.position.y = -radius * -1*math.cos(1.0/steps * i * 2 * math.pi)
        q = transformations.quaternion_from_euler(0, 0, 1.0/steps * 2 * i * math.pi, 'rxyz')
        
        pose.position.z = 10
        
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        
        msg.pose = pose
        
        msg.scale.x = 100000
        msg.scale.y = 100000
        msg.scale.z = 100000
        
        msg.color = ColorRGBA(0, 1, 0, 1)
        
        rospy.loginfo(msg)
        pub.publish(msg)
        
        i=i+1
        if i==steps:
           i=0
        rospy.sleep(0.1)
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

