#!/usr/bin/env python
# Software License Agreement
__version__ = "0.0.1"
__status__ = "Production"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2014, P.A.N.D.O.R.A. Team. All rights reserved."
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of P.A.N.D.O.R.A. Team nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
__author__ = "Tsirigotis Christos"
__maintainer__ = "Tsirigotis Christos"
__email__ = "tsirif@gmail.com"

import math

import unittest

import rospy
import roslib
import actionlib
import sys
import rosbag

from pandora_testing_tools.msg import ReplayBags.action
from state_manager.state_client import StateClient
from geometry_msgs.msg import Point

def distance(a, b):

    return math.sqrt( (a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2 )

def direction(a, b):
        
    dire = Point()
    norm = distance(a, b)
    dire.x = (b.x - a.x)/norm
    dire.y = (b.y - a.y)/norm
    dire.z = (b.z - a.z)/norm
    return dire

class TestBase(unittest.TestCase):

    def mockCallback(self, data):

        self.alertList.append(data)
        self.replied = True
        rospy.logdebug(self.alertList)

    @classmethod
    def connect(cls, subscriber_topics, publisher_topics):

        cls.state_changer = StateClient(False)
        rospy.sleep(0.1)
        cls.state_changer.transition_to_state(2)
        rospy.sleep(2)

        cls.subscriber_topics = subscriber_topics
        cls.publisher_topics = publisher_topics

    def setUp(self):

        self.subscribers = []
        for topic, messagePackage, messageType in self.subscriber_topics:
            _temp = __import__(messagePackage+'.msg', globals(), locals(), messageType, -1)
            messageTypeObj = getattr(_temp, messageType)
            mock_subscriber = rospy.Subscriber(
                topic, 
                messageTypeObj, self.mockCallback)
            self.subscribers.append(mock_subscriber)
        self.alertList = []
        self.replied = False
        
        self.publishers = []
        for topic, messagePackage, messageType in self.publisher_topics:
            _temp = __import__(messagePackage+'.msg', globals(), locals(), messageType, -1)
            messageTypeObj = getattr(_temp, messageType)
            mock_publisher = rospy.Publisher(topic, messageTypeObj)
            self.publishers.append(mock_publisher)

        self.bag_client = actionlib.SimpleActionClient('/test/bag_player', ReplayBagsAction)
        self.bag_client.wait_for_server()
        self.goal = ReplayBagsGoal()
        self.goal.start = True

    def tearDown(self):

        for mock_subscriber in self.subscribers:
            mock_subscriber.unregister()
        for mock_publisher in self.publishers:
            mock_publisher.unregister()

    def playFromBag(self, block):

        self.bag_client.send_goal(self.goal)
        if block is True:
            self.bag_client.wait_for_result()

