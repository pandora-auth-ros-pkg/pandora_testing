#!/usr/bin/env python
PKG = 'node_tests'
import roslib; 
import rospy
import actionlib
import unittest
import mox
from time import sleep
import rosbag
import sys
import mocksubscriber
import moxcomparators

from node_tests_msgs.msg import *

class SubscriberMockUnitTest(unittest.TestCase):

  def setUp(self):
    self.mocker = mox.Mox()
    
  def testUsingMox(self):
    
    mock_obj = self.mocker.CreateMock(mocksubscriber.mockSubscriber)
    
    for topic, msg, t in rosbag.Bag(bagfile).read_messages():    
      if topic in topicsExact :
        compList = []
        mock_obj.callMethod(moxcomparators.msgEquals(msg))
        
    self.mocker.ReplayAll()
    self.rsub = mocksubscriber.realSubscriber(topicsExact, messageTypeObj, mock_obj)
    
    client = actionlib.SimpleActionClient('/replay_bags', ReplayBagsAction)
    client.wait_for_server()
    rospy.loginfo('ready')

    goal = ReplayBagsGoal()
    goal.start = True
    client.send_goal(goal)
    client.wait_for_result()
    
    self.mocker.VerifyAll()
    
    rospy.loginfo('exiting')


if __name__ == '__main__':
  
    import rostest
    
    argv = rospy.myargv(argv=sys.argv)
  
    messagePackage = argv[1]
    messageType = argv[2]
    bagfile = argv[3]
    topicsExact = argv[4]
    
    _temp = __import__(messagePackage+'.msg', globals(), locals(), messageType, -1)
    messageTypeObj = getattr(_temp, messageType)
    
    rospy.init_node('tester')
    rostest.rosrun(PKG, 'test_using_mox', SubscriberMockUnitTest) 

  
