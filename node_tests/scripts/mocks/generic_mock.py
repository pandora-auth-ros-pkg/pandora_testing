#!/usr/bin/env python
import roslib; roslib.load_manifest('node_tests')
import rospy
import actionlib
import unittest
import mox
from time import sleep
import rosbag
import sys
#~ import messageInfoParser
import mocksubscriber
import moxcomparators
import bcolors

from node_tests_msgs.msg import *

class SubscriberMockUnitTest(unittest.TestCase):

  def setUp(self):
    self.person_mocker = mox.Mox()
    
  def testUsingMox(self):
    
    dao = self.person_mocker.CreateMock(mocksubscriber.mockSubscriber)
    
    for topic, msg, t in rosbag.Bag(bagfile).read_messages():    
      if topic in topicsExact :
        compList = []
        dao.callMethod(moxcomparators.msgEquals(msg))
        
    self.person_mocker.ReplayAll()
    self.rsub = mocksubscriber.realSubscriber(topicsExact, messageTypeObj, dao)
    
    client = actionlib.SimpleActionClient('/replay_bags', ReplayBagsAction)
    client.wait_for_server()
    rospy.loginfo('ready')

    goal = ReplayBagsGoal()
    goal.start = True
    client.send_goal(goal)
    client.wait_for_result()
    
    self.person_mocker.VerifyAll()
    
    rospy.loginfo('exiting')


if __name__ == '__main__':
  
  argv = rospy.myargv(argv=sys.argv)
  
  messagePackage = argv[1]
  messageType = argv[2]
  bagfile = argv[3]
  topicsExact = argv[4]
  
  #~ pathToMsgDeclaration = argv[5]

  _temp = __import__(messagePackage+'.msg', globals(), locals(), messageType, -1)
  messageTypeObj = getattr(_temp, messageType)

  #~ messageGoal = messageType[:-10]+messageType[-4:]
  #~ fieldList = messageInfoParser.parseMessage(pathToMsgDeclaration + '/msg/' + messageGoal + '.msg')

  #~ for i in fieldList:
    #~ print i
    
  rospy.init_node('tester')
  suite = unittest.TestLoader().loadTestsFromTestCase(SubscriberMockUnitTest)
  unittest.TextTestRunner(verbosity=2).run(suite)
  
