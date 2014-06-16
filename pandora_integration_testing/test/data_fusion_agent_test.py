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

PKG = 'pandora_integration_testing'
NAME = 'hole_data_fusion_test'

import sys

import unittest

import roslib; roslib.load_manifest(PKG)
import rostest
import rospy

from pandora_testing_tools.testing_interface import test_base
from pandora_testing_tools.testing_interface import alert_delivery
from pandora_testing_tools.mocks import mock_navigation
from pandora_testing_tools.mocks import mock_gui
from pandora_data_fusion_msgs.srv import GetObjectsSrv
from pandora_data_fusion_msgs.srv import GetObjectsSrvResponse
from state_manager_communications.msg import robotModeMsg

class DataFusionAgentTest(test_base.TestBase):

    def setUp(self):

        self.get_objects = rospy.ServiceProxy('/data_fusion/get_objects', GetObjectsSrv, True)
        rospy.wait_for_service('/data_fusion/get_objects')
        self.mockNavigation = mock_navigation.MockNavigation(
            do_exploration_topic = '/navigation/do_exploration',
            move_base_topic = '/move_base')
        self.mockGui = mock_gui.MockGui(
            gui_validation_topic = '/gui/validate_victim')
        self.deliveryBoy = alert_delivery.AlertDeliveryBoy()

    def tearDown(self):

        self.mockGui.__del__()
        self.mockNavigation.__del__()

    def test_part_1(self):
        
        #  Changes robot state to start.
        self.state_changer.transition_to_state(
            robotModeMsg.MODE_START_AUTONOMOUS)
        #  A hole is found.
        self.deliveryBoy.deliverHoleAlert(yaw = 0, pitch = 0, probability = 0.8)
        self.deliveryBoy.deliverHoleAlert(yaw = 0, pitch = 0, probability = 0.9)
        #  Qualifies to a victim and is sent to agent.
        self.assertEqual(1, len(self.messageList[1][0].victims))
        outs = self.get_objects()
        self.assertEqual(1, len(outs.holes))
        self.assertEqual(0, len(outs.faces))
        self.assertEqual(1, len(outs.victimsToGo))
        self.assertEqual(0, len(outs.victimsVisited))
        victimPose = outs.victimsToGo[0].pose
        rospy.sleep(1)

    def test_part_2(self):

        #  Agent recognises new victim and orders navigation to go to it.
        self.assertEqual(3, self.messageList[0][-1].mode)
        self.assertTrue(self.mockNavigation.moved_base)
        self.mockNavigation.move_base_succedes = True
        self.assertEqual(0, self.mockNavigation.preempted)
        self.assertEqual(0, 
            test_base.distance(victimPose.position, self.mockNavigation.target_position))
        self.assertTrue(test_base.isSameOrientation(
          victimPose.orientation, self.mockNavigation.target_orientation))
        self.mockNavigation.reply = True
        rospy.sleep(1)

    def test_part_3(self):

        #  Navigation went to the victim and a face is associated with the victim.
        self.assertEqual(4, self.messageList[0][-1].mode)
        self.deliveryBoy.deliverFaceAlert(yaw = 0, pitch = 0, probability = 0.85)
        outs = self.get_objects()
        self.assertEqual(1, len(outs.holes))
        self.assertEqual(1, len(outs.faces))
        self.assertEqual(1, len(outs.victimsToGo))
        self.assertEqual(0, len(outs.victimsVisited))
        victimBefore = outs.victimsToGo[0].pose.position
        self.assertEqual(1, len(self.messageList[1][-1].victims))
        victim_probability = self.messageList[1][-1].victims[0].probability
        rospy.sleep(1)

    def test_part_4(self):

        #  Agent recognises the change in victim and verifies it. Asks Gui for
        #  validation. Gui validates the victim.
        self.mockGui.victimValid = True
        self.assertEqual(victim_probability, self.mockGui.probability)
        self.assertEqual(["FACE"], self.mockGui.sensorIDsFound)
        self.mockGui.reply = True
        rospy.sleep(1)

    def test_part_5(self):

        #  Agent informs Data Fusion of victim's validation status and orders
        #  navigation to explore. Victim is transfered to victimsVisited list.
        self.assertEqual(2, self.messageList[0][-1].mode)
        self.assertTrue(self.mockNavigation.entered_exploration)
        outs = self.get_objects()
        self.assertEqual(1, len(outs.holes))
        self.assertEqual(1, len(outs.faces))
        self.assertEqual(0, len(outs.victimsToGo))
        self.assertEqual(1, len(outs.victimsVisited))
        victimAfter = outs.victimsVisited[0].pose.position
        self.assertEqual(0, test_base.distance(victimBefore, victimAfter))
        rospy.sleep(1)

    def test_part_6(self):

        #  Another hole is detected and qualifies to victim.
        self.deliveryBoy.deliverHoleAlert(yaw = -1, pitch = 0, probability = 0.5)
        self.deliveryBoy.deliverHoleAlert(yaw = -1, pitch = 0, probability = 0.8)
        self.assertEqual(1, len(self.messageList[1][-1].victims))
        outs = self.get_objects()
        self.assertEqual(2, len(outs.holes))
        self.assertEqual(1, len(outs.faces))
        self.assertEqual(1, len(outs.victimsToGo))
        self.assertEqual(1, len(outs.victimsVisited))
        victimPose = outs.victimsToGo[0].pose
        rospy.sleep(1)

    def test_part_7(self):

        #  Agent is informed and orders navigation to go to it but navigation
        #  aborts the order...
        self.assertEqual(3, self.messageList[0][-1].mode)
        self.assertTrue(self.mockNavigation.moved_base)
        self.mockNavigation.move_base_succedes = False
        self.assertEqual(0, self.mockNavigation.preempted)
        self.assertEqual(0, 
            test_base.distance(victimPose.position, self.mockNavigation.target_position))
        self.assertTrue(test_base.isSameOrientation(
          victimPose.orientation, self.mockNavigation.target_orientation))
        self.mockNavigation.reply = True
        rospy.sleep(1)

    def test_part_8(self):

        #  Agents acknowledges victim as potential noise and orders Data Fusion
        #  to delete it. Navigation is ordered to return to exploration.
        self.assertEqual(2, self.messageList[0][-1].mode)
        self.assertTrue(self.mockNavigation.entered_exploration)
        outs = self.get_objects()
        self.assertEqual(1, len(outs.holes))
        self.assertEqual(1, len(outs.faces))
        self.assertEqual(0, len(outs.victimsToGo))
        self.assertEqual(1, len(outs.victimsVisited))
        rospy.sleep(1)

    def test_part_9(self):

        #  Another hole is detected and qualifies to a victim.
        self.deliveryBoy.deliverHoleAlert(yaw = 1, pitch = 0, probability = 0.8)
        self.deliveryBoy.deliverHoleAlert(yaw = 1, pitch = 0, probability = 0.8)
        self.assertEqual(1, len(self.messageList[1][-1].victims))
        outs = self.get_objects()
        self.assertEqual(2, len(outs.holes))
        self.assertEqual(1, len(outs.faces))
        self.assertEqual(1, len(outs.victimsToGo))
        self.assertEqual(1, len(outs.victimsVisited))
        victimPose = outs.victimsToGo[0].pose
        rospy.sleep(1)

    def test_part_10(self):

        #  Agent recognises new victim and orders navigation to go to it.
        self.assertEqual(3, self.messageList[0][-1].mode)
        self.assertTrue(self.mockNavigation.moved_base)
        self.assertEqual(0, self.mockNavigation.preempted)
        self.assertEqual(0, 
            test_base.distance(victimPose.position, self.mockNavigation.target_position))
        self.assertTrue(test_base.isSameOrientation(
          victimPose.orientation, self.mockNavigation.target_orientation))
        rospy.sleep(1)

    def test_part_11(self):

        #  Current victim is thought to be displaced.
        self.deliveryBoy.deliverHoleAlert(yaw = 0.97, pitch = 0, probability = 0.9)
        self.deliveryBoy.deliverHoleAlert(yaw = 0.95, pitch = 0, probability = 0.9)
        self.deliveryBoy.deliverHoleAlert(yaw = 0.94, pitch = 0, probability = 0.9)
        self.assertEqual(1, len(self.messageList[1][-1].victims))
        outs = self.get_objects()
        self.assertEqual(2, len(outs.holes))
        self.assertEqual(1, len(outs.faces))
        self.assertEqual(1, len(outs.victimsToGo))
        self.assertEqual(1, len(outs.victimsVisited))
        victimPose = outs.victimsToGo[0].pose
        rospy.sleep(1)

    def test_part_12(self):

        #  Agent think that displacement cannot be ignored and re-orders navigation.
        #  Navigation target has changed.
        self.assertEqual(3, self.messageList[0][-1].mode)
        self.assertTrue(self.mockNavigation.moved_base)
        self.assertEqual(1, self.mockNavigation.preempted)
        self.assertEqual(0, 
            test_base.distance(victimPose.position, self.mockNavigation.target_position))
        self.assertTrue(test_base.isSameOrientation(
          victimPose.orientation, self.mockNavigation.target_orientation))
        self.mockNavigation.move_base_succedes = True
        self.mockNavigation.reply = True
        rospy.sleep(1)

    def test_part_13(self):

        #  Agent waits for verification but victim fails to verify...
        #  Agent orders Data Fusion to think this victim as non valid.
        #  Navigation returns to exploration.
        rospy.sleep(15)
        self.assertEqual(2, self.messageList[0][-1].mode)
        self.assertTrue(self.mockNavigation.entered_exploration)
        outs = self.get_objects()
        self.assertEqual(2, len(outs.holes))
        self.assertEqual(1, len(outs.faces))
        self.assertEqual(0, len(outs.victimsToGo))
        self.assertEqual(2, len(outs.victimsVisited))

if __name__ == '__main__':

    rospy.sleep(15)
    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
    subscriber_topics = [
        ("/robot/state/clients", "state_manager_communications", "robotModeMsg"),
        ("/data_fusion/world_model", "pandora_data_fusion_msgs", "WorldModelMsg")]
    DataFusionAgentTest.connect(subscriber_topics, list())
    rostest.rosrun(PKG, NAME, DataFusionAgentTest, sys.argv)
    DataFusionAgentTest.disconnect()

