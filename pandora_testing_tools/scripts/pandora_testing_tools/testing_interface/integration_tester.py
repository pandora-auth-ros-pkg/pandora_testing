#!usr/bin/env python

import rospy
from pandora_testing_tools.testing_interface import alert_delivery
from pandora_testing_tools.mocks import mock_gui
from pandora_testing_tools.mocks import mock_navigation
from pandora_testing_tools.mocks import mock_end_effector_planner

class IntegrationTester():

    def __init__(self, mocking_navigation = True, mocking_end_effector_planner = True,
        mocking_gui = True):

        if mocking_navigation:
            self.navigation = mock_navigation.MockNavigation(
                do_exploration_topic = '/do_exploration',
                move_base_topic = '/move_base')
            rospy.loginfo("[IntegrationTester] Movk navi ok")
        if mocking_gui:
            self.gui = mock_gui.MockGui(
                gui_validation_topic = '/gui/validate_victim')
            rospy.loginfo("[IntegrationTester] Movk gui ok")
        if mocking_end_effector_planner:
            self.end_effector_planner = mock_end_effector_planner.MockEndEffectorPlanner(
                end_effector_planner_topic = '/control/move_end_effector_planner_action')
            rospy.loginfo("[IntegrationTester] Movk end effector planner ok")
        self.delivery_boy = alert_delivery.AlertDeliveryBoy('headCamera')
        rospy.loginfo("[IntegrationTester] Alert deliverery boy ok")
