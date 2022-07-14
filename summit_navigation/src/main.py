#! /usr/bin/env python

import rospy
from summit_navigation.srv import SummitGoal, SummitGoalResponse
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose
import yaml


class GoToLocation:
    """
    A service server that takes a pre-registered label (string) as request, and excutes an action to go
    to that label's coordinates.

    The labels are stored in the spot_recorder package: ~/catkin_ws/spot_recorder/spots.text
    """

    def __init__(self):
        self.service = rospy.Service(
            "goto_loc", SummitGoal, self.serviceCallback)

        self.action_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        self.action_client.wait_for_server()

    def serviceCallback(self, msg):
        coord = self.getLabelCoord(msg.label)

        if not coord:
            rospy.logerr(
                "Invalid location label received. Ensure this label has been registered by the spot_recorder node")
            return SummitGoalResponse(False, "Invalid Location Label")

        goal = MoveBaseGoal()
        goal.target_pose.pose.position.x = coord['position']['x']
        goal.target_pose.pose.position.y = coord['position']['y']
        goal.target_pose.pose.orientation.z = coord['orientation']['z']
        goal.target_pose.header.frame_id = "map"

        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()
        status = self.action_client.get_state()

        if status < 2:
            rospy.logerr("Error navigating to the location '" +
                         msg.label + "'.")
            return SummitGoalResponse(False, "Failed to navigate successfully")

        return SummitGoalResponse(True, "Destination Reached")

    def getLabelCoord(self, label):
        # with open("/home/user/catkin_ws/src/spot_recorder/spots.txt") as f:
        # spots = f.read()

        stream = open("/home/user/catkin_ws/src/spot_recorder/spots.yaml", "r")
        doc = yaml.load(stream)

        for entry in doc:
            if entry["label"] == label:
                return entry

        return None

        # coord = Pose()
        # if label == "Room":
        #     coord.position.x = -3.252
        #     coord.position.y = -3.508
        #     coord.orientation.z = -0.185
        #     return coord

        # if label == "Turtle":
        #     coord.position.x = -1.987
        #     coord.position.y = 3.980
        #     coord.orientation.z = 0.632
        #     return coord

        # if label == "Table":
        #     coord.position.x = 3.907
        #     coord.position.y = 1.961
        #     coord.orientation.z = 0.881
        #     return coord

        # return None


if __name__ == "__main__":
    rospy.init_node("summit_navigation", log_level=rospy.INFO)

    goto_loc = GoToLocation()

    rospy.spin()
