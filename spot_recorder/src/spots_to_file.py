#! /usr/bin/env python

import os
import rospy
from spot_recorder.srv import RecordSpot, RecordSpotResponse
from geometry_msgs.msg import PoseWithCovarianceStamped


class SpotRecorder:
    def __init__(self, filename="./spots.txt"):
        self.spots = []
        self.pose = None
        self.filename = filename

        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped,
                         callback=self.pose_callback, queue_size=1)
        self.service = rospy.Service(
            "record_spot", RecordSpot, self.save_spot_callback)

    def pose_callback(self, msg):
        self.pose = msg.pose.pose

    def save_spot_callback(self, msg):
        if msg.label == "end":
            self.write_spots_to_file()
            rospy.loginfo("Spots data written to {}".format(self.filename))
            return RecordSpotResponse(success=True, filename=self.filename)

        self.spots.append({"label": msg.label, "pose": self.pose})
        rospy.loginfo("Recorded Spot {}".format(msg.label))
        return RecordSpotResponse(success=True, filename="")

    def write_spots_to_file(self):
        with open(self.filename, mode="w") as f:
            for spot in self.spots:
                data = "-\n"
                data += "  label: {}\n".format(spot['label'])
                data += "  position:\n"
                data += "    x: {}\n".format(spot["pose"].position.x)
                data += "    y: {}\n".format(spot["pose"].position.y)
                data += "  orientation:\n"
                data += "    z: {}\n".format(spot["pose"].orientation.z)

                f.write(data)
            f.close()
        self.spots = []


if __name__ == "__main__":
    rospy.init_node("spot_recorder")
    rospy.loginfo("Spot Recorder Node started")

    recorder = SpotRecorder(
        filename="/home/user/catkin_ws/src/spot_recorder/spots.yaml")
    rospy.spin()
