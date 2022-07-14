#! /usr/bin/env python

import rospy
from nav_msgs.srv import GetMap

rospy.init_node("get_map_data")
rospy.loginfo("get_map_data node started")

service = rospy.ServiceProxy("static_map", GetMap)
service.wait_for_service()
rospy.loginfo("/static_map service is ready")

response = service()
rospy.loginfo("Static Map Data:\n- Resolution: {}\n- Dimension: {} x {}".format(
    response.map.info.resolution, response.map.info.width, response.map.info.height))
