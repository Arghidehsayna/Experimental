#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

class MarkerDetection:
    def __init__(self):
        rospy.init_node('marker_detection', anonymous=True)

        # Publishers
        self.marker_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)

        # Subscribers
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)

    def gazebo_callback(self, data):
        # Extract marker information from Gazebo data
        markers = self.extract_markers(data)

        # Convert markers to MarkerArray
        marker_array = self.convert_to_marker_array(markers)

        # Publish the MarkerArray to RViz
        self.marker_array_pub.publish(marker_array)

    def extract_markers(self, model_states):
        markers = []

        # Customize this part based on your marker data structure
        for model_name, pose in zip(model_states.name, model_states.pose):
            if "marker" in model_name.lower():
                markers.append((model_name, pose))

        return markers

    def convert_to_marker_array(self, markers):
        marker_array = MarkerArray()

        # Process extracted markers and convert them to RViz visualization markers
        for idx, (model_name, pose) in enumerate(markers):
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = "top"  # Replace with your reference frame
            marker.ns = "aruco_markers"
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.2  # Adjust size
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        return marker_array

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        marker_detection = MarkerDetection()
        marker_detection.run()
    except rospy.ROSInterruptException:
        pass

