#!/usr/bin/env python
import rospy
import numpy as np
import tf
import tf.msg
from visualization_msgs.msg import Marker

rospy.init_node('robot_markers', anonymous=True)

self.food_publishers[cl] = rospy.Publisher('/detector/'+self.object_labels[cl],
                        DetectedObject, queue_size=10)

rospy.Subscriber('/detector/pizza', DetectedObject, self.food_detected_callback)
rospy.Subscriber('/detector/banana', DetectedObject, self.food_detected_callback)
rospy.Subscriber('/detector/apple', DetectedObject, self.food_detected_callback)
rospy.Subscriber('/detector/sandwich', DetectedObject, self.food_detected_callback)
rospy.Subscriber('/detector/orange', DetectedObject, self.food_detected_callback)
rospy.Subscriber('/detector/broccoli', DetectedObject, self.food_detected_callback)
rospy.Subscriber('/detector/carrot', DetectedObject, self.food_detected_callback)
rospy.Subscriber('/detector/hot_dog', DetectedObject, self.food_detected_callback)
rospy.Subscriber('/detector/donut', DetectedObject, self.food_detected_callback)
rospy.Subscriber('/detector/cake', DetectedObject, self.food_detected_callback)

robot_marker_publisher = rospy.Publisher("/robot_marker", Marker, queue_size=10)
#cmd_pose_marker_publisher = rospy.Publisher("/cmd_pose_marker", Marker, queue_size=10)



trans_listener = tf.TransformListener()

while not rospy.is_shutdown():
    robot_marker = Marker(type=Marker.CYLINDER, id=0, lifetime=rospy.Duration()))
    try:
        origin_frame = "/map" if mapping else "/odom"
        (translation,rotation) = trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
        x = translation[0]
        y = translation[1]
        euler = tf.transformations.euler_from_quaternion(rotation)
        theta = euler[2]
        robot_marker.pose.orientation.w = theta
        robot_marker.pose.position.x = x
        robot_marker.pose.position.y = y
        robot_marker.pose.position.z = 0.0
        robot_marker.header.frame_id = "/robot_marker"
        robot_marker.header.stamp = rospy.Time(0)
        robot_marker.scale.x = 0.15
        robot_marker.scale.y = 0.15
        robot_marker.color.g = 0.0
        robot_marker.color.r = 0.0
        robot_marker.color.b = 1.0
        robot_marker.color.a = 0.0

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

    publisher.Publish(robot_marker)
    rate.sleep()



