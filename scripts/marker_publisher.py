#!/usr/bin/env python

import rospy
import tf
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose2D, PoseStamped
from asl_turtlebot_project.msg import FoodLoc, FoodLocList, DetectedObject

class Visualizer:
    
    def __init__(self):
        rospy.init_node('marker_publisher', anonymous=True)
        # current state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # goal state
        self.x_g = 0.0
        self.y_g = 0.0
        self.theta_g = 0.0
        
        # food location listener
        self.food_loc = []
        
        self.stop_sign_marker = MarkerArray()

        # tf listener
        self.trans_listener = tf.TransformListener()
        
        #subscribe to stop sign detector
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_callback)

        #subscribe to puddle detector
        #rospy.Subscriber('/coords_puddle', Pose2D, self.current_puddle_coords_callback )

        # subscribe to cmd_post
        rospy.Subscriber('/cmd_pose', Pose2D, self.cmd_pose_callback)
        
        # subscribe to food_loc
        rospy.Subscriber('/food_loc', FoodLocList, self.food_callback)
        
        # publishers
        topic = 'visualization_marker'
        self.publisher_stop_sign = rospy.Publisher(topic, MarkerArray, queue_size=10)
        topic_stop_sign = 'stop_sign_marker'
        self.publisher = rospy.Publisher(topic_stop_sign, Marker, queue_size=10)
        topic_goal = 'goal_marker'
        self.publisher_goal = rospy.Publisher(topic_goal, Marker, queue_size=10)
        topic_food = 'food_marker_array'
        self.publisher_food = rospy.Publisher(topic_food, MarkerArray, queue_size=10)

    def cmd_pose_callback(self, msg):
        self.x_g = msg.x
        self.y_g = msg.y
        self.theta_g = msg.theta  
    
    def stop_sign_callback(self, msg):
        print("Found stop sign!")
        stop_sign_marker = Marker(type=Marker.CYLINDER, id=0, lifetime=rospy.Duration())
        try:
            origin_frame = "/map"
            (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, 
                                                                    '/base_footprint', 
                                                                    rospy.Time(0))
            self.x = translation[0]
            self.y = translation[1]
            euler = tf.transformations.euler_from_quaternion(rotation)
            self.theta = euler[2]
            
            marker.pose.orientation.w = self.theta
            marker.pose.position.x = self.x
            marker.pose.position.y = self.y
            marker.pose.position.z = 0.0
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        marker.header.frame_id = '/map'
        marker.header.stamp = rospy.Time(0)
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.b = 1.0
        marker.color.g = 0.0
        marker.color.a = 0.8
        self.stop_sign_marker.markers.append(marker)

        text_marker = Marker(type=Marker.TEXT_VIEW_FACING, id=1, lifetime=rospy.Duration())
        text_marker.header.frame_id = '/map'
        text_marker.header.stamp = rospy.Time(0)
        text_marker.action = text_marker.ADD
        text_marker.text = 'Stop'
        text_marker.scale.z = 0.15
        text_marker.color.a = 1.0
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.pose.position.z = 0.0
        text_marker.pose.position.x = self.x
        text_marker.pose.position.y = self.y
        self.stop_sign_marker.markers.append(text_marker)

    def food_callback(self, msg):
        print("Got food message!")
        self.food_loc = msg.ob_msgs
    
    def create_pose_marker(self):
        marker = Marker(type=Marker.CYLINDER, id=0, lifetime=rospy.Duration())
        try:
            origin_frame = "/map"
            (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, 
                                                                    '/base_footprint', 
                                                                    rospy.Time(0))
            self.x = translation[0]
            self.y = translation[1]
            euler = tf.transformations.euler_from_quaternion(rotation)
            self.theta = euler[2]
            
            marker.pose.orientation.w = self.theta
            marker.pose.position.x = self.x
            marker.pose.position.y = self.y
            marker.pose.position.z = 0.0
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        marker.header.frame_id = '/map'
        marker.header.stamp = rospy.Time(0)
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.b = 0.0
        marker.color.g = 0.0
        marker.color.a = 0.8
        return marker
        
    def create_goal_marker(self):
        # Create goal marker_publisher
        goal_marker = Marker(type=Marker.CYLINDER, id=0, lifetime=rospy.Duration())
        goal_marker.pose.orientation.w = self.theta_g
        goal_marker.pose.position.x = self.x_g
        goal_marker.pose.position.y = self.y_g
        goal_marker.pose.position.z = 0.0
        goal_marker.header.frame_id = '/map'
        goal_marker.header.stamp = rospy.Time(0)
        goal_marker.scale.x = 0.15
        goal_marker.scale.y = 0.15
        goal_marker.scale.z = 0.3
        goal_marker.color.r = 1.0
        goal_marker.color.b = 0.0
        goal_marker.color.g = 0.0
        goal_marker.color.a = 0.8
        return goal_marker
        
    def create_food_marker(self):
        markerArray = MarkerArray()
        foodId = 0
        for item in self.food_loc:
            #print("Adding marker for ", item.name)
            marker = Marker(type=Marker.SPHERE, id=foodId, lifetime=rospy.Duration())
            marker.header.frame_id = '/map'
            marker.header.stamp = rospy.Time(0)
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.6
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.z = 0.0
            marker.pose.position.x = item.x
            marker.pose.position.y = item.y
            markerArray.markers.append(marker)
            foodId += 1
            
            text_marker = Marker(type=Marker.TEXT_VIEW_FACING, id=foodId, lifetime=rospy.Duration())
            text_marker.header.frame_id = '/map'
            text_marker.header.stamp = rospy.Time(0)
            text_marker.action = text_marker.ADD
            text_marker.text = item.name
            text_marker.scale.z = 0.15
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.pose.position.z = 0.0
            text_marker.pose.position.x = item.x
            text_marker.pose.position.y = item.y - 0.12
            markerArray.markers.append(text_marker)
            foodId += 1
        return markerArray
            
        
    def run(self):
        rate = rospy.Rate(10) # 10 Hz

        # continuously publish marker 
        while not rospy.is_shutdown():
            
            marker = self.create_pose_marker()
            goal_marker = self.create_goal_marker()
            food_marker_array = self.create_food_marker()
            

            self.publisher.publish(marker)
            self.publisher_goal.publish(goal_marker)
            self.publisher_food.publish(food_marker_array)
            self.publisher_stop_sign.publish(self.stop_sign_marker)

            rate.sleep()
            
if __name__ == '__main__':
    mrkr = Visualizer()
    mrkr.run()
