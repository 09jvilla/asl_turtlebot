#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
import tf
import math
from enum import Enum
import numpy as np
# import Collections.defaultdict

# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup (hw2+)
use_gazebo = rospy.get_param("sim")

# how is nav_cmd being decided -- human manually setting it, or rviz
rviz = rospy.get_param("rviz")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")


# threshold at which we consider the robot at a location
POS_EPS = .1
THETA_EPS = .3

# time to stop at a stop sign
STOP_TIME = 3

# minimum distance from a stop sign to obey it
STOP_MIN_DIST = .5

# time taken to cross an intersection
CROSSING_TIME = 3

# state machine modes, not all implemented
class Mode(Enum):
    IDLE = 1
    POSE = 2
    STOP = 3
    CROSS = 4
    NAV = 5
    MANUAL = 6


print "supervisor settings:\n"
print "use_gazebo = %s\n" % use_gazebo
print "rviz = %s\n" % rviz
print "mapping = %s\n" % mapping

class Supervisor:

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)
        # initialize variables
        self.x = 0
        self.y = 0
        self.theta = 0
        self.mode = Mode.IDLE
        self.last_mode_printed = None
        self.trans_listener = tf.TransformListener()
        # command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        # command vel (used for idling)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd_nav_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        #list of food items observed during the initial exploration stage.
        #format: [[msg.name1, msg.confidence1, object_pose1], [msg.name2, msg.confidence2, object_pose2], ....]
        self.food_list = []
        #faster, more efficient iteration of food_list above in a dictionary format.
        self.food_dict= dict()
        #list of food items with [(item, goal)]. Example: [(apple, (1,1)), (orange, (2,2))]
        #All items
        self.basket = []

        # subscribers
        # stop sign detector
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)
        # high-level navigation pose
        rospy.Subscriber('/nav_pose', Pose2D, self.nav_pose_callback)
        # if using gazebo, we have access to perfect state
        if use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        # if using rviz, we can subscribe to nav goal click
        if rviz:
            rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)

	#rospy.Subscriber('/map', nav_msgs/OccupancyGrid, self.map_callback)
        # puddle detector
        rospy.Subscriber('/coords_puddle', Pose2D, self.current_puddle_coords_callback )

        #food subscriber
        #take an order from the command line
        rospy.Subscriber('/delivery_request', String, self.food_order_callback)

        # food detector
        # TODO: come up with way to subscribe to all types of food - coco_labels.txt
        #       banana, apple, sandwich, orange, broccoli, carrot, hot dog, pizza, donut, cake
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

    def gazebo_callback(self, msg):
        pose = msg.pose[msg.name.index("turtlebot3_burger")]
        twist = msg.twist[msg.name.index("turtlebot3_burger")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]


    def current_puddle_coords_callback(self,msg):
        self.pud_x = msg.x
        self.pud_y = msg.y

        self.pud_dist = np.hypot(self.pud_x, self.pud_y)

        if self.pud_dist < 0.5:
            print("stopping for puddle!")
            self.mode = Mode.IDLE

    def food_detected_callback(self, msg):
        """ callback for detected food object from mobilenet """
        #name = msg.name
        #if name not in self.food_list:
        #    self.food_list.append(name)

        #confidence = msg.confidence
        #dist = msg.distance
        #thetaleft = msg.thetaleft
        #thetaright = msg.thetaright
        #theta = (thetaleft + thetaright) / 2.0
	position,angle = self.trans_listener.lookupTransform("/map", "/velodyne", rospy.Time(0)) #check for node names
	self.x = position[0]
        self.y = position[1]
        euler = tf.transformations.euler_from_quaternion(angle)
        self.theta = euler[2]
	theta = (msg.thetaleft + msg.thetaright)/2
	object_x = self.x + msg.distance * np.cos(theta)
	object_y = self.y + msg.distance * np.sin(theta)
	object_pose = np.array([object_x, object_y])
	object_list  = [msg.name, msg.confidence, object_pose]
	self.food_dict.update({msg.name: [msg.confidence, object_pose]})

	# If first item in list, add
	if len(self.food_list) == 0:
		self.food_list.append(object_list)

	#ELSE
	for i in range(len(self.food_list)):
		#If it has same name and if it is close to an existing an element
		if (self.food_list[i][0] == object_list[0]) and (np.linalg.norm(self.food_list[i][2] - object_list[2]) < 0.5):
			if (self.food_list[i][1] < object_list[1]):
				self.food_list.remove(self.food_list[i])
				self.food_list.append(object_list) #Higher Confidence estimate, update
				break
		else: #Not close to the existing list, hence append
			self.food_list.append(object_list)
	print(self.food_list,msg.distance)
	#current_list = [msg.name, msg.confidence, self.x, self.y, self.theta, msg.distance, msg.thetaleft,msg.thetaright]
        #self.food_list.append(current_list)
        ''' TODO: - Convert dist & theta into world coordinate
                     - may be this can be handled in EKF model - Similar to SLAM algo?
                  - initialize location estimate if not seen before
                  - update estimate based on confidence & measurements?
                  - store value for when we need to go retrieve it

        '''
    def food_order_callback(self, msg):
        message= str(msg)
        items = message.split(',')
        print(items)
        # REF:# object_list  = [msg.name, msg.confidence, object_pose]
        for item in items:
            if item not in self.food_dict:
                print("Your order of ", item, "is out of stock")
            else:
                _, goal = self.food_dict[item]
                print(item, 'at location: ', goal)#for debugging
                self.cmd_nav_publisher.publish(goal) #put this somewhere else



    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """
        origin_frame = "/map" if mapping else "/odom"
        print("rviz command received!")
        try:

            nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
            self.x_g = nav_pose_origin.pose.position.x
            self.y_g = nav_pose_origin.pose.position.y
            quaternion = (
                    nav_pose_origin.pose.orientation.x,
                    nav_pose_origin.pose.orientation.y,
                    nav_pose_origin.pose.orientation.z,
                    nav_pose_origin.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.theta_g = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        self.mode = Mode.NAV

    def nav_pose_callback(self, msg):
        self.x_g = msg.x
        self.y_g = msg.y
        self.theta_g = msg.theta
        self.mode = Mode.NAV

    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign
        dist = msg.distance

        # if close enough and in nav mode, stop
        if dist > 0 and dist < STOP_MIN_DIST and self.mode == Mode.NAV:
            self.init_stop_sign()


    ############ your code starts here ############
    # feel free to change the code here
    # you may or may not find these functions useful
    # there is no "one answer"


    def go_to_pose(self):
        """ sends the current desired pose to the pose controller """

        pose_g_msg = Pose2D()
        pose_g_msg.x = self.x_g
        pose_g_msg.y = self.y_g
        pose_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(pose_g_msg)

    def nav_to_pose(self):
        """ sends the current desired pose to the naviagtor """

        nav_g_msg = Pose2D()
        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(nav_g_msg)

    def stay_idle(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()

        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self,x,y,theta):
        """ checks if the robot is at a pose within some threshold """

        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS and abs(theta-self.theta)<THETA_EPS)

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """
        self.stop_sign_start = rospy.get_rostime()
        self.mode = Mode.STOP

    def has_stopped(self):
        """ checks if stop sign maneuver is over """

        return (self.mode == Mode.STOP and (rospy.get_rostime()-self.stop_sign_start)>rospy.Duration.from_sec(STOP_TIME))

    def init_crossing(self):
        """ initiates an intersection crossing maneuver """

        self.cross_start = rospy.get_rostime()
        self.mode = Mode.CROSS

    def has_crossed(self):
        """ checks if crossing maneuver is over """

        return (self.mode == Mode.CROSS and (rospy.get_rostime()-self.cross_start)>rospy.Duration.from_sec(CROSSING_TIME))

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """



        #################################################################################
        # Do not change this for hw2 -- this won't affect your FSM since you are using gazebo
        if not use_gazebo:
            try:
                origin_frame = "/map" if mapping else "/odom"
                (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
        #################################################################################

    # YOUR STATE MACHINE
    # Currently it will just go to the pose without stopping at the stop sign

        # logs the current mode
        if not(self.last_mode_printed == self.mode):
            rospy.loginfo("Current Mode: %s", self.mode)
            self.last_mode_printed = self.mode

        # checks wich mode it is in and acts accordingly
        if self.mode == Mode.IDLE:
            # send zero velocity
            self.stay_idle()

        elif self.mode == Mode.POSE:
            # moving towards a desired pose
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.go_to_pose()

        elif self.mode == Mode.STOP:
            # at a stop sign
            if self.has_stopped():
                self.init_crossing()
            else:
                self.mode = Mode.STOP

        elif self.mode == Mode.CROSS:
            # crossing an intersection
            if self.has_crossed():
                self.nav_to_pose()
            else:
                self.mode = Mode.CROSS

        elif self.mode == Mode.NAV:
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.nav_to_pose()

        else:
            raise Exception('This mode is not supported: %s'
                % str(self.mode))

    ############ your code ends here ############

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
