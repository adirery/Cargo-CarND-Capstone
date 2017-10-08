#!/usr/bin/env python

import rospy
<<<<<<< HEAD
from geometry_msgs.msg 	import PoseStamped, TwistStamped
from styx_msgs.msg 	import Lane, Waypoint, TrafficLight, TrafficLightArray
from std_msgs.msg	import Int32, Float32, Bool
=======
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32, Header	
>>>>>>> 3fb7ab526de76a8fab9aa1a3d92bf7ca03d1764e
import tf
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

<<<<<<< HEAD
LOOKAHEAD_WPS = 100	# Number of waypoints we will publish. You can change this number
STOP_BUFFER = 5.0
MAX_DECEL = 4.0
=======
LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 5.0
MAX_ACCEL = 1.0
SAFE_DIST = 30.0
>>>>>>> 3fb7ab526de76a8fab9aa1a3d92bf7ca03d1764e

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

<<<<<<< HEAD
        rospy.Subscriber('/current_pose', 	PoseStamped, 	self.pose_cb, 			queue_size = 1)
        rospy.Subscriber('/base_waypoints', 	Lane, 		self.waypoints_cb,		queue_size = 1)
	rospy.Subscriber('/traffic_waypoint', 	Int32, 		self.traffic_cb,		queue_size = 1)
	rospy.Subscriber('/current_velocity', 	TwistStamped, 	self.current_velocity_cb, 	queue_size = 1)
	
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints',  Lane, queue_size=1)
=======
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
	self.traffic_waypoint = None
	self.obstacle_waypoint = None
	self.current_velocity = None
	
	rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)
	rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
>>>>>>> 3fb7ab526de76a8fab9aa1a3d92bf7ca03d1764e

        # TODO: Add other member variables you need below
	self.current_pose = None	# current car position
        self.base_waypoints = []	# list of waypoints
	self.base_wp_orig_v = []	# list of velocities of waypoints
	self.next_waypoint = None	# Next waypoint
	self.traffic_waypoint = -1	# traffic waypoint index
	self.current_velocity = 0.0	# current velocity
	self.breaking = False	
	self.decel = 1.0

	rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

    def loop(self):
<<<<<<< HEAD
        if(self.current_pose and self.base_waypoints):
            lane = 	Lane()
            pose = 	self.current_pose
            waypoints = self.base_waypoints.waypoints
            traffic_wp =self.traffic_waypoint
            next_wp = 	self.get_next_waypoint(pose, waypoints)  
            
            # get distance to traffic light and minimum stopping distance
            tl_dist = self.dist(pose.pose.position, waypoints[traffic_wp].pose.pose.position)
            min_stopping_dist = self.current_velocity**2 / (2.0 * MAX_DECEL) + STOP_BUFFER
            
            # break if red light and enough room to stop
            if traffic_wp == -1:
                self.breaking = False
                lane.waypoints = self.get_final_waypoints(waypoints, next_wp, next_wp + LOOKAHEAD_WPS)
            elif not self.breaking and tl_dist < min_stopping_dist:
                lane.waypoints = self.get_final_waypoints(waypoints, next_wp, next_wp + LOOKAHEAD_WPS) 
            else:
                self.breaking = True
                lane.waypoints = self.get_final_waypoints(waypoints, next_wp, traffic_wp)
	    
            self.final_waypoints_pub.publish(lane)
=======
	# first version	
	if(self.current_pose and self.base_waypoints):
	    # get closest waypoint index
            closest_index = self.get_closest_waypoint(self.current_pose.pose)
            # get the first waypoint index currently ahead of the car 
            next_index = self.get_next_waypoint(self.current_pose.pose, closest_index)  
            # lane title 
	    lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time(0)         
            # safety stop distance 
	    min_dist_stop = self.current_velocity**2 / (2.0 * MAX_DECEL) + SAFE_DIST
	    # generate final_waypoints and publish
	    if self.traffic_waypoint and self.traffic_waypoint != -1:
		tl_dist = self.distance(self.base_waypoints.waypoints, closest_index, self.traffic_waypoint)
	        #tl_dist = self.dist(self.current_pose.pose.position, self.base_waypoints.waypoints[self.traffic_waypoint].pose.pose.position)
	        rospy.loginfo("self.current_velocity {}, self.traffic_waypoint {}, tl_dist {}, min_dist_stop {}".format(
			       self.current_velocity,
                               self.traffic_waypoint,
                               tl_dist,
                               min_dist_stop))

                if tl_dist < min_dist_stop:
	            # set final_waypoints to traffic light and set velocity
		    rospy.loginfo("braking")
	            final_waypoints = []
                    for i in range(next_index, self.traffic_waypoint):
            		index = i % len(self.base_waypoints.waypoints)
            		wp = Waypoint()
            		wp.pose.pose.position.x  = self.base_waypoints.waypoints[index].pose.pose.position.x
            		wp.pose.pose.position.y  = self.base_waypoints.waypoints[index].pose.pose.position.y
            		wp.pose.pose.position.z  = self.base_waypoints.waypoints[index].pose.pose.position.z
            		wp.pose.pose.orientation = self.base_waypoints.waypoints[index].pose.pose.orientation
		        final_waypoints.append(wp)
                    # set speed to stop before traffic light
		    lane.waypoints = self.decelerate(final_waypoints)
		else:
		    rospy.loginfo("no braking since too far to traffic light")
		    lane.waypoints = self.base_waypoints.waypoints[next_index : (next_index + LOOKAHEAD_WPS)]

            else:
	        rospy.loginfo("no detected traffic light before")
		lane.waypoints = self.base_waypoints.waypoints[next_index : (next_index + LOOKAHEAD_WPS)]
		
  	    self.final_waypoints_pub.publish(lane)

    def decelerate(self, waypoints):
        last = waypoints[-1]
        last.twist.twist.linear.x = 0.0
        for wp in waypoints[:-1][::-1]:
            dist = self.dist(wp.pose.pose.position, last.pose.pose.position)
            dist = max(0.0, dist-SAFE_DIST)
            vel  = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.0:
                vel = 0.0
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
	    rospy.loginfo("wp.twist.twist.linear.x {}".format(wp.twist.twist.linear.x))
        return waypoints
>>>>>>> 3fb7ab526de76a8fab9aa1a3d92bf7ca03d1764e

    def dist(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    # Carry over codes from CarND-Path-Planning-Project
    def get_closest_waypoint(self, pose, waypoints):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
	closest_index = 0
     	closest_dist = float('inf')
	for i in range(len(waypoints)):
	    dist = self.dist(pose.pose.position, waypoints[i].pose.pose.position)
	    if dist < closest_dist:
	        closest_dist = dist
	        closest_index = i

        return closest_index

    def get_next_waypoint(self, pose, index):
	"""Identifies the first waypoint that is currently ahead of the car
        Args:
            index(int): index of the closest waypoint in self.waypoints

        Returns:
            int: index of the first waypoint currently ahead of the car

	"""
	next_index = index 
        p1 = pose.position
	p2 = self.base_waypoints.waypoints[index].pose.pose.position
	heading = math.atan2( (p2.y-p1.y),(p2.x-p1.x) );
	quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
	yaw = euler[2]
	angle = abs(yaw-heading);

        if angle > math.pi/4.0:
            next_index += 1
	
	return next_index

<<<<<<< HEAD
    def get_final_waypoints(self, waypoints, start_wp, end_wp):
        final_waypoints = []
        for i in range(start_wp, end_wp):
            index = i % len(waypoints)
            wp = Waypoint()
            wp.pose.pose.position.x  = waypoints[index].pose.pose.position.x
            wp.pose.pose.position.y  = waypoints[index].pose.pose.position.x
            wp.pose.pose.position.z  = waypoints[index].pose.pose.position.x
            wp.pose.pose.orientation = waypoints[index].pose.pose.orientation

            if self.breaking:
                dist = self.distance(wp.pose.pose.position, waypoints[end_wp].pose.pose.position)
                if dist > STOP_BUFFER and self.current_velocity < 1.0:
                     wp.twist.twist.linear.x = 2.0
                elif dist < STOP_BUFFER and self.current_velocity < 1.0:
                     wp.twist.twist.linear.x = 0.0
                else:
                     wp.twist.twist.linear.x = min(self.current_velocity, waypoints[index].twist.twist.linear.x)
            else:
                wp.twist.twist.linear.x = waypoints[index].twist.twist.linear.x
            final_waypoints.append(wp)

        if self.breaking:
            # Find traffic light waypoint
            tl_wp = len(final_waypoints)

            # If breaking, set all waypoints past traffic_wp within Lookahead distance to 0.0
            for i in range(end_wp, start_wp + LOOKAHEAD_WPS):
                index = i % len(waypoints)
                wp = Waypoint()
                wp.pose.pose.position.x  = waypoints[index].pose.pose.position.x
                wp.pose.pose.position.y  = waypoints[index].pose.pose.position.x
                wp.pose.pose.position.z  = waypoints[index].pose.pose.position.x
                wp.pose.pose.orientation = waypoints[index].pose.pose.orientation               
                wp.twist.twist.linear.x = 0.0
                final_waypoints.append(wp)
            final_waypoints = self.decelerate(final_waypoints, tl_wp)
        return final_waypoints    
                

    def decelerate(self, waypoints, tl_wp):
        last = waypoints[tl_wp]
        last.twist.twist.linear.x = 0.0
        for wp in waypoints[:tl_wp][::-1]:
            dist = self.dist(wp.pose.pose.position, last.pose.pose.position)
            dist = max(0.0, dist - STOP_BUFFER)
            vel = math.sqrt(2 * self.decel * dist) 
            if vel < 1.0:
                vel = 0.0
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints

    def publish(self, waypoints):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)

=======
>>>>>>> 3fb7ab526de76a8fab9aa1a3d92bf7ca03d1764e
    def pose_cb(self, msg):
        # TODO: Implement
	self.current_pose = msg.pose
  
    def waypoints_cb(self, msg):
        # TODO: Implement
        self.base_waypoints = msg

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x

    def traffic_cb(self, msg):
<<<<<<< HEAD
        self.traffic_waypoint = msg.data
        pass

    def obstacle_cb(self, msg):
        self.obstacle_waypoint = msg.data
	pass
=======
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_waypoint = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        self.obstacle_waypoint = msg.data

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x
>>>>>>> 3fb7ab526de76a8fab9aa1a3d92bf7ca03d1764e

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
