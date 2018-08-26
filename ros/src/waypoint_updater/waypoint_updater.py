#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import numpy as np
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

'''

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
MAX_DECEL = 1.0
MPS2MPH = 2.236936
SAFETY_FACTOR = 0.90

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')



        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)


        # Attribute initialization
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1
        self.speed_limit = rospy.get_param('/waypoint_loader/velocity') / 3.6
        rospy.loginfo("Speed limit set to %.2f MPH", self.speed_limit*MPS2MPH)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)


        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints and self.waypoint_tree:
                self.publish_waypoints()
            rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        """
        Callback for /base_waypoints message. 
        Updates the list of points and the KDTree
        """
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[w.pose.pose.position.x, w.pose.pose.position.y] for w in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_closest_waypoint_id(self):
        """
        Returns the index of the closest waypoint ahead of the vehicle
        """
        if self.waypoint_tree:
            pt = [self.pose.pose.position.x, self.pose.pose.position.y]
            closest_id = self.waypoint_tree.query(pt, 1)[1]

            closest_pt = np.array(self.waypoints_2d[closest_id])
            prev_pt = np.array(self.waypoints_2d[closest_id - 1])
            pt = np.array(pt)
            value = np.dot(closest_pt - prev_pt, pt - closest_pt)
            if value > 0:
                closest_id = (closest_id + 1) % len(self.waypoints_2d)

            return closest_id
        return 0

    def publish_waypoints(self):
        if not self.base_waypoints:
            return
        lane = self.generate_lane()
        self.final_waypoints_pub.publish(lane)

    def generate_lane(self):
        lane = Lane()

        closest_idx = self.get_closest_waypoint_id()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(waypoints, closest_idx)

        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        result = []
        for i, wp in enumerate(waypoints):
            new_point = Waypoint()
            new_point.pose = wp.pose

            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)  # Two waypints back from line so the front of
            # the car stops at the line
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * SAFETY_FACTOR * dist)
            if vel < 1.0:
                vel = 0.0

            new_point.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            result.append(new_point)

        return result


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
