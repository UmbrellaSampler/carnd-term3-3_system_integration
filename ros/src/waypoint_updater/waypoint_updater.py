#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.next_waypoints = None
        self.map_waypoints = None
        self.pose = None

        rospy.spin()

    def pose_distance(self, index):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        return dl(self.map_waypoints[index].pose.pose.position, self.pose.position)

    # find the next waypoint in front of the car
    def find_next_waypoint(self):
        min_dist = 10000000000.0
        next_index = -1
        if self.map_waypoints is None:
            rospy.logwarn("No Waypoints given...")
            return None
        for i in range(len(self.map_waypoints)):
            current_distance = self.pose_distance(i)
            if current_distance < min_dist:
                min_dist = current_distance
                next_index = i
        rospy.loginfo("index of next wp: %s", next_index)
        return next_index

#
    def pose_cb(self, msg):
        self.pose = msg.pose
        next_index = self.find_next_waypoint()
        if next_index is None or next_index is -1:
            return
        self.next_waypoints = []
        if next_index >= 0:
            for i in range(0, LOOKAHEAD_WPS):
                if next_index >= len(self.map_waypoints):
                    next_index = 0
                self.next_waypoints.append(self.map_waypoints[next_index])
                next_index += 1
        lane = Lane()
        lane.waypoints = self.next_waypoints
        rospy.logdebug("Number of final waypoints: %s", str(len(self.next_waypoints)))
        self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, waypoints):
        rospy.logwarn("Setting waypoints...")
        self.map_waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        # set waypoint speed smoothly
        stop_waypoint = msg.data
        rospy.logwarn("Stop Waypoint: %s ", stop_waypoint)
        # if no tl set nextx waypoints to target speed
        # current vel = , current index = , target speed of current_index = current_vel
        # target vel = 0.0, target index =
        # total_stop_distance = distance (current_index, target_index)
        # if stopline too close dont do anything
        # for i in range indices
        # current_wp_distance = .
        # current_target_vel = current_vel * current_wp_distance / total_remaining_wp_distance
        # if decel too big adapt
        # total_remaining_wp_distance =
        # current_vel =
        # current-index =

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    # distance between two waypoints given their indices
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
