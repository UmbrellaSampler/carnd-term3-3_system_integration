#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import copy

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
        self.next_index = -1
        self.stop_index = None
        self.stop_index_update = None
        self.stop_active = False

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
            self.next_index = -1
            return
        for i in range(len(self.map_waypoints)):
            current_distance = self.pose_distance(i)
            if current_distance < min_dist:
                min_dist = current_distance
                next_index = i
        # rospy.logwarn("index of next wp: %s", next_index)
        # rospy.logwarn("closest wp x: %s"), str(self.map_waypoints[next_index].)

        # rospy.logwarn("vel next wp: %s", self.get_waypoint_velocity(self.map_waypoints[next_index]))
        return next_index

    def is_stop_active(self):
        now = rospy.Time.now()
        if self.stop_index_update is not None:
            if now - self.stop_index_update < rospy.Duration(0.5):
                return True
        return False

    def update_next_waypoints(self):
        if not self.stop_active or self.stop_index < self.next_index or self.stop_index > self.next_index + 200:
            # rospy.logwarn("Updating waypoints without stop:")
            # rospy.logwarn("stop_active: %s", str(self.stop_active))
            # rospy.logwarn("stop_index: %s", str(self.stop_index))
            # rospy.logwarn("start next_index: %s", str(self.next_index))
            self.next_waypoints = []
            for i in range(0, LOOKAHEAD_WPS):

                # if next_index >= len(self.map_waypoints):
                #     next_index = 0
                # self.next_waypoints.append(copy.deepcopy(self.map_waypoints[self.next_index + i]))
                self.next_waypoints.append(self.map_waypoints[self.next_index + i])

        # rospy.logwarn("end next_index: %s", str(self.next_index + i))
        #
        else:
            #get vehicle vel.
            rospy.logwarn("Updating waypoints with stop:")
            current_wp_vel = self.get_waypoint_velocity(self.map_waypoints[self.next_index])
            # if self.next_waypoints is not None and len(self.next_waypoints) > 0:
            #     current_wp_vel = self.get_waypoint_velocity(self.next_waypoints[0])
            self.next_waypoints = []
            # target speed of current_index = current_vel
            # target vel = 0.0, target index =
            total_stop_distance = self.distance(self.map_waypoints, self.next_index, self.stop_index)
            number_of_waypoints_to_stop = self.stop_index - self.next_index
            # rospy.logwarn("current_wp_vel: %s", current_wp_vel)
            # rospy.logwarn("stop_waypoint: %s", self.stop_index)
            # rospy.logwarn("next_index: %s", self.next_index)
            # rospy.logwarn("Total stop distance: %s", total_stop_distance)
            # rospy.logwarn("number_of_waypoints_to_stop: %s", number_of_waypoints_to_stop)

            for i in range(0, LOOKAHEAD_WPS):
                # if next_index >= len(self.map_waypoints):
                #     next_index = 0
                self.next_waypoints.append(self.map_waypoints[self.next_index + i])
                waypoint_vel = 0.0
                if self.next_index + i <= self.stop_index:
                    remaining_distance_to_stop = self.distance(self.map_waypoints, self.next_index + i, self.stop_index)

                    if float(remaining_distance_to_stop > 1.0):
                        waypoint_vel = float(current_wp_vel) * float(remaining_distance_to_stop) / float(total_stop_distance)
                self.set_waypoint_velocity(self.next_waypoints, i, waypoint_vel)
                rospy.logwarn("i: %s, remaining_distance_to_stop: %s", i, remaining_distance_to_stop)
                rospy.logwarn("waypoint_vel: %s", self.get_waypoint_velocity(self.next_waypoints[i]))

        # rospy.logwarn("Number of final waypoints: %s", str(len(self.next_waypoints)))
        lane = Lane()
        lane.waypoints = self.next_waypoints

        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        start = rospy.Time.now()
        self.pose = msg.pose
        next_index = self.find_next_waypoint()
        is_stop_active = self.is_stop_active()
        rospy.logwarn("Calculated next_index: %s, prev index: %s", next_index, self.next_index)
        # rospy.logwarn("is_stop_active: %s", is_stop_active)

        if next_index is None or next_index < 0:
            rospy.logwarn("Not updating waypoints:")
            return

        if self.next_index > next_index or self.next_index < next_index: #or self.stop_active is None or self.stop_active is not is_stop_active:
            rospy.logwarn("updating waypoints: ")
            self.stop_active = is_stop_active
            self.next_index = next_index
            self.update_next_waypoints()

        else:
            rospy.logwarn("skip update waypoints: ")

        end = rospy.Time.now()
        duration = (end-start).to_sec()
        rospy.logwarn("Duration: %s", str(duration))

    def waypoints_cb(self, waypoints):
        rospy.logwarn("Setting waypoints...")
        self.map_waypoints = waypoints.waypoints

    def print_waypoints(self):
        rospy.logwarn("Next Waypoints:")
        for i in range(len(self.next_waypoints)):
            rospy.logwarn(self.next_waypoints[i].twist.twist.linear.x)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        # set waypoint speed smoothly
        stop_waypoint = msg.data
        now = rospy.Time.now()
        # if self.stop_index is None or self.stop_index is not stop_waypoint:
        #     rospy.logwarn("Setting stop in waypoints:")
        self.stop_index = stop_waypoint
        self.stop_index_update = now

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
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
