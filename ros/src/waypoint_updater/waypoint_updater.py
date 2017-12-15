#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math
import copy
from std_msgs.msg import Int32

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.
Once you have created dbw_node, you will update this node to use the status of traffic lights too.
Please note that our simulator also provides the exact location of traffic lights and their
current status (my note: RED/AMBER/GREEN) in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # subscribers
        self.sub_current_pose = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.sub_base_waypoints = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.sub_traffic_waypoint = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # self.sub_obstacle_waypoint = rospy.Subscriber('/obstacle_waypoint',?,self.obstacle_cb)

        # publishers
        self.pub_final_waypoints = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.LOOKAHEAD_WPS = 200

        self.waypoints_base = []
        self.dist_max = 0.0  # max dist from the closest waypoint

        rospy.spin()

    def publish_waypoints(self, waypoints):
        lane = Lane()
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.pub_final_waypoints.publish(lane)

    def get_nearest_front(self, current_pose, waypoints):
        dist_square_min = 99999
        index_min = 0
        for index in range(len(waypoints)):
            dist_square_cur = (waypoints[index].pose.pose.position.x - current_pose.position.x) ** 2 \
                              + (waypoints[index].pose.pose.position.y - current_pose.position.y) ** 2
            if dist_square_cur < dist_square_min:
                dist_square_min = dist_square_cur
                index_min = index
        return index_min

    def pose_cb(self, msg):
        current_pose = msg.pose

        closest_waypoint_index = self.get_nearest_front(current_pose, self.waypoints_base)

        waypoint_x = self.waypoints_base[closest_waypoint_index].pose.pose.position.x
        waypoint_y = self.waypoints_base[closest_waypoint_index].pose.pose.position.y
        current_pose_x = current_pose.position.x
        current_pose_y = current_pose.position.y

        dist = self.map_distance(waypoint_x, waypoint_y, current_pose_x, current_pose_y)

        if dist > self.dist_max:
            self.dist_max = dist

        index_end = min(closest_waypoint_index + self.LOOKAHEAD_WPS - 1, len(self.waypoints_base))

        self.publish_waypoints(self.waypoints_base[closest_waypoint_index: index_end])

    def waypoints_cb(self, msg):
        self.waypoints_base = copy.deepcopy(msg.waypoints)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def map_distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))

    def map_heading(self, x, map_x, y, map_y):
        return math.atan2((map_y - y), (map_x - x))

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt(
            (a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
