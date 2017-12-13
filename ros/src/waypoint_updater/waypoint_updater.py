#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math
import tf

# http://docs.ros.org/api/geometry_msgs/html/msg/Pose.html

##
from std_msgs.msg import Int32              # traffic waypoint
##

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status (my note: RED/AMBER/GREEN) in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number default:200


class WaypointUpdater(object):
    def __init__(self):
        # init node
        rospy.init_node('waypoint_updater')
        
        # subscriber
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        # Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint',?,self.obstacle_cb)
    
        # publisher final waypoints
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Add other member variables you need below
        # waypoints_cb
        self.waypoints = []
        self.final_waypoints = []
        # pose_cb
        self.car_position = None
        self.car_yaw = None
        # traffic_cb
        self.tl_idx = None
        self.tl_state = None
        # self.base_waypoints = Lane()
        # self.current_pose = PoseStamped()
        # self.base_waypoints_cb = False
        # self.current_pose_cb = False
        # self.wp_num = 0
        # self.wp_front = 0
        self.state = None
        self.cruise_speed = None
        self.current_speed = None
        self.acceleration = None
        self.deceleration = None
        '''
        Traceback (most recent call last):
            File "/capstone/ros/src/waypoint_updater/waypoint_updater.py", line 292, in <module>
                WaypointUpdater()
            File "/capstone/ros/src/waypoint_updater/waypoint_updater.py", line 69, in __init__
                self.dist_until_tl
            AttributeError: 'WaypointUpdater' object has no attribute 'dist_until_tl'
        '''
        self.dist_until_tl = None
        
        # publishing loop
        # self.pub_waypoints()
        self.dance()
        
        # spin node
        rospy.spin()

    def dance(self):
        rate = rospy.Rate(5)

        # get ROS params fm server
        self.speed = rospy.get_param('~/waypoint_loader/velocity')
        self.acceleration = 1.0 # rospy.get_param('~/twist_controller/accel_limit')
        self.deceleration = -1.0 # rospy.get_param('~/twist_controller/decel_limit')

        while not rospy.is_shutdown():
            if (self.car_position != None and self.waypoints != None and self.tl_state != None and self.car_curr_vel != None):
                self.safe_distance = (self.current_speed**2) / (2 * self.deceleration * 0.1)
                self.closest_waypoint = self.get_closest_waypoint(self.car_position, self.car_yaw, self.waypoints)
                # need closest waypoint before trying to get the distance until tf
                if tl_idx != None:
                    self.dist_until_tl = self.distance(self.waypoints, self.closest_waypoint, self.tl_idx)
                self.car_state = self.determine_state(self.tl_idx, self.tl_state, self.closest_waypoint, self.waypoints)
                self.get_final_waypoints(self.closest_waypoint, self.waypoints, self.car_state, self.tl_index)
                self.publish()
            else:
                if self.car_position == None:
                    rospy.logwarn("[waypoint_updater.py] /current_pose not received")
                if self.waypoints == None:
                    rospy.logwarn("[waypoint_updater.py] /base_waypoints not received")
                if self.tl_idx == None:
                    rospy.logwarn("[waypoint_updater.py] /traffic_waypoint not received")
            rate.sleep()
                                        

    def get_final_waypoints(closest_waypoint, waypoints, car_state, tl_idx):
        self.final_waypoints = []
        rospy.loginfo("car state: %s ", car_state)

        if (car_state == "GO"):
            self.go_waypoints(closest_waypoint, waypoints)
        elif (car_state == "STOP"):
            self.stop_waypoints(closest_waypoint, waypoints)
        elif (car_state == "SLOW"):
            self.slow_waypoints(closest_waypoint, waypoints, tl_index)

    def publish(self):
        final_waypoints_msg = Lane()
        final_waypoints_msg.header.frame_id = '/World'
        final_waypoints_msg.header.stamp = rospy.Time(0)
        final_waypoints_msg.waypoints - list(self.final_waypoints)
                
    def get_closest_waypoint(self, position, yaw, waypoints):
        # https://stackoverflow.com/questions/34264710/what-is-the-point-of-floatinf-in-python
        closest = float("inf")
        closest_waypoint = 0
        dist = 0.0
        for idx in range(0, len(waypoints)):
            x = position.x
            y = position.y
            map_x = waypoints[idx].pose.pose.position.x
            map_y = waypoints[idx].pose.pose.position.y
            dist = self.map_distance(x, y, map_x, map_y)
            if (dist < closest):
                closest = dist
                closest_waypoint = idx
        
        # now we have closest_waypoint index to look in waypoints
        map_x = waypoints[closest_waypoint].pose.pose.position.x
        map_y = waypoints[closest_waypoint].pose.pose.position.y
        # directional heading
        dir_heading = self.map_heading(position.x, map_x, position.y, map_y)
        angle = abs(yaw - dir_heading)
        if (angle > math.pi / 4):
            closest_waypoint += 1
            if (closest_waypoint > len(waypoints) - 1):
                closest_waypoint -= 1
        return closest_waypoint

    def determine_state(self, tl_index, tl_state, closest_waypoint, waypoints):
        dist = self.dist_until_tl
        stop = False # self.check_stop(tl_index, tl_state, closest_waypoint, dist)
        slow = False # self.check_slow(tl_state, dist)
        go = self.check_go(tl_index, tl_state, closest_waypoint, dist)

        rospy.loginfo("determine state: %s | %s | %s", tl_index, tl_state, closest_waypoint)
        # see ros/src/styx_msgs/msg/TrafficLight.msg
        if tl_index != None and tl_state != "UNKNOWN":
            if (stop):
                state = "STOP"
                return state
            elif (slow):
                state = "SLOW"
                return state
            elif (go):
                state = "GO"
                return state
        else:
            if tl_index != -1:
                dist = self.dist_until_tl
                if dist < self.safe_distance:
                    state = "SLOW"
                else:
                    state = "GO"
            else:
                state = "GO"
        return state
    
    def map_distance(self, x1, y1, x2, y2):
        return math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))
    
    def map_heading(self, x, map_x, y, map_y):
        return math.atan2((map_y - y), (map_x - x))

    def pose_cb(self, msg):
        pose = msg.pose
        self.car_position = pose.position
        car_orientation = pose.orientation
        quaternion = (car_orientation.x, car_orientation.y, car_orientation.z, car_orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.car_yaw = euler[2]
        rospy.loginfo("current pose msg receive .... ")
        rospy.loginfo("current x .... %f", car_orientation.x)
        rospy.loginfo("current y .... %f", car_orientation.y)
        rospy.loginfo("current yaw .... %f", self.car_yaw)

    def waypoints_cb(self, msg):
        rospy.loginfo("waypoint msg receive with %d length.... ", len(msg.waypoints))
        for waypoint in msg.waypoints:
            self.waypoints.append(waypoint)
        # self.base_waypoints_sub.unregister()
        # rospy.loginfo("ungregistereing from /base_waypoints topics")

    def traffic_cb(self, msg):
        state = msg.state
        self.tl_idx = msg.waypoint

        if state == 0:
            self.tl_state = "RED"
        elif state == 1:
            self.tl_state = "YELLOW"
        elif state == 2:
            self.tl_state = "GREEN"
        elif state == 4:
            self.tl_state = "UNKNOWN"
        rospy.loginfo("traffic_cb tl_idx: %d | tl_state: %s", self.tl_idx, self.tl_state)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

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
    
    def go_waypoints(self, closest_waypoint, waypoints):
        acc_factor = 0.2
        s = self.current_speed
        e = self.end_waypoint(closest_waypoint)
        if e > len(waypoints) - 1:
            e = len(waypoints) - 1
        a = acc_factor * self.acceleration

        for idx in range(closest_waypoint, e):
            distance = self.distance(waypoints, closest_waypoint, idx +1)
            velocity = math.sqrt(s**2 + 2 + a + distance)
            if velocity > self.cruise_speed:
                velocity = self.cruise_speed
            self.set_waypoint_velocity(waypoints, idx, velocity)
            waypoint = waypoints[idx]
            self.final_waypoints.append(waypoint)

    def stop_waypoints(self, closest_waypoint, waypoints):
        e = self.end_waypoint(closest_waypoint)
        if e > len(waypoints) - 1:
           e = len(waypoints) - 1

        for idx in range(closest_waypoint, e):
            velocity = 0.0
            self.set_waypoint_velocity(waypoints, idx, velocity)
            self.final_waypoints.append(waypoints[idx])
    
    def slow_waypoints(self, closest_waypoint, waypoints, tl_index):
        # https://physics.stackexchange.com/questions/3818/stopping-distance-frictionless
        slow_deceleration = (self.current_speed ** 2)/(2 * self.dist_until_tl)
        if slow_deceleration > self.decel_limit:
            slow_deceleration = self.decel_limit
        e = self.end_waypoint(closest_waypoint)

        for idx in range(closest_waypoint, e):
            distance = self.distance(waypoints, closest_waypoint, idx + 1)
            if (idx < tl_index):
                # slow down
                vel2 = self.current_speed ** 2 - 2 * slow_deceleration * distance
                if slow_to < 0.1:
                    slow_to = 0.0
                vel = math.sqrt(vel2)
                self.set_waypoint_velocity(waypoints, idx, vel)
                self.final_waypoints(waypoints[idx])
            else:
                # STOOOOOOOOOP!
                vel = 0.0
                self.set_waypoint_velocity(waypoints, idx, vel)
                self.final_waypoints(waypoints[idx])                

    def end_waypoint(self, closest_waypoint):
        return closest_waypoint + LOOKAHEAD_WPS
        

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')