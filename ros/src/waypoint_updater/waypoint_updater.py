#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
import math
from std_msgs.msg import Int32
import tf

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.
Once you have created dbw_node, you will update this node to use the status of traffic lights too.
Please note that our simulator also provides the exact location of traffic lights and their
current status (my note: RED/AMBER/GREEN) in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

LOOKAHEAD_WPS = 44  # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
  def __init__(self):
    rospy.init_node('waypoint_updater')

    self.wp_closest_last = None
    self.wps_count = 0
    self.wps_final = []

    # subscribers
    rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb)
    rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
    rospy.Subscriber('/current_velocity', TwistStamped,
                     self.current_velocity_cb)
    rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
    self.current_pose = None
    self.base_waypoints = None
    self.current_velocity = None
    self.traffic_waypoint = None

    # publishers
    self.pub_final_waypoints = rospy.Publisher('final_waypoints', Lane,
                                               queue_size=1)

    self.spd_chg_loop()

  def publish_waypoints(self):
    lane = Lane()
    lane.header = self.base_waypoints.header
    lane.waypoints = self.wps_final
    self.pub_final_waypoints.publish(lane)

  def current_velocity_cb(self, current_velocity):
    self.current_velocity = current_velocity

  def current_pose_cb(self, msg):
    self.current_pose = msg

  def waypoints_cb(self, waypoints):
    self.base_waypoints = waypoints
    self.wps_count = len(waypoints.waypoints)

  def traffic_cb(self, msg):
    # TODO: Callback for /traffic_waypoint message. Implement
    self.traffic_waypoint = msg

  def obstacle_cb(self, msg):
    # TODO: Callback for /obstacle_waypoint message. We will implement it later
    pass

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

  def find_wp_next_front(self):
    # returns the closest waypoint# to the car's current position
    wp_closest = 0
    dist_min = 9999999.9
    # list does not exist
    if self.wp_closest_last is None:
      wps_list = list(range(0, self.wps_count))
    # list exists
    else:
      wp_min = self.wp_closest_last
      wp_max = (self.wp_closest_last + int(LOOKAHEAD_WPS / 2)) % self.wps_count
      # populate the list
      if wp_min < wp_max:  # wp min/max do not go over wps_count => min2max
        wps_list = list(range(wp_min, wp_max))
      else:  # wp min/max in range of wps_count => min2wps_count + zero2max
        wps_list = list(range(wp_min, self.wps_count)).extend(
          list(range(0, wp_max)))
    # find min distance
    for i in wps_list:
      car = (self.current_pose.pose.position.x,
             self.current_pose.pose.position.y,
             self.current_pose.pose.position.z)
      wpi = (self.base_waypoints.waypoints[i].pose.pose.position.x,
             self.base_waypoints.waypoints[i].pose.pose.position.y,
             self.base_waypoints.waypoints[i].pose.pose.position.z)
      dist_car2wpi = math.sqrt(
        (car[0] - wpi[0]) ** 2 + (car[1] - wpi[1]) ** 2 + (
        car[2] - wpi[2]) ** 2)
      if (dist_car2wpi < dist_min):
        wp_closest = i  # found closest wp. But need to check if behind/front
        dist_min = dist_car2wpi
    # check if closest wp is in front of car
    x = self.current_pose.pose.orientation.x
    y = self.current_pose.pose.orientation.y
    z = self.current_pose.pose.orientation.z
    w = self.current_pose.pose.orientation.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([x, y, z, w])
    angle_wp = math.atan2(self.base_waypoints.waypoints[
                            wp_closest].pose.pose.position.y - self.current_pose.pose.position.y,
                          self.base_waypoints.waypoints[
                            wp_closest].pose.pose.position.x - self.current_pose.pose.position.x)
    if 0.5 * 3.1416 < ((angle_wp - yaw) % 2.0 * 3.1416) < 1.5 * 3.1416:
      wp_closest = (wp_closest + 1) % self.wps_count
    # keep the last value
    self.wp_closest_last = wp_closest
    return wp_closest

  def spd_chg_loop(self):
    # assign the waypoint speeds[m/s] depending on the situation
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
      if (self.current_pose and self.base_waypoints) is not None:
        wp_closest2car = self.find_wp_next_front()
        # create list of wps to change speeds
        self.wps_final = []
        for i in range(wp_closest2car, wp_closest2car + LOOKAHEAD_WPS + 1):
          if self.wps_count <= i:
            i = i - self.wps_count
          self.wps_final.append(self.base_waypoints.waypoints[i])
        # 1-car is before a published TL => set wps speeds according to safety
        if self.traffic_waypoint is not None and self.current_velocity is not None and self.traffic_waypoint != -1 and wp_closest2car < self.traffic_waypoint.data:
          nwps_tl2car = self.traffic_waypoint.data - wp_closest2car
          current_velocity = self.current_velocity.twist.linear.x
          dist_car2tl = self.distance(self.base_waypoints.waypoints,
                                      wp_closest2car,
                                      self.traffic_waypoint.data)
          spd_div_dist = abs(current_velocity / dist_car2tl)
          # A - far from TL and slow speed => go slow
          if (15.0 < dist_car2tl and current_velocity < 5.0):
            for wp in self.wps_final:
              wp.twist.twist.linear.x = 3.0
          # B - close to TL and decel needed low => safe to stop
          elif (dist_car2tl <= 6.0 and spd_div_dist < 2.5):
            for wp in self.wps_final:
              wp.twist.twist.linear.x = -1.0
          # C - low spd/dist decel needed => safe to stop
          elif spd_div_dist < 2.5:
            if nwps_tl2car < len(self.wps_final):  # TL is before end of wps
              ix_end = nwps_tl2car
              for wp in self.wps_final[ix_end:]:
                wp.twist.twist.linear.x = -1.0
            else:  # TL is beyond end of wps (truncate)
              ix_end = len(self.wps_final)
            # set spd using linear function w.r.t distance
            for k, wp in enumerate(self.wps_final[:ix_end]):
              wp.twist.twist.linear.x = spd_div_dist * self.distance(
                self.base_waypoints.waypoints, wp_closest2car + k,
                self.traffic_waypoint.data)
              # 2-car is past any published TL => set ALL wps speeds slightly below speed limit
        else:
          for k, wp in enumerate(self.wps_final):
            spd_lim = rospy.get_param(
              '/waypoint_loader/velocity') * 1000.0 / 3600.0  # [m/s]
            self.set_waypoint_velocity(self.wps_final, k, spd_lim - 0.5)
        self.publish_waypoints()
      rate.sleep()


if __name__ == '__main__':
  try:
    WaypointUpdater()
  except rospy.ROSInterruptException:
    rospy.logerr('Could not start waypoint updater node.')