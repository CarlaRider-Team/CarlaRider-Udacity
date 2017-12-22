#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.wps1 = None

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1


        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        # TODO THIS SHOULDN'T BE HERE
        self.light_classifier.get_classification(cv_image)


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement

        closest_wp_dist = 99999999.9
        closest_wp_index = None

        # check needed because waypoints are only transmitted once at the beginning
        if self.waypoints is not None:
            wps = self.waypoints.waypoints
            self.wps1 = wps
        
        n = len(self.wps1)
        for i in range(n):
            wpsi = self.wps1[i].pose.pose.position
            wp_dx = wpsi.x - pose.position.x
            wp_dy = wpsi.y - pose.position.y
            wp_dz = wpsi.z - pose.position.z
            wp_dist = math.sqrt(wp_dx**2+wp_dy**2+wp_dz**2)
            if wp_dist < closest_wp_dist:
                closest_wp_index = i
                closest_wp_dist = wp_dist
        #print('closest_wp_index=',closest_wp_index,' closest_wp_dist=',closest_wp_dist)

        return closest_wp_index

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)


    def make_pose(self, x, y):
    #Helper function to generate Pose object given x and y
        light_pose = Pose()
        light_pose.position.x = x
        light_pose.position.y = y
        return light_pose


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        closest_light_wp = None

        
        if (self.waypoints is None):
            return -1, TrafficLight.UNKNOWN

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            # 1-Iterate through all the stop line positions
            ix = 0
            for curr_light_position in stop_line_positions:
                curr_light_pose = self.make_pose(curr_light_position[0], curr_light_position[1])
                curr_light_wp = self.get_closest_waypoint(curr_light_pose)
                # 2-Find the nearest forward stop line (light)
                if curr_light_wp >= car_position: #checking that light_wp is ahead of car_wp
                    if closest_light_wp is None:    #if this is the first light
                        closest_light_wp = curr_light_wp
                        closest_light_ix = ix
                        light_pose = curr_light_pose
                    elif curr_light_wp < closest_light_wp:
                        closest_light_wp = curr_light_wp
                        closest_light_ix = ix
                        light_pose = curr_light_pose
                ix = ix + 1
            # 3-create light
            light = self.lights[closest_light_ix]

        if light:
            state = light.state                    # remove this line after classifier is implemented
            # state = self.get_light_state(light)  # use    this line after classifier is implemented
            out_string = "process_traffic_lights: closest_light_wp="+str(closest_light_wp)+" closest_light_ix="+str(closest_light_ix)+" closest_light_st="+str(state)
            rospy.loginfo(out_string)
            return closest_light_wp, state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
