from styx_msgs.msg import TrafficLight
import numpy as np
import cv2
class TLClassifier(object):
    def __init__(self):
        #TODO load classifier

        #self.model = load_model('TRAINED MODEL')

        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        output = image.copy()
        result = TrafficLight.UNKNOWN
        light_color = cv2.cvtColor(image, cv2.COLOR_BRG2HSV)

        lower_light_color = np.array([10,60,50])
        upper_light_color = np.array([10,255,255])
        light_color1 = cv2.inRange(light_color, lower_light_color, upper_light_color)

        lower_light_color1 = np.array([170,70,50])
        upper_light_color1 = np.array([180,255,255])
        light_color2 = cv2.inRange(light_color, lower_light_color1, upper_light_color1)

        combined_lights = cv2.addWeighted(light_color1, 1.0, light_color2, 1.0, 0.0)
        blurred_lights = cv2.GaussianBlur(combined_lights,(15,15),0)
        circular_lights = cv2.HoughCircles(blurred_lights,cv2.HOUGH_GRADIENT,0.5,40, param1=60,param2=30,minRadius=5,maxRadius=200)

        if circular_lights is not None:
            result = TrafficLight.RED

        return result.output
