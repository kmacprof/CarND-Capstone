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
from math import sqrt
import numpy as np

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights helps you acquire an accurate ground truth data source for the traffic light
        classifier, providing the location and current color state of all traffic lights in the
        simulator. This state can be used to generate classified images or subbed into your solution to
        help you work on another single component of the node. This topic won't be available when
        testing your solution in real life so don't rely on it in the final submission.
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
        #print str(self.waypoints)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint

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

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        wp_i=-1
        min_wp = -1
        min_dist = 99999
        if self.waypoints:
            for wp in self.waypoints.waypoints:
                wp_i = wp_i+1
                #print str(wp.pose.pose.position.x)
                #print str(wp.pose.pose.position.y)
                x = wp.pose.pose.position.x - pose.position.x
                y = wp.pose.pose.position.y - pose.position.y
                dist = sqrt(x*x+y*y)
                if dist < min_dist:
                    min_wp = wp_i
                    min_dist = dist
            #print "min_wp:", min_wp
        return min_wp


    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image

        print str(trans)
        print str(rot)
        #print str(point_in_world)
        objectPoints = np.array([[point_in_world.x, point_in_world.y, point_in_world.z]])
        rvec = tf.transformations.quaternion_matrix(rot)[:3, :3]
        tvec = np.array(trans)
        cameraMatrix = np.array([[fx,  0, image_width/2],
                                       [ 0, fy, image_height/2],
                                       [ 0,  0,  1]])
        distCoeffs = None
        ret, _ = cv2.projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs)
        #print str(ret)

        x = ret[0][0][0]
        y = ret[0][0][1]

        print x
        print y

        return (x, y)

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

        self.camera_image.encoding = "rgb8"
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_positions = self.config['light_positions']
        light_wps = []

        #match light positions to waypoints so we can use the waypoint index nearest to the light
        #to determine if the light is ahead of the car_position

        for lp in light_positions:
            #put the light Position into Pose format
            lpPose = Pose()
            lpPose.position.x = lp[0] # x
            lpPose.position.y = lp[1] # y
            lpPose.position.z = 0
            lp_wp_i = self.get_closest_waypoint(lpPose)
            light_wps.append(lp_wp_i)



        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            if car_position >=0:
                print car_position

        #TODO find the closest visible traffic light (if one exists)
        lp_i = -1
        min_lp_i = -1
        min_lp_dist = 99999;
        if car_position >=0:
            #iterate through light positions, find closest light
            for lp in light_positions:
                #print str(lp)
                lp_i = lp_i + 1
                lp_x = lp[0]
                lp_y = lp[1]
                dx = self.waypoints.waypoints[car_position].pose.pose.position.x - lp_x
                dy = self.waypoints.waypoints[car_position].pose.pose.position.y - lp_y
                dist = sqrt(dx*dx+dy*dy)

                #if the waypoint index closest to the light is ahead of the car position
                # WARN: This currently cannot handle wrap around!
                if dist < min_lp_dist and light_wps[lp_i] > car_position:
                    min_lp_dist = dist
                    min_lp_i = lp_i

            #print min_lp_i;
            #print min_lp_dist;

            #determine if the light is in the camera frame.
            #You would think there is a more sophisticated way to do this, but I
            #don't think we have enough information on the camera Pose / FOV
            #we also don't have the traffic light heights
            if min_lp_dist < 175.0: #you can see a light generally at 175m distance
                #light = light_wps[lp_i] #waypoint position of the nearest light
                print min_lp_i
                light = self.lights[min_lp_i]
                light_wp = light_wps[min_lp_i]
                print str(light.pose)



        if light:
            state = self.get_light_state(light)
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()

    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
