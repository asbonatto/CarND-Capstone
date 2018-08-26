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
from scipy.spatial import KDTree


# Detector Stuff
import os
from cfg import *
from mobiledet.utils import utils
from mobiledet.models.keras_yolo import yolo_eval, decode_yolo_output, create_model
from keras import backend as K

import time
import tensorflow
from keras.models import load_model

STATE_COUNT_THRESHOLD = 3
WAYPOINT_LOOKAHEAD = 100
SAVE_IMAGE = False
MOCK_TRAFFIC_LIGHTS = False


class TLDetector(object):

    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        self.waypoints_2d = None
        self.waypoint_tree = None

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

        self.is_site = self.config['is_site']

        #TODO Remove hack to force site mode or ground_truth for testing
        # self.is_site = True
        self.ground_truth = False


        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.RED
        self.last_state = TrafficLight.RED
        self.last_wp = -1
        self.state_count = 0

        self.waypoints_2d = None
        self.waypoint_tree = None
        self.vgg_model = None
        self.graph = None
        self.sess = None
        self.initialized = False

        if self.is_site:
            # Detector Stuff
            self.model_image_size = None
            model_path = os.path.expanduser('./weights/parking_lot.h5')
            anchors_path = os.path.expanduser('./model_data/lisa_anchors.txt')
            classes_path = os.path.expanduser('./model_data/lisa_classes.txt')
            
            self.class_names  = utils.get_classes(classes_path)
 
            anchors = utils.get_anchors(anchors_path)
            if SHALLOW_DETECTOR:
                anchors = anchors * 2
                        
            self.yolo_model, _ = create_model(anchors, self.class_names, load_pretrained=True, 
            feature_extractor=FEATURE_EXTRACTOR, pretrained_path=model_path, freeze_body=True)

            # Check if model is fully convolutional, assuming channel last order.
            self.model_image_size = self.yolo_model.layers[0].input_shape[1:3]

            self.sess = K.get_session()  

            # Generate output tensor targets for filtered bounding boxes.
            self.yolo_outputs = decode_yolo_output(self.yolo_model.output, anchors, len(self.class_names))

            self.input_image_shape = K.placeholder(shape=(2, ))
            self.boxes, self.scores, self.classes = yolo_eval(
                self.yolo_outputs,
                self.input_image_shape,
                score_threshold=.6,
                iou_threshold=.6)

            self.graph = tensorflow.get_default_graph()
        else:
            try:
                model_path = os.path.expanduser('./weights/vgg16_1.h5')
                self.vgg_model = load_model(model_path)
                self.graph = tensorflow.get_default_graph()
            except:
                rospy.logerr(
                    "Could not load model. Have you downloaded the vgg16_1.h5 file to the weights folder? You can download it here: https://s3-eu-west-1.amazonaws.com/sdcnddata/vgg16_1.h5")


        self.initialized = True


        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[w.pose.pose.position.x, w.pose.pose.position.y] for w in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        # only process every 10th frame for performance
        #self.image_counter = (self.image_counter + 1) % 10
        #if self.image_counter == 0:
        #    return

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

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        return self.waypoint_tree.query([x, y], 1)[1]

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # For testing, just return the light state:
        return light.state
        # if (not self.has_image):
        #     self.prev_light_loc = None
        #     return False
        #
        # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        #
        # # Get classification
        # return self.light_classifier.get_classification(cv_image)

    def classify_traffic_light(self):
        """Determine the state of the traffic light in the scene (if any)
           Using a VGG16 network to classify.
           https://github.com/FabianHertwig/CarND-Capstone

        Args:
            None

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
                 UNKNOWN if not found

        """
        if self.vgg_model and self.initialized and self.camera_image and self.graph:
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
            with self.graph.as_default():
                x = cv2.resize(src=cv_image, dsize=(256, 256))
                x = x / 255.0
                x = np.expand_dims(x, 0)
                predicted_class = self.vgg_model.predict(x)
                predicted_class = np.argmax(predicted_class)
                if predicted_class == 0:
                    return TrafficLight.RED
                elif predicted_class == 1:
                    return TrafficLight.YELLOW
                elif predicted_class == 2:
                    return TrafficLight.GREEN

        return TrafficLight.UNKNOWN

    def detect_traffic_light(self):
        """Determine the state of the traffic light in the scene (if any)
           Using a Yolo_V2 network to detect and classify in a single step.
           https://github.com/darknight1900/MobileDet

        Args:
            None


            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        """
        # Lisa        styx_msgs/TrafficLight[] (uint8)
        # stop=0      RED=0
        # go=1        GREEN=2
        # warning=2   YELLOW=1
        # dontcare=3  UNKNOWN=4

        getstate = {"stop" :     TrafficLight.RED,
                    "warning" :  TrafficLight.YELLOW, 
                    "go" :       TrafficLight.GREEN, 
                    "donotcare" : TrafficLight.UNKNOWN }

        predicted_class = "donotcare"

        if self.sess and self.initialized and self.camera_image and self.graph:
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # resized_image = cv_image.resize(
            #     tuple(reversed(self.model_image_size)))
            height, width, channels = cv_image.shape
            resized_image = cv2.resize(cv_image, tuple(reversed(self.model_image_size)))
            image_data = np.array(resized_image, dtype='float32')
            image_data /= 255.
            image_data = np.expand_dims(image_data, 0)  # Add batch dimension.

            start = time.time()
            with self.graph.as_default():
                out_boxes, out_scores, out_classes = self.sess.run(
                    [self.boxes, self.scores, self.classes],
                    feed_dict={
                        self.yolo_model.input: image_data,
                        self.input_image_shape: [width, height],
                        K.learning_phase(): 0
                    })
            last = (time.time() - start)
            
            # print('{}: Found {} boxes'.format(last, len(out_boxes)))

            for i, c in reversed(list(enumerate(out_classes))):
                predicted_class = self.class_names[c]
                box = out_boxes[i]
                score = out_scores[i]
                label = 'detector: {} {:.2f}'.format(predicted_class, score)
                print(label)
            # print(lightstate[predicted_class])

        # Return the state of the class with the highest probability (if any), UNKNOWN otherise
        return getstate[predicted_class]


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        

        if self.pose and self.waypoints and self.waypoint_tree:
            closest_light = None
            line_wp_idx = None

            # List of positions that correspond to the line to stop in front of for a given intersection
            stop_line_positions = self.config['stop_line_positions']


            car_position = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
        else:
            return -1, TrafficLight.UNKNOWN

        closest_light, distance, line_wp_idx = self.get_closest_light_in_front(car_position, stop_line_positions)

        if closest_light and distance < WAYPOINT_LOOKAHEAD:
            state = self.get_light_state(closest_light)

            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                # Get stop line waypoint index
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                distance = temp_wp_idx - car_position

                if 0 <= distance < diff:
                    diff = distance
                    closest_light = light
                    line_wp_idx = temp_wp_idx

            if self.ground_truth:
                if closest_light:
                    state = self.get_light_state(closest_light)
                    return line_wp_idx, state
            elif self.is_site:
                state = self.detect_traffic_light()
                if state != TrafficLight.UNKNOWN:
                    return line_wp_idx, state
            else:
                if not self.pose:
                    print('xxxxxxxxxxxxxxx')
                # print(self.pose)
                # print(self.waypoints)
                # print(self.waypoint_tree)
                state = self.classify_traffic_light()
                if state != TrafficLight.UNKNOWN:
                    return line_wp_idx, state

        return -1, TrafficLight.UNKNOWN



if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
