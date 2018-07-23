from styx_msgs.msg import TrafficLight
from keras.models import load_model
from keras import backend as K
import tensorflow as tf
import numpy as np
import cv2
import rospy
import yaml

import os

# Detector Stuff
from cfg import *
from mobiledet.utils import utils
from mobiledet.models.keras_yolo import yolo_eval, decode_yolo_output, create_model
from keras import backend as K
import time


class TLClassifier(object):
    model = None

    def __init__(self):

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.is_site = self.config['is_site']

        if self.is_site:
            # Detector Stuff
            self.model_image_size = None
            self.sess = None
            self.initialized = False
            model_path = os.path.expanduser('./weights/mobilenet_s2_best.FalseFalse.h5')
            anchors_path = os.path.expanduser('./model_data/lisa_anchors.txt')
            classes_path = os.path.expanduser('./model_data/lisa_classes.txt')

            class_names = utils.get_classes(classes_path)
            anchors = utils.get_anchors(anchors_path)
            if SHALLOW_DETECTOR:
                anchors = anchors * 2

            print(class_names)
            print(anchors)

            self.yolo_model, yolo_model_for_training = create_model(anchors, class_names, load_pretrained=True,
                                                                    feature_extractor=FEATURE_EXTRACTOR,
                                                                    pretrained_path=model_path, freeze_body=True)

            model_file_basename, file_extension = os.path.splitext(os.path.basename(model_path))

            model_input = self.yolo_model.input.name.replace(':0', '')  # input_1
            model_output = self.yolo_model.output.name.replace(':0', '')  # conv2d_3/BiasAdd

            sess = K.get_session()
            width, height, channels = int(self.yolo_model.input.shape[2]), int(self.yolo_model.input.shape[1]), int(
                self.yolo_model.input.shape[3])

            # END OF keras specific code

            # Check if model is fully convolutional, assuming channel last order.
            self.model_image_size = self.yolo_model.layers[0].input_shape[1:3]

            self.sess = K.get_session()  # TODO: Remove dependence on Tensorflow session.

            # Generate output tensor targets for filtered bounding boxes.
            self.yolo_outputs = decode_yolo_output(self.yolo_model.output, anchors, len(class_names))

            self.input_image_shape = K.placeholder(shape=(2,))
            self.boxes, self.scores, self.classes = yolo_eval(
                self.yolo_outputs,
                self.input_image_shape,
                score_threshold=.6,
                iou_threshold=.6)

            self.initialized = True


        else:
            cwd = os.path.dirname(os.path.realpath(__file__))
            os.chdir(cwd)
            try:
                self.model = load_model('vgg16_1.h5')
                # this is key : save the graph after loading the model
                self.graph = tf.get_default_graph()
            except:
                rospy.logerr(
                    "Could not load model. Have you downloaded the vgg16_1.h5 file to the light_classification folder? You can download it here: hhtps://s3-eu-west-1.amazonaws.com/sdcnddata/vgg_16_1.h5")

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if self.is_site:
            # Lisa        styx_msgs/TrafficLight[] (uint8)
            # stop=0      RED=0
            # go=1        GREEN=2
            # warning=2   YELLOW=1
            # dontcare=3  UNKNOWN=4

            if self.sess and self.initialized:
                cv_image = self.bridge.imgmsg_to_cv2(self.camera_image)
                # resized_image = cv_image.resize(
                #     tuple(reversed(self.model_image_size)))
                height, width, channels = cv_image.shape
                resized_image = cv2.resize(cv_image, tuple(reversed(self.model_image_size)))
                image_data = np.array(resized_image, dtype='float32')
                image_data /= 255.
                image_data = np.expand_dims(image_data, 0)  # Add batch dimension.

                start = time.time()
                out_boxes, out_scores, out_classes = self.sess.run(
                    [self.boxes, self.scores, self.classes],
                    feed_dict={
                        self.yolo_model.input: image_data,
                        self.input_image_shape: [width, height],
                        K.learning_phase(): 0
                    })
                last = (time.time() - start)

                print('{}: Found {} boxes for {}'.format(last, len(out_boxes), idx))

            # TODO return the actual detected state
            return TrafficLight.UNKNOWN
        else:
            if self.model:
                with self.graph.as_default():
                    x = cv2.resize(src=image, dsize=(256, 256))
                    x = x / 255.0
                    x = np.expand_dims(x, 0)
                    pred = self.model.predict(x)
                    pred = np.argmax(pred)
                    if pred == 0:
                        return TrafficLight.RED
                    elif pred == 1:
                        return TrafficLight.YELLOW
                    elif pred == 2:
                        return TrafficLight.GREEN
                    else:
                        return TrafficLight.UNKNOWN
            else:
                rospy.logerr("No model to predict traffic light state.")
        return TrafficLight.UNKNOWN
