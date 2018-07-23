from styx_msgs.msg import TrafficLight
from keras.models import load_model
from keras import backend as K
import tensorflow as tf
import numpy as np
import cv2
import rospy
import os



class TLClassifier(object):

    model = None

    def __init__(self):
        cwd = os.path.dirname(os.path.realpath(__file__))
        os.chdir(cwd)
        try:
            self.model = load_model('vgg16_1.h5')
            # this is key : save the graph after loading the model
            self.graph = tf.get_default_graph()
        except:
            rospy.logerr("Could not load model. Have you downloaded the vgg16_1.h5 file to the light_classification folder? You can download it here: hhtps://s3-eu-west-1.amazonaws.com/sdcnddata/vgg_16_1.h5")


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
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
