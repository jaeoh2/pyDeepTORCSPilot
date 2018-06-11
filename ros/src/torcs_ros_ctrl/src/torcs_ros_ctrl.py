#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Header
from torcs_msgs.msg import TORCSSensors, TORCSCtrl
from sensor_msgs.msg import Image

import numpy as np
import threading

import json

from scipy import misc

from keras.optimizers import SGD
from keras.models import model_from_json

# refer from : https://github.com/udacity/self-driving-car/blob/master/steering-models/steering-node/steering_node.py

class SteeringNode(object):
    def __init__(self, get_model_callback, model_callback):
        rospy.init_node('steering_model')
        self.model = get_model_callback()
        self.get_model = get_model_callback
        self.predict = model_callback
        self.img = None
        self.steering = 0.0
        self.image_lock = threading.RLock()
        self.image_sub = rospy.Subscriber('/image_color', Image,
                                          self.update_image)
        self.pub = rospy.Publisher('/torcs_ctrl', TORCSCtrl, queue_size=1)
        rospy.Timer(rospy.Duration(0.02), self.get_steering)

    def update_image(self, img):
        d = map(ord, img.data) # ord:return ASCII code
        arr = np.ndarray(shape=(img.height, img.width, 3),
                         dtype=np.int,
                         buffer=np.array(d)[:,:,::-1])
        if self.image_lock.acquire(True):
            self.img = arr
            if self.model is None:
                self.model = self.get_model()
            self.steering = self.predict(self.model, self.img)
            self.image_lock.release()

    def get_steering(self, event):
        if self.img is None:
            return
        message = TORCSCtrl()
        message.steering = self.steering
        message.accel = 0.6
        message.brake = 0.0
        message.clutch = 0.0
        message.gear = 3

        self.pub.publish(message)


def process(model, img):
    img = misc.imresize(img[320:, :, :], (50, 200, 3))
    steering = model.predict(img[None, :, :, :])[0][0]
    print steering
    return steering


def get_model(model_file):
    with open(model_file, 'r') as jfile:
        model = model_from_json(json.load(jfile))

    sgd = SGD(lr=0.0001, decay=1e-6, momentum=0.9, nesterov=True)
    model.compile(sgd, "mse")
    weights_file = model_file.replace('json', 'keras')
    model.load_weights(weights_file)
    return model


if __name__ == "__main__":
    model_json = '' # path to model definition json
    node = SteeringNode(lambda: get_model(model_json), process)
    rospy.spin()
