#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from audio_common_msgs.msg import AudioData
import numpy as np


class RespeakerSubscriber(object):


    def __init__(self):
        self.sub_audio = rospy.Subscriber('audio', AudioData, self.callback)

    def callback(self, data):
        data = np.fromstring(data.data, dtype=np.int16)
        print(data)


if __name__ == '__main__':
    rospy.init_node('respeaker_subscriber', anonymous=True)
    RespeakerSubscriber()
    rospy.spin()
