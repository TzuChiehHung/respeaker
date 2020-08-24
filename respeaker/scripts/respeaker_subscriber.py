#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from respeaker_msgs.msg import RespeakerMsg
import numpy as np


class RespeakerSubscriber(object):


    def __init__(self):
        self.sub_audio = rospy.Subscriber('audio', RespeakerMsg, self.callback)

    def callback(self, data):
        print(data)


if __name__ == '__main__':
    rospy.init_node('respeaker_subscriber', anonymous=True)
    RespeakerSubscriber()
    rospy.spin()
