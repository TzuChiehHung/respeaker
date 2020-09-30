#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from respeaker_msgs.msg import RespeakerMsg
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class RespeakerVisualizer:


    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], 'b-')
        self.x_data, self.y_data = [] , []

    def plot_init(self):
        self.ax.set_xlim(0, 1024)
        self.ax.set_ylim(-32768, 32768)
        return self.ln

    def audio_callback(self, msg):
        self.x_data = np.arange(0, len(msg.data))
        self.y_data = msg.data

    def update_plot(self, frame):
        self.ax.set_xlim(0, len(self.x_data))
        self.ln.set_data(self.x_data, self.y_data)
        return self.ln


if __name__ == '__main__':
    rospy.init_node('respeaker_visualizer', anonymous=True)
    vis = RespeakerVisualizer()
    sub = rospy.Subscriber('audio', RespeakerMsg, vis.audio_callback)
    ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
    plt.show(block=True)
