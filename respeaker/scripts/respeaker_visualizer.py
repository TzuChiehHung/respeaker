#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from respeaker_msgs.msg import RespeakerMsg
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from scipy import signal


class RespeakerVisualizer:


    def __init__(self):
        self.fig, self.axs = plt.subplots(2,1)

        self.fs = 16000
        self.data = []
        self.chunk = 1024
        self.t = np.arange(0, self.chunk)

        # time series
        self.raw, = self.axs[0].plot([], [], 'b-')

        # frequency domain
        self.psd, = self.axs[1].plot([], [], 'b-')

    def plot_init(self):
        self.axs[0].set_xlim(0, self.chunk)
        self.axs[0].set_ylim(-32768, 32768)

        self.axs[1].set_xlim(0, self.fs//2)
        self.axs[1].set_ylim(-60, 60)
        return

    def audio_callback(self, msg):
        if self.fs != msg.frame_rate or self.chunk != len(msg.data):
            self.fs = msg.frame_rate
            self.chunk = len(msg.data)
            self.t = np.arange(0, self.chunk)
            self.plot_init()

        self.data = msg.data

    def update_plot(self, frame):
        self.raw.set_data(self.t, self.data)

        f, dbhz = self.power_spectral_density(self.data, self.fs)
        self.psd.set_data(f, dbhz)
        return

    def power_spectral_density(self, data, sample_rate):
        f, Pxxf = signal.welch(
            data,
            sample_rate,
            nperseg=1024,
            return_onesided=True,
            detrend='constant',
            scaling='density')
        dbhz = 10*np.log10(Pxxf)
        return f, dbhz


if __name__ == '__main__':
    rospy.init_node('respeaker_visualizer', anonymous=True)
    vis = RespeakerVisualizer()
    sub = rospy.Subscriber('/respeaker/audio', RespeakerMsg, vis.audio_callback)
    ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init, interval=100)
    plt.show(block=True)
