#!/usr/bin/env python
# -*- coding: utf-8 -*-

from contextlib import contextmanager
import os
import sys
import pyaudio
from respeaker_msgs.msg import RespeakerMsg
import rospy
import numpy as np

# suppress error messages from ALSA
# https://stackoverflow.com/questions/7088672/pyaudio-working-but-spits-out-error-messages-each-time
# https://stackoverflow.com/questions/36956083/how-can-the-terminal-output-of-executables-run-by-python-functions-be-silenced-i

@contextmanager
def ignore_stderr(enable=True):
    if enable:
        devnull = None
        try:
            devnull = os.open(os.devnull, os.O_WRONLY)
            stderr = os.dup(2)
            sys.stderr.flush()
            os.dup2(devnull, 2)
            try:
                yield
            finally:
                os.dup2(stderr, 2)
                os.close(stderr)
        finally:
            if devnull is not None:
                os.close(devnull)
    else:
        yield

# Partly copied from https://github.com/furushchev/respeaker_ros


class RespeakerAudio(object):


    def __init__(self, on_audio, channels=None, suppress_error=True):
        self.on_audio = on_audio
        with ignore_stderr(enable=suppress_error):
            self.pyaudio = pyaudio.PyAudio()
        self.available_channels = None
        self.channels = channels
        self.device_index = None
        self.rate = 16000

        # find device
        count = self.pyaudio.get_device_count()
        rospy.logdebug('%d audio devices found' % count)
        for i in range(count):
            info = self.pyaudio.get_device_info_by_index(i)
            name = info['name'].encode('utf-8')
            chan = info['maxInputChannels']
            rospy.logdebug(' - %d: %s' % (i, name))
            if name.lower().find('seeed-2mic-voicecard') >= 0:
                self.available_channels = chan
                self.device_index = i
                rospy.loginfo('Found %d: %s (channels: %d)' % (i, name, chan))
                break
        if self.device_index is None:
            rospy.logwarn('Failed to find respeaker device by name. Using default input')
            info = self.pyaudio.get_default_input_device_info()
            self.available_channels = info['maxInputChannels']
            self.device_index = info['index']

        if self.available_channels != 2:
            rospy.logwarn('%d channel is found for respeaker' % self.available_channels)
            rospy.logwarn('You may have to update firmware.')
        if self.channels is None:
            self.channels = range(self.available_channels)
        else:
            self.channels = filter(lambda c: 0 <= c < self.available_channels, self.channels)
        if not self.channels:
            raise RuntimeError('Invalid channels %s. (Available channels are %s)' % (
                self.channels, self.available_channels))
        rospy.loginfo('Using channels %s' % self.channels)

        self.stream = self.pyaudio.open(
            input=True, start=False,
            format=pyaudio.paInt16,
            channels=self.available_channels,
            rate=self.rate,
            frames_per_buffer=1024,
            stream_callback=self.stream_callback,
            input_device_index=self.device_index,
        )

    def __del__(self):
        self.stop()
        try:
            self.stream.close()
        except:
            pass
        finally:
            self.stream = None
        try:
            self.pyaudio.terminate()
        except:
            pass

    def stream_callback(self, in_data, frame_count, time_info, status):
        # split channel
        data = np.fromstring(in_data, dtype=np.int16)
        chunk_per_channel = len(data) / self.available_channels
        data = np.reshape(data, (chunk_per_channel, self.available_channels))
        for chan in self.channels:
            chan_data = data[:, chan]
            # invoke callback
            sampwidth = self.pyaudio.get_sample_size(pyaudio.paInt16)
            self.on_audio(chan_data, chan, sampwidth, self.rate)
        return None, pyaudio.paContinue

    def start(self):
        if self.stream.is_stopped():
            self.stream.start_stream()

    def stop(self):
        if self.stream.is_active():
            self.stream.stop_stream()


class RespeakerNode(object):


    def __init__(self):
        rospy.on_shutdown(self.on_shutdown)
        self.update_rate = rospy.get_param('~update_rate', 10.0)
        self.main_channel = rospy.get_param('~main_channel', 0)
        suppress_pyaudio_error = rospy.get_param('~suppress_pyaudio_error', True)

        self.respeaker_audio = RespeakerAudio(self.on_audio, suppress_error=suppress_pyaudio_error)

        # advertise
        self.pub_audio = rospy.Publisher('audio', RespeakerMsg, queue_size=10)
        self.pub_audios = {c:rospy.Publisher('audio/channel%d' % c, RespeakerMsg, queue_size=10) for c in self.respeaker_audio.channels}

        self.respeaker_audio.start()
        # self.info_timer = rospy.Timer(rospy.Duration(1.0 / self.update_rate),
        #                               self.on_timer)
        # self.timer_led = None
        # self.sub_led = rospy.Subscriber('status_led', ColorRGBA, self.on_status_led)

    def on_shutdown(self):
        try:
            self.respeaker_audio.stop()
        except:
            pass
        finally:
            self.respeaker_audio = None

    # def on_status_led(self, msg):
    #     self.respeaker.set_led_color(r=msg.r, g=msg.g, b=msg.b, a=msg.a)
    #     if self.timer_led and self.timer_led.is_alive():
    #         self.timer_led.shutdown()
    #     self.timer_led = rospy.Timer(rospy.Duration(3.0),
    #                                    lambda e: self.respeaker.set_led_trace(),
    #                                    oneshot=True)

    def on_audio(self, data, channel, sampwidth, fs):
        audio = RespeakerMsg()
        audio.sample_width = sampwidth
        audio.frame_rate = fs
        audio.data = data
        self.pub_audios[channel].publish(audio)
        if channel == self.main_channel:
            self.pub_audio.publish(audio)

    # def on_timer(self, event):
    #     stamp = event.current_real or rospy.Time.now()
    #     is_voice = self.respeaker.is_voice()
    #     doa_rad = math.radians(self.respeaker.direction - 180.0)
    #     doa_rad = angles.shortest_angular_distance(
    #         doa_rad, math.radians(self.doa_yaw_offset))
    #     doa = math.degrees(doa_rad)

    #     # vad
    #     if is_voice != self.prev_is_voice:
    #         self.pub_vad.publish(Bool(data=is_voice))
    #         self.prev_is_voice = is_voice

    #     # doa
    #     if doa != self.prev_doa:
    #         self.pub_doa_raw.publish(Int32(data=doa))
    #         self.prev_doa = doa

    #         msg = PoseStamped()
    #         msg.header.frame_id = self.sensor_frame_id
    #         msg.header.stamp = stamp
    #         ori = T.quaternion_from_euler(math.radians(doa), 0, 0)
    #         msg.pose.position.x = self.doa_xy_offset * np.cos(doa_rad)
    #         msg.pose.position.y = self.doa_xy_offset * np.sin(doa_rad)
    #         msg.pose.orientation.w = ori[0]
    #         msg.pose.orientation.x = ori[1]
    #         msg.pose.orientation.y = ori[2]
    #         msg.pose.orientation.z = ori[3]
    #         self.pub_doa.publish(msg)

    #     # speech audio
    #     if is_voice:
    #         self.speech_stopped = stamp
    #     if stamp - self.speech_stopped < rospy.Duration(self.speech_continuation):
    #         self.is_speeching = True
    #     elif self.is_speeching:
    #         buf = self.speech_audio_buffer
    #         self.speech_audio_buffer = str()
    #         self.is_speeching = False
    #         duration = 8.0 * len(buf) * self.respeaker_audio.bitwidth
    #         duration = duration / self.respeaker_audio.rate / self.respeaker_audio.bitdepth
    #         rospy.loginfo('Speech detected for %.3f seconds' % duration)
    #         if self.speech_min_duration <= duration < self.speech_max_duration:

    #             self.pub_speech_audio.publish(AudioData(data=buf))


if __name__ == '__main__':
    rospy.init_node('respeaker_node')
    RespeakerNode()
    rospy.spin()
