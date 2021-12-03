#!/usr/bin/env python3

import pylab as plt
import numpy as np
import rospy
from audio_common_msgs.msg import AudioData

size = 650
X = np.linspace(0, size, size)
Y = (X * 0) + 255

plt.ion()
graph = plt.plot(X, Y)[0]

current_message = AudioData()


def update_display(_msg: AudioData):
    global current_message
    current_message = _msg


rospy.init_node("audio_display")
rospy.Subscriber("audio/audio", AudioData, update_display, queue_size=1)
while not rospy.is_shutdown():
    buffer = None
    try:
        buffer = np.frombuffer(current_message.data, dtype=np.uint8)
        buffer = np.append(buffer, np.zeros(size - buffer.shape[0]))
        graph.set_ydata(buffer)
        plt.draw()
        plt.pause(0.01)
    except ValueError as e:
        print(e)
        if buffer is not None:
            print("buffer shape", buffer.shape)
        print("graph length", size)
