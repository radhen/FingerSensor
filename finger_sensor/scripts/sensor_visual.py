#!/usr/bin/env python
"""
Run
$ export MPLBACKEND="module://gr.matplotlib.backend_gr"
before running the visualization to use the gr backend
http://gr-framework.org/tutorials/matplotlib.html
It has built in double buffering, and works great. It's
supposed to be fast too, but it maybe feels a tiny bit
more sluggish, maybe because of the lack of flickering.
"""
import rospy
from std_msgs.msg import Int32MultiArray

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl


def Int32MultiArray2np(msg):
    arr = np.array(msg.data)
    return arr

    # Not working:
    # Ignoring data_offset
    # shape = [i.size for i in msg.layout.dim]
    # strides = [i.stride for i in msg.layout.dim]
    # print(shape, strides, arr)
    # return np.lib.stride_tricks.as_strided(arr, shape, strides)


class FingerSensorVisualizer(object):
    def __init__(self, topic='/sensor_values'):
        self.nh = rospy.init_node('FingerSensorVisualizer', anonymous=True)
        cm = mpl.cm.get_cmap('YlOrRd')
        self.fig, self.ax = plt.subplots()
        self.im = self.ax.imshow(np.zeros((2, 8)),
                                 cmap=cm,
                                 interpolation='none',
                                 vmin=np.log(5000),
                                 vmax=np.log(2**16 - 1),
                                 # Doesn't seem to make a difference (?)
                                 animated=True)
        self.fig.colorbar(self.im, orientation='horizontal')
        self.fig.show()
        self.fig.canvas.draw()
        # Subscribe last, otherwise the callback might be called
        # before setup is finished...
        self.sub = rospy.Subscriber(topic,
                                    Int32MultiArray,
                                    self.callback,
                                    queue_size=1)

    def callback(self, msg):
        nparr = Int32MultiArray2np(msg)
        if nparr.shape != (16,):
            raise ValueError("Need 16 sensor values!")

        data = nparr.reshape(2, 8)
        self.im.set_data(np.log(data))
        self.ax.set_title(str(data))
        self.fig.canvas.draw()


if __name__ == '__main__':
    vis = FingerSensorVisualizer()
    rospy.spin()
