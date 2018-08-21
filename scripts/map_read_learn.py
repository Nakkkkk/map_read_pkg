#!/usr/bin/python
# coding: UTF-8

import rospy
import cv2
import tf
import numpy as np
from nav_msgs.msg import *
from geometry_msgs.msg import *

class map_read():
  def __init__(self):
    self.sub = rospy.Subscriber('map', OccupancyGrid, self.mapCallback)
    rospy.loginfo('A')

  def mapCallback(self,data):
    rospy.loginfo('A')
    orientation = data.info.origin.orientation
    e = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))

    rospy.loginfo("Received a %d X %d map @ %.3f m/pix_%f_%f_%f",
    data.info.width,data.info.height,data.info.resolution,
    data.info.origin.position.x,data.info.origin.position.y,e[2])

    size = data.info.width, data.info.height, 1
    img_out = np.zeros(size, dtype=np.uint8)

    for y in range(data.info.height):
      for x in range(data.info.width):
        i = x + (data.info.height - y - 1) * data.info.width
        intensity = 205;
        if data.data[i] >= 0 and data.data[i] <=100:
          intensity = round((100.0 - data.data[i])*2.55);
        img_out[y][x] = intensity;

#    cv2.imwrite('/home/hoge/test_map.png', img_out)

if __name__ == '__main__':
  rospy.init_node('map_read_learn',anonymous=True)

  try:
    mr = map_read()
    rospy.spin()
  except rospy.ROSInterruptException: pass
