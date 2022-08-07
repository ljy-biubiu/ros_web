#!/usr/bin/env python
# -*- coding: utf-8 -*

from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
import rospy
import numpy as np

class state_monitor:
  def __init__(self, type_name, topic_name, name):
    self._name = name
    self._type_name = type_name
    self._topic_name = topic_name
    self._state = True
    self._stamp = rospy.get_rostime()
    self._dictionary ={"std_msgs/Bool" : Bool,"sensor_msgs/Imu" : Imu, "sensor_msgs/PointCloud2" : PointCloud2,  "nav_msgs/Odometry" : Odometry,"sensor_msgs/Image" : Image, "sensor_msgs/NavSatFix" : NavSatFix}
    self.sub()
    self.timer()

  def sub(self):
    rospy.Subscriber(self._topic_name, self._dictionary[self._type_name], self.callback)

  def callback(self, rosmsg):
    self._stamp = rospy.get_rostime()

  def timer_callback(self, event):
    self._timer_stamp = rospy.get_rostime()
    time_difference = abs(self._timer_stamp - self._stamp)
    if time_difference > rospy.Duration(0.5):
      self._state = False
    else:
      self._state = True

  def timer(self):  
    rospy.Timer(rospy.Duration(1), self.timer_callback)
    
def monitor_setup():
    param_list = rospy.get_param("/state_monitor_node/status_monitor")
    monitor_list = []
    for value in param_list:
      monitor_list.append(state_monitor(value['type'], value['topic'], value['name'])) 
    return monitor_list  

def publisher_callback(event):
    global monitor_list
    global sensor_state_pub    
    sensor_state = Bool()
    sensor_state.data = True
    for obj in monitor_list:
      if obj._state == False:
        sensor_state.data = False
    sensor_state_pub.publish(sensor_state)


if __name__ == '__main__':
  rospy.init_node('state_monitor_node', anonymous=True)
  global monitor_list
  global sensor_state_pub
  sensor_state_pub = rospy.Publisher("/cti/status_monitor/sensor_state", Bool, queue_size = 1)
  monitor_list = monitor_setup()
  rospy.Timer(rospy.Duration(0.2), publisher_callback)
  rospy.spin()

        
