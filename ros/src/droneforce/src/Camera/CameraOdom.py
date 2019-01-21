#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseWithCovariance, Pose, TwistWithCovariance, Point, Quaternion
from nav_msgs.msg import Odometry



class CameraOdom(object):
  def __init__(self):
    rospy.init_node("Camera Odom Publisher")

    self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)

    rate = rospy.Rate(30)

    odom = Odometry()
    # odom.header = Header()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "/odom"


    while not rospy.is_shutdown():
      odom.pose.pose.position = Point(0.691202606265655, 1.5, 1.87676289314649)
      odom.pose.pose.orientation = Quaternion()
      self.odom_pub.publish(odom)
      rate.sleep()






if __name__ == "__main__":
  CameraOdom()
