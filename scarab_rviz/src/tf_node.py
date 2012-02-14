#!/usr/bin/env python
#
# A node that listens for PoseWithCovarianceStamped and publishes PoseStamped
import roslib; roslib.load_manifest('scarab_rviz')
import rospy

import threading

import geometry_msgs.msg

class Stripper(object):
    def __init__(self):
        self._pub = rospy.Publisher("pose_stamped", geometry_msgs.msg.PoseStamped)
        self._pose_sub = rospy.Subscriber("amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped,
                                          self._pose_callback)
        
    def _pose_callback(self, pose_msg):
            msg = geometry_msgs.msg.PoseStamped(header = pose_msg.header,
                                                pose = pose_msg.pose.pose)
            self._pub.publish(msg)

def main():
    rospy.init_node('range_pose_aggregator')

    strip = Stripper()
    
    rospy.spin()

if __name__ == "__main__":
    main()
