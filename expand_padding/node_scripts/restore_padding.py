#!/usr/bin/env python
import rospy
import dynamic_reconfigure.client

from move_base_msgs.msg import RecoveryStatus
from move_base_msgs.msg import MoveBaseActionResult

class RestorePadding(object):

    def __init__(self):
        self.shrink = False
        self.padding_name = rospy.get_param("~padding_name", "/move_base_node/global_costmap/footprint_padding")
        self.global_name = rospy.get_param("~global_name", "/move_base_node/global_costmap")
        self.default_padding = rospy.get_param(self.padding_name)
        self.client = dynamic_reconfigure.client.Client(self.global_name, timeout=5.0)
        self.recovery_sub = rospy.Subscriber("/move_base/recovery_status", RecoveryStatus, self.recovery_cb)
        self.move_base_sub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.move_base_cb)

    def recovery_cb(self, msg):
        if msg.recovery_behavior_name.startswith('expand_padding'):
            self.shrink = True

    def move_base_cb(self, msg):
        if self.shrink is True:
            self.client.update_configuration({"footprint_padding" : self.default_padding})
            self.shrink = False

if __name__ == '__main__':
    rospy.init_node('restore_padding')
    rp = RestorePadding()
    rospy.spin()
