#!/usr/bin/env python
import rospy
from move_base_msgs.msg import RecoveryStatus
from actionlib_msgs.msg import GoalStatusArray

class RestorePadding(object):

    def __init__(self):
        self.recovery_sub = rospy.Subscriber("/move_base/recovery_status", RecoveryStatus, self.recovery_cb)
        self.move_base_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.move_base_cb)

    def recovery_cb(self, msg):
        # if padding is small, start checking environment and move_base status

if __name__ == '__main__':
    rospy.init_node('restore_padding')
    rp = RestorePadding()
    rp.run()
    rospy.spin()
