#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import dynamic_reconfigure.client

from move_base_msgs.msg import RecoveryStatus
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Float32

from sound_play.libsoundplay import SoundClient

class RestorePadding(object):

    def __init__(self):
        self.shrink = False
        self.padding_name = rospy.get_param("~padding_name", "/move_base_node/global_costmap/footprint_padding")
        self.global_name = rospy.get_param("~global_name", "/move_base_node/global_costmap")
        # self.default_padding = rospy.get_param(self.padding_name)
        self.client = dynamic_reconfigure.client.Client(self.global_name, timeout=5.0)
        self.client_en = SoundClient(sound_action='/robotsound', blocking=True)
        self.recovery_sub = rospy.Subscriber("/move_base/recovery_status", RecoveryStatus, self.recovery_cb)
        self.move_base_sub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.move_base_cb)
        self.width_sub = rospy.Subscriber("/compute_y_distance/output", Float32, self.width_cb)
        self.width = 0

    def speak(self, client, speech_text):
        client.say(speech_text, volume=1.0, replace=False)
        return client.actionclient.get_result()

    def recovery_cb(self, msg):
        # self.speak(self.client_en, msg.recovery_behavior_name)
        if msg.recovery_behavior_name.startswith('expand_padding'):
            self.shrink = True

    def move_base_cb(self, msg):
        if self.shrink is True:
            self.default_padding = max(self.width - 0.5, 0.1)
            self.client.update_configuration({"footprint_padding" : self.default_padding})
            rospy.loginfo("restore footprint_padding to {}".format(self.default_padding))
            self.shrink = False

    def width_cb(self, msg):
        self.width = msg.data

if __name__ == '__main__':
    rospy.init_node('restore_padding')
    rp = RestorePadding()
    rospy.spin()
