#!/usr/bin/env python

import rospy
from cerevoice_tts_msgs.msg import TtsActionFeedback

if __name__ == '__main__':
    rospy.init_node('print_words')

    def cb(msg):
        if not msg.feedback.last_said.startswith("cptk_"):
            print msg.feedback.last_said

    rospy.Subscriber('/tts/feedback', TtsActionFeedback, cb, queue_size=5)

    rospy.spin()
