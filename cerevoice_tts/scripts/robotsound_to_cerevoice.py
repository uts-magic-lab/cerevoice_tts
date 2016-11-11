#!/usr/bin/env python

import rospy

from sound_play.msg import SoundRequest
from cerevoice_tts_msgs.msg import TtsAction, TtsGoal
from actionlib import SimpleActionClient

"""
Hook up /robotsound SoundRequest goals
into cerevoice_tts action server.

Author: Sammy Pfeiffer
"""


class RobotSoundToCerevoice(object):
    def __init__(self):
        self.ac = SimpleActionClient('/tts',
                                     TtsAction)
        while not self.ac.wait_for_server(rospy.Duration(5.0)):
            rospy.loginfo("Waiting for /tts action server...")
        rospy.loginfo("Connected!")
        self.sub = rospy.Subscriber('/robotsound',
                                    SoundRequest,
                                    self.sound_cb,
                                    queue_size=1)
        rospy.loginfo("Subscribed to /robotsound, will replay texts!")

    def sound_cb(self, msg):
        g = TtsGoal()
        g.text = msg.arg
        rospy.loginfo("Saying: " + str(g.text))
        self.ac.send_goal(g)


if __name__ == '__main__':
    rospy.init_node('robotsound_to_cerevoice')
    rc = RobotSoundToCerevoice()
    rospy.spin()
