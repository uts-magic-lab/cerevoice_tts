#!/usr/bin/env python

import rospy
from actionlib import SimpleActionClient
from cerevoice_tts_msgs.msg import TtsAction, TtsGoal, TtsFeedback

if __name__ == '__main__':
    rospy.init_node('test_sentences')

    rospy.loginfo("Waiting for /tts action server...")
    tts_ac = SimpleActionClient('/tts', TtsAction)
    tts_ac.wait_for_server()

    def feedback_callback(feedback_msg):
        rospy.loginfo("Got feedback: " + str(feedback_msg))

    tts_ac.feedback_cb = feedback_callback
    rospy.loginfo("Connected!")

    g = TtsGoal()

    # Sentences from the examples of the SDK PDF

    s1 = "Testing <mark name='MyEasyToRecognizeMarker'/>1 2 3"
    g.text = s1

    rospy.loginfo("Saying: " + str(s1))
    tts_ac.send_goal_and_wait(g)

    s2 = "Today, <voice emotion='happy'>the sun is shining.</voice>"
    g.text = s2

    rospy.loginfo("Saying: " + str(s2))
    tts_ac.send_goal_and_wait(g)

    s3 = "The outbreak<voice emotion='sad'>cast a shadow</voice> over the family."
    g.text = s3

    rospy.loginfo("Saying: " + str(s3))
    tts_ac.send_goal_and_wait(g)

    s4 = '<spurt audio="g0001_004">cough</spurt>, excuse me, <spurt audio="g0001_018">err</spurt>, hello'
    g.text = s4

    rospy.loginfo("Saying: " + str(s4))
    tts_ac.send_goal_and_wait(g)

    # Prosody stuff docu: https://www.w3.org/TR/speech-synthesis

    s5 = '<prosody contour="(0%,+20Hz) (10%,+30%) (40%,+10Hz)"> good morning </prosody>'
    g.text = s5

    rospy.loginfo("Saying: " + str(s5))
    tts_ac.send_goal_and_wait(g)

    s6 = '<prosody duration="2s"> good </prosody> morning'
    g.text = s6

    rospy.loginfo("Saying: " + str(s6))
    tts_ac.send_goal_and_wait(g)


    s7 = 'good <prosody volume="+10"> good </prosody> good <prosody volume="-10"> good </prosody>'
    g.text = s7

    rospy.loginfo("Saying: " + str(s7))
    tts_ac.send_goal_and_wait(g)

