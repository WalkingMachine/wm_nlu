#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from lab_ros_speech_to_text.msg import *
from wm_nlu.srv import *
from wm_tts.msg import *

class test:
    def __init__(self):
        self.pub = rospy.Publisher('say', say, queue_size=10)

    def stt_callback(self,data):
        if data.is_final:
            rospy.loginfo(data.text)
            answer = self.answer_question_client(data.text)
            if answer:
                rospy.loginfo(answer.str.data)

                answer_tts = say()
                answer_tts.sentence = answer.str.data
                self.pub.publish(answer_tts)

    def answer_question_client(self,question):

        rospy.wait_for_service('answer_question')
        try:
            answer_question = rospy.ServiceProxy('answer_question', AnswerQuestion)
            resp = answer_question(String(question))
            return resp
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def listener(self):

        rospy.init_node('stt_listener', anonymous=True)
        rospy.Subscriber("stt", Speech, self.stt_callback)
        rospy.spin()


if __name__ == '__main__':
    test = test()
    test.listener()