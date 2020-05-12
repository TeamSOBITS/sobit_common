#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from text_to_speech.srv import TextToSpeech


def speak_txt(txt):
    """
    指定された文章を発話するサービスを呼び出す
    Args:
        txt           : 発話する文章
    Returns:
        True or False : 成功ならTrue, 失敗ならFalse
    """
    rospy.loginfo('waiting service')
    rospy.wait_for_service('/speech_word')
    try:
        service = rospy.ServiceProxy('/speech_word', TextToSpeech)
        response = service(txt)
        print("Service call success")
        return True
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
    return False


if __name__ == "__main__":
    rospy.init_node("speak")
    speak_txt("Yes")
