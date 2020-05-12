#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from web_speech_recognition.srv import SpeechRecognition


def processing(timeout_sec):
    """
    Web Speech APIを用いて音声認識を行うサービスを呼び出す
    Args:
        timeout_sec        : 音声認識の最大起動時間[sec]
    Return:
        "" or "transcript" : 音声認識結果を返す(聞き取れなかった場合は、文字列なし)
    """
    rospy.loginfo('waiting service')
    rospy.wait_for_service('/speech_recognition')

    try:
        service = rospy.ServiceProxy('/speech_recognition', SpeechRecognition)
        response = service(timeout_sec)
        print(response.transcript)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return ""
    return response.transcript


if __name__ == "__main__":
    rospy.init_node("listen")
    processing(10)
