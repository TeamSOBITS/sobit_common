#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sobit_common_msg.msg import StringArray
from sobit_common_msg.srv import RunCtrl

detect_object_list = StringArray()


def callback_detect_objects(msg):
    """
    ssdで検出したObjectのリストをSubscribe
    """
    global detect_object_list
    detect_object_list = msg
    print(detect_object_list)


def ssd_predict_ctrl(mode):
    """
    ssdによる推論の実行をコントロールするサービスを呼び出す
    Args:
        mode          : True = 推論の開始, False = 推論のストップ
    Returns:
        True or False : 成功ならTrue, 失敗ならFalse
    """
    rospy.wait_for_service("/pytorch_ssd/run_ctrl", 3.0)
    try:
        service = rospy.ServiceProxy("/pytorch_ssd/run_ctrl", RunCtrl)
        res = service(mode)
        return res.response
    except rospy.ServiceException as e:
        rospy.logerr("ssd_ctrl service call failed : {}".format(e))
    return False


def start_object_recognition(target_obj, timeout_sec=15.0):
    """
    ssdによる物体認識を開始する
    Args:
        target_obj    : 見つけたいObjectの名前
        timeout_sec   : ssdによる推論の最大起動時間[sec]
    Returns:
        True or False : 見つけたならTrue, 見つからなかったらFalse
    """
    global detect_object_list
    detect_object_list = []

    ssd_predict_ctrl(True)
    rospy.Subscriber("/pytorch_ssd/detect_list", StringArray, callback_detect_objects)

    is_detected = False
    start_time = rospy.Time.now()

    while rospy.Time.now() < (start_time + rospy.Duration(timeout_sec)):
        _detect_object_list = detect_object_list.copy()
        for detect_object in _detect_object_list:
            if detect_object == target_obj:
                is_detected = True
                rospy.loginfo('Object_Recog -> detect [%s]', target_obj)
                break
        rospy.sleep(0.5)
    ssd_predict_ctrl(False)
    rospy.sleep(1.0)

    if is_detected:
        return True
    else:
        return False


if __name__ == '__main__':
    rospy.init_node('recog_test')
    start_object_recognition('potato_chips')
