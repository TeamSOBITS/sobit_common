#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sobit_common_msg.srv import wheel_control


def processing(req_straight, req_turn):
    """
    指定した距離・角度に合わせて移動を行うサービスを呼び出す
    Args:
        req_straight  : 直進距離
        req_turn      : 回転量
    Return:
        True or False : 成功ならTrue, 失敗ならFalse
    """
    rospy.loginfo('Waiting service')
    rospy.wait_for_service('/robot_ctrl/wheel_control')
    try:
        wheel_control_service = rospy.ServiceProxy('/robot_ctrl/wheel_control', wheel_control)
        response = wheel_control_service(req_straight, req_turn)
        print("Service call success")
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    return response


if __name__ == '__main__':
    rospy.init_node("wheel_controller_client")
    processing(0, 90)
