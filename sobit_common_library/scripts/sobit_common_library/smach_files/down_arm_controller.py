#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from sobit_common_msg.srv import put_ctrl


def down_arm():
    """
    アームを段階的に下げていき、机と衝突したらその位置でアームを止めるサービスを呼び出す
    Return:
        True or False : 成功ならTrue, 失敗ならFalse
    """
    rospy.wait_for_service("/robot_ctrl/put_controller")
    try:
        put_ctrol = rospy.ServiceProxy("/robot_ctrl/put_controller", put_ctrl)
        res = put_ctrol()
        return res.result
    except rospy.ServiceException as e:
        rospy.logerr(e)
    return False


if __name__ == "__main__":
    rospy.init_node("put_controller_client_node")
    down_arm()
