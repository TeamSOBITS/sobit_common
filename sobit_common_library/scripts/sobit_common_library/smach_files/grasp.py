#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point
import joint_controller
import wheel_controller


def grasp_object(object_name, shift_x=-0.45, shift_y=0.025, shift_z=0.0, move_straight_x=0.33, move_back_x=-0.35):
    """
    指定したtf名の物体把持を行う
    把持後は、INIAL_POSEに戻り
    Args:
        object_name     : 掴みたい物体のtf名
        shift_x         : 指定したtfからx方向に"shift_x"[m]ずらした地点にアームの先端を移動させる
        shift_y         : 指定したtfからy方向に"shift_y"[m]ずらした地点にアームの先端を移動させる
        shift_z         : 指定したtfからz方向に"shift_z"[m]ずらした地点にアームの先端を移動させる
        move_straight_x : アームを移動させた後に直進する距離[m]
        move_back_x     : 物体を掴んだ後に後退する距離[m]
    Return:
        True or False   : 把持に成功したならTrue, 失敗ならFalseを返す
    """
    response = joint_controller.move_gripper_to_target(object_name, Point(x=shift_x, y=shift_y, z=shift_z))
    rospy.sleep(2.0)
    if response:
        joint_controller.open_gripper()
        rospy.sleep(1.0)
        wheel_controller.processing(move_straight_x, 0.0)
        rospy.sleep(2.0)
        joint_controller.close_gripper()
        rospy.sleep(1.0)
        joint_controller.registered_motion("HIGHT_POSE")
        rospy.sleep(1.0)
        wheel_controller.processing(move_back_x, 0.0)
        rospy.sleep(1.0)
        joint_controller.registered_motion("INITIAL_POSE")
        return True
    else:
        return False


# メイン
if __name__ == '__main__':
    rospy.init_node("grasp")
    grasp_object("grape_juice")
