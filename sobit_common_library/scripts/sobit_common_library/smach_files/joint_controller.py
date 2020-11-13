#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import math
import sobit_common_msg.srv


def open_gripper():
    """
    グリッパーを開くサービスを呼び出す
    Return:
        True or False : 成功ならTrue, 失敗ならFalse
    """
    rospy.wait_for_service("/robot_ctrl/gripper_open_and_close")
    try:
        gripper_open_service = rospy.ServiceProxy("/robot_ctrl/gripper_open_and_close", sobit_common_msg.srv.gripper_ctrl)
        res = gripper_open_service(0.4)
        return res.is_moved
    except rospy.ServiceException as e:
        rospy.logerr("Gripper_open Service call failed: %s", e)
        return False


def close_gripper():
    """
    グリッパーを閉じるサービスを呼び出す
    Return:
        True or False : 成功ならTrue, 失敗ならFalse
    """
    rospy.wait_for_service("/robot_ctrl/gripper_open_and_close")
    try:
        gripper_open_service = rospy.ServiceProxy("/robot_ctrl/gripper_open_and_close", sobit_common_msg.srv.gripper_ctrl)
        res = gripper_open_service(0.00)
        return res.is_moved
    except rospy.ServiceException as e:
        rospy.logerr("Gripper_close Service call failed: %s", e)
        return False


def move_gripper_to_target(target_frame_name, shift):
    """
    グリッパーを目標のtfの位置まで移動させるサービスを呼び出す
    Args:
        target_frame_name : 目標のtf名
        shift             : 目標のtfからどれだけずらすか
    Return:
        True or False : 成功ならTrue, 失敗ならFalse
    """
    rospy.wait_for_service('/robot_ctrl/gripper_move_to_target')
    try:
        gripper_move_to_target_service = rospy.ServiceProxy('/robot_ctrl/gripper_move_to_target', sobit_common_msg.srv.gripper_move)
        res = gripper_move_to_target_service(target_frame_name, shift)
        return res.is_moved
    except rospy.ServiceException as e:
        rospy.logerr("Gripper_move Service call failed: %s", e)
        return False


def registered_motion(motion_type):
    """
    事前に登録したモーション通りにロボットを動かすサービスを呼び出す
    Args:
        motion_type   : 登録したモーション名
    Return:
        True or False : 成功ならTrue, 失敗ならFalse
    """
    rospy.wait_for_service("/robot_ctrl/motion_ctrl")
    try:
        registered_motion_service = rospy.ServiceProxy("/robot_ctrl/motion_ctrl", sobit_common_msg.srv.robot_motion)
        res = registered_motion_service(motion_type)
        return res.is_moved
    except rospy.ServiceException as e:
        rospy.logerr("motion_ctrl Service call failed: %s", e)
        return False


def xtion_ctrl(deg):
    """
    xtionのPan, Tiltを動かすサービスを呼び出す
    Args:
        deg           : 動かす角度
    Return:
        True or False : 成功ならTrue, 失敗ならFalse
    """
    rospy.wait_for_service("/robot_ctrl/xtion_ctrl")
    rad = math.radians(deg)
    try:
        gripper_open_service = rospy.ServiceProxy("/robot_ctrl/xtion_ctrl", sobit_common_msg.srv.gripper_ctrl)
        res = gripper_open_service(rad)
        return res.is_moved
    except rospy.ServiceException as e:
        rospy.logerr("xtion_ctrl Service call failed: %s", e)
        return False
