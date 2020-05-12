cmake_minimum_required(VERSION 2.8.3)
project(sobit_common_msg)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
 FILES
 current_state.msg
 current_state_array.msg
 BoundingBox.msg
 BoundingBoxes.msg
 StringArray.msg
 ObjectPose.msg
 ObjectPoseArray.msg
)

add_service_files(
 FILES
 grasping_jedgment.srv
 gripper_ctrl.srv
 gripper_move.srv
 odom_base.srv
 robot_motion.srv
 wheel_ctrl.srv
 wheel_control.srv
 put_ctrl.srv
 RunCtrl.srv
)

generate_messages(
 DEPENDENCIES
 std_msgs
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
