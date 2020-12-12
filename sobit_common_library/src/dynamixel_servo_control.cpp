#include <queue>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <sobit_common_msg/current_state.h>
#include "sobit_common_library/dynamixel/dynamixel_control.h"
#include "sobit_common_library/dynamixel/dynamixel_port_control.h"
#include "sobit_common_library/dynamixel/dynamixel_setting.h"

namespace {
typedef struct {
  uint8_t  dxl_id;
  uint16_t dxl_current;
} CurrentRequest;

std::queue<CurrentRequest>                    current_request_queue;
dynamixel_port_control::DynamixelPortControl* driver_addr;

void currentCtrlCallback(const sobit_common_msg::current_state msg) {
  std::string target_joint_name  = msg.joint_name;
  double      target_current_val = msg.current_ma;
  for (std::vector<dynamixel_control::DynamixelControl>::iterator it = driver_addr->joint_list_.begin(); it != driver_addr->joint_list_.end(); it++) {
    if (it->getJointName() == target_joint_name) {
      CurrentRequest current_req;
      current_req.dxl_id      = it->getDxlId();
      current_req.dxl_current = it->current2DxlCurrent(target_current_val);
      current_request_queue.push(current_req);
      break;
    }
  }
}

}  // namespace
int main(int argc, char** argv) {
  ros::init(argc, argv, ros::this_node::getName());
  ros::NodeHandle                     nh;
  ros::Subscriber                     sub_current_ctrl = nh.subscribe("/current_ctrl", 10, &currentCtrlCallback);
  dynamixel_setting::DynamixelSetting setting(nh);
  if (!setting.load()) {
    return -1;
  }
  dynamixel_port_control::DynamixelPortControl dynamixel_servo_control(nh, setting);
  controller_manager::ControllerManager        cm(&dynamixel_servo_control, nh);
  driver_addr = &dynamixel_servo_control;
  dynamixel_servo_control.initializeSettingParam();
  if (!dynamixel_servo_control.startUpPosition()) {
    return 0;
  }
  // return 0;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time     t  = dynamixel_servo_control.getTime();
  ros::Duration dt = dynamixel_servo_control.getDuration(t);
  while (ros::ok()) {
    dt = dynamixel_servo_control.getDuration(t);
    t  = dynamixel_servo_control.getTime();

    dynamixel_servo_control.read(t, dt);
    cm.update(t, dt);
    dynamixel_servo_control.write(t, dt);

    while (current_request_queue.size() > 0) {
      CurrentRequest req = current_request_queue.front();
      bool           res = dynamixel_servo_control.setCurrentLimit(req.dxl_id, req.dxl_current);
      if (res) {
        current_request_queue.pop();
      } else {
        break;
      }
    }
  }
  dynamixel_servo_control.setTorqueAll(false);
  spinner.stop();
}