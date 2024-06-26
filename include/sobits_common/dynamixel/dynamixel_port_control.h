#ifndef _DYNAMIXEL_PORT_CONTROL_H
#define _DYNAMIXEL_PORT_CONTROL_H

// #include <memory>
#include <vector>

#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include <sobits_msgs/current_state.h>
#include <sobits_msgs/current_state_array.h>
#include "sobits_common/dynamixel/dynamixel_joint_control.h"
#include "sobits_common/dynamixel/dynamixel_setting.h"

namespace dynamixel_port_control {
typedef struct {
    int16_t home;
    double  home_rad;
    int16_t start;
    double  start_rad;
    int16_t step;
    double  step_rad;
} HomeMotionData;

class DynamixelPortControl : public hardware_interface::RobotHW {
    public:
        DynamixelPortControl( ros::NodeHandle nh, dynamixel_setting::DynamixelSetting &setting );
        ~DynamixelPortControl(){};

        ros::Time     getTime() const { return ros::Time::now(); }
        ros::Duration getDuration( ros::Time t ) const { return (ros::Time::now() - t); }

        bool read ( ros::Time time, ros::Duration period );
        void write( ros::Time time, ros::Duration period );

        void initializeSettingParam();
        void updateSettingParam(){};

        void setTorque   ( uint8_t id, bool torque );
        void setTorqueAll( bool torque );

        bool setCurrentLimit   ( uint8_t id, uint16_t current_lim );
        void setVelocityLim    ( uint8_t id, uint32_t vel_lim );
        void setAccelerationLim( uint8_t id, uint32_t acc_lim );

        void setPositionDGain( uint8_t id, uint16_t d_gain );
        void setPositionIGain( uint8_t id, uint16_t i_gain );
        void setPositionPGain( uint8_t id, uint16_t p_gain );

        void setOperationMode( uint8_t id, uint8_t mode );

        bool startUpPosition();

        void readPosition( ros::Time time, ros::Duration period );
        void readVelocity( ros::Time time, ros::Duration period );
        void readCurrent ( ros::Time time, ros::Duration period );

        int  getCurrentLoad( uint8_t id );

        std::vector<dynamixel_control::DynamixelControl> joint_list_;

    private:
        bool    dxl_res_;
        uint8_t joint_num_;

        dynamixel::PacketHandler * packet_handler_;
        dynamixel::PortHandler *   port_handler_;

        hardware_interface::JointStateInterface    jnt_state_interface_;
        hardware_interface::PositionJointInterface jnt_pos_interface_;
        joint_limits_interface::PositionJointSoftLimitsInterface jnt_limit_interface_;

        std::unique_ptr<dynamixel::GroupBulkRead>  read_status_group_;
        std::unique_ptr<dynamixel::GroupBulkRead>  read_current_group_;
        std::unique_ptr<dynamixel::GroupBulkWrite> write_position_group_;

        ros::Publisher pub_current_;
};

}  // namespace dynamixel_port_control

#endif /* _DYNAMIXEL_PORT_CONTROL_H */