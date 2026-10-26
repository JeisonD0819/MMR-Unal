#include "my_robot_hardware/arm_hardware_interface.hpp"

namespace Arm_hardware {

hardware_interface::CallbackReturn ArmHardwareInterface::on_init
    (const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    joint1_motor_id = std::stoi(info_.hardware_parameters["joint1_motor_id"]);
    joint2_motor_id = std::stoi(info_.hardware_parameters["joint2_motor_id"]);
    joint3_motor_id = std::stoi(info_.hardware_parameters["joint3_motor_id"]);
    joint4_motor_id = std::stoi(info_.hardware_parameters["joint4_motor_id"]);

    port_ = info_.hardware_parameters["dynamixel_port"];

    driver_ = std::make_shared<XL430Driver>(port_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_configure
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    if (driver_->init() !=0) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_activate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    driver_ -> activateWithPositionMode(joint1_motor_id);
    driver_ -> activateWithPositionMode(joint2_motor_id);
    driver_ -> activateWithPositionMode(joint3_motor_id);
    driver_ -> activateWithPositionMode(joint4_motor_id);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    driver_ ->deactivate(joint1_motor_id);
    driver_ ->deactivate(joint2_motor_id);
    driver_ ->deactivate(joint3_motor_id);
    driver_ ->deactivate(joint4_motor_id);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArmHardwareInterface::read
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;

    set_state("arm_joint1/position",driver_-> getPositionRadian(joint1_motor_id));
    set_state("arm_joint2/position",driver_-> getPositionRadian(joint2_motor_id));
    set_state("arm_joint3/position",driver_-> getPositionRadian(joint3_motor_id));
    set_state("arm_joint4/position",driver_-> getPositionRadian(joint4_motor_id));
    
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmHardwareInterface::write
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    driver_ -> setTargetPositionRadian(joint1_motor_id, get_command("arm_joint1/position"));
    driver_ -> setTargetPositionRadian(joint2_motor_id, get_command("arm_joint2/position"));
    driver_ -> setTargetPositionRadian(joint3_motor_id, get_command("arm_joint3/position"));
    driver_ -> setTargetPositionRadian(joint4_motor_id, get_command("arm_joint4/position"));

    return hardware_interface::return_type::OK;
}

} // namespace Arm_hardware
 
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(Arm_hardware::ArmHardwareInterface, hardware_interface::SystemInterface)