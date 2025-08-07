// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from agrorob_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef AGROROB_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
#define AGROROB_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "agrorob_msgs/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace agrorob_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotState_left_rear_wheel_rotational_speed_rad_s
{
public:
  explicit Init_RobotState_left_rear_wheel_rotational_speed_rad_s(::agrorob_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  ::agrorob_msgs::msg::RobotState left_rear_wheel_rotational_speed_rad_s(::agrorob_msgs::msg::RobotState::_left_rear_wheel_rotational_speed_rad_s_type arg)
  {
    msg_.left_rear_wheel_rotational_speed_rad_s = std::move(arg);
    return std::move(msg_);
  }

private:
  ::agrorob_msgs::msg::RobotState msg_;
};

class Init_RobotState_right_rear_wheel_rotational_speed_rad_s
{
public:
  explicit Init_RobotState_right_rear_wheel_rotational_speed_rad_s(::agrorob_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_left_rear_wheel_rotational_speed_rad_s right_rear_wheel_rotational_speed_rad_s(::agrorob_msgs::msg::RobotState::_right_rear_wheel_rotational_speed_rad_s_type arg)
  {
    msg_.right_rear_wheel_rotational_speed_rad_s = std::move(arg);
    return Init_RobotState_left_rear_wheel_rotational_speed_rad_s(msg_);
  }

private:
  ::agrorob_msgs::msg::RobotState msg_;
};

class Init_RobotState_left_front_wheel_rotational_speed_rad_s
{
public:
  explicit Init_RobotState_left_front_wheel_rotational_speed_rad_s(::agrorob_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_right_rear_wheel_rotational_speed_rad_s left_front_wheel_rotational_speed_rad_s(::agrorob_msgs::msg::RobotState::_left_front_wheel_rotational_speed_rad_s_type arg)
  {
    msg_.left_front_wheel_rotational_speed_rad_s = std::move(arg);
    return Init_RobotState_right_rear_wheel_rotational_speed_rad_s(msg_);
  }

private:
  ::agrorob_msgs::msg::RobotState msg_;
};

class Init_RobotState_right_front_wheel_rotational_speed_rad_s
{
public:
  explicit Init_RobotState_right_front_wheel_rotational_speed_rad_s(::agrorob_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_left_front_wheel_rotational_speed_rad_s right_front_wheel_rotational_speed_rad_s(::agrorob_msgs::msg::RobotState::_right_front_wheel_rotational_speed_rad_s_type arg)
  {
    msg_.right_front_wheel_rotational_speed_rad_s = std::move(arg);
    return Init_RobotState_left_front_wheel_rotational_speed_rad_s(msg_);
  }

private:
  ::agrorob_msgs::msg::RobotState msg_;
};

class Init_RobotState_left_rear_wheel_turn_angle_rad
{
public:
  explicit Init_RobotState_left_rear_wheel_turn_angle_rad(::agrorob_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_right_front_wheel_rotational_speed_rad_s left_rear_wheel_turn_angle_rad(::agrorob_msgs::msg::RobotState::_left_rear_wheel_turn_angle_rad_type arg)
  {
    msg_.left_rear_wheel_turn_angle_rad = std::move(arg);
    return Init_RobotState_right_front_wheel_rotational_speed_rad_s(msg_);
  }

private:
  ::agrorob_msgs::msg::RobotState msg_;
};

class Init_RobotState_right_rear_wheel_turn_angle_rad
{
public:
  explicit Init_RobotState_right_rear_wheel_turn_angle_rad(::agrorob_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_left_rear_wheel_turn_angle_rad right_rear_wheel_turn_angle_rad(::agrorob_msgs::msg::RobotState::_right_rear_wheel_turn_angle_rad_type arg)
  {
    msg_.right_rear_wheel_turn_angle_rad = std::move(arg);
    return Init_RobotState_left_rear_wheel_turn_angle_rad(msg_);
  }

private:
  ::agrorob_msgs::msg::RobotState msg_;
};

class Init_RobotState_left_front_wheel_turn_angle_rad
{
public:
  explicit Init_RobotState_left_front_wheel_turn_angle_rad(::agrorob_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_right_rear_wheel_turn_angle_rad left_front_wheel_turn_angle_rad(::agrorob_msgs::msg::RobotState::_left_front_wheel_turn_angle_rad_type arg)
  {
    msg_.left_front_wheel_turn_angle_rad = std::move(arg);
    return Init_RobotState_right_rear_wheel_turn_angle_rad(msg_);
  }

private:
  ::agrorob_msgs::msg::RobotState msg_;
};

class Init_RobotState_right_front_wheel_turn_angle_rad
{
public:
  explicit Init_RobotState_right_front_wheel_turn_angle_rad(::agrorob_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_left_front_wheel_turn_angle_rad right_front_wheel_turn_angle_rad(::agrorob_msgs::msg::RobotState::_right_front_wheel_turn_angle_rad_type arg)
  {
    msg_.right_front_wheel_turn_angle_rad = std::move(arg);
    return Init_RobotState_left_front_wheel_turn_angle_rad(msg_);
  }

private:
  ::agrorob_msgs::msg::RobotState msg_;
};

class Init_RobotState_right_rear_wheel_encoder_imp
{
public:
  explicit Init_RobotState_right_rear_wheel_encoder_imp(::agrorob_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_right_front_wheel_turn_angle_rad right_rear_wheel_encoder_imp(::agrorob_msgs::msg::RobotState::_right_rear_wheel_encoder_imp_type arg)
  {
    msg_.right_rear_wheel_encoder_imp = std::move(arg);
    return Init_RobotState_right_front_wheel_turn_angle_rad(msg_);
  }

private:
  ::agrorob_msgs::msg::RobotState msg_;
};

class Init_RobotState_left_rear_wheel_encoder_imp
{
public:
  explicit Init_RobotState_left_rear_wheel_encoder_imp(::agrorob_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_right_rear_wheel_encoder_imp left_rear_wheel_encoder_imp(::agrorob_msgs::msg::RobotState::_left_rear_wheel_encoder_imp_type arg)
  {
    msg_.left_rear_wheel_encoder_imp = std::move(arg);
    return Init_RobotState_right_rear_wheel_encoder_imp(msg_);
  }

private:
  ::agrorob_msgs::msg::RobotState msg_;
};

class Init_RobotState_right_front_wheel_encoder_imp
{
public:
  explicit Init_RobotState_right_front_wheel_encoder_imp(::agrorob_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_left_rear_wheel_encoder_imp right_front_wheel_encoder_imp(::agrorob_msgs::msg::RobotState::_right_front_wheel_encoder_imp_type arg)
  {
    msg_.right_front_wheel_encoder_imp = std::move(arg);
    return Init_RobotState_left_rear_wheel_encoder_imp(msg_);
  }

private:
  ::agrorob_msgs::msg::RobotState msg_;
};

class Init_RobotState_left_front_wheel_encoder_imp
{
public:
  Init_RobotState_left_front_wheel_encoder_imp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotState_right_front_wheel_encoder_imp left_front_wheel_encoder_imp(::agrorob_msgs::msg::RobotState::_left_front_wheel_encoder_imp_type arg)
  {
    msg_.left_front_wheel_encoder_imp = std::move(arg);
    return Init_RobotState_right_front_wheel_encoder_imp(msg_);
  }

private:
  ::agrorob_msgs::msg::RobotState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::agrorob_msgs::msg::RobotState>()
{
  return agrorob_msgs::msg::builder::Init_RobotState_left_front_wheel_encoder_imp();
}

}  // namespace agrorob_msgs

#endif  // AGROROB_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
