// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from agrorob_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef AGROROB_MSGS__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
#define AGROROB_MSGS__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "agrorob_msgs/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace agrorob_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotState & msg,
  std::ostream & out)
{
  out << "{";
  // member: left_front_wheel_encoder_imp
  {
    out << "left_front_wheel_encoder_imp: ";
    rosidl_generator_traits::value_to_yaml(msg.left_front_wheel_encoder_imp, out);
    out << ", ";
  }

  // member: right_front_wheel_encoder_imp
  {
    out << "right_front_wheel_encoder_imp: ";
    rosidl_generator_traits::value_to_yaml(msg.right_front_wheel_encoder_imp, out);
    out << ", ";
  }

  // member: left_rear_wheel_encoder_imp
  {
    out << "left_rear_wheel_encoder_imp: ";
    rosidl_generator_traits::value_to_yaml(msg.left_rear_wheel_encoder_imp, out);
    out << ", ";
  }

  // member: right_rear_wheel_encoder_imp
  {
    out << "right_rear_wheel_encoder_imp: ";
    rosidl_generator_traits::value_to_yaml(msg.right_rear_wheel_encoder_imp, out);
    out << ", ";
  }

  // member: right_front_wheel_turn_angle_rad
  {
    out << "right_front_wheel_turn_angle_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.right_front_wheel_turn_angle_rad, out);
    out << ", ";
  }

  // member: left_front_wheel_turn_angle_rad
  {
    out << "left_front_wheel_turn_angle_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.left_front_wheel_turn_angle_rad, out);
    out << ", ";
  }

  // member: right_rear_wheel_turn_angle_rad
  {
    out << "right_rear_wheel_turn_angle_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.right_rear_wheel_turn_angle_rad, out);
    out << ", ";
  }

  // member: left_rear_wheel_turn_angle_rad
  {
    out << "left_rear_wheel_turn_angle_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.left_rear_wheel_turn_angle_rad, out);
    out << ", ";
  }

  // member: right_front_wheel_rotational_speed_rad_s
  {
    out << "right_front_wheel_rotational_speed_rad_s: ";
    rosidl_generator_traits::value_to_yaml(msg.right_front_wheel_rotational_speed_rad_s, out);
    out << ", ";
  }

  // member: left_front_wheel_rotational_speed_rad_s
  {
    out << "left_front_wheel_rotational_speed_rad_s: ";
    rosidl_generator_traits::value_to_yaml(msg.left_front_wheel_rotational_speed_rad_s, out);
    out << ", ";
  }

  // member: right_rear_wheel_rotational_speed_rad_s
  {
    out << "right_rear_wheel_rotational_speed_rad_s: ";
    rosidl_generator_traits::value_to_yaml(msg.right_rear_wheel_rotational_speed_rad_s, out);
    out << ", ";
  }

  // member: left_rear_wheel_rotational_speed_rad_s
  {
    out << "left_rear_wheel_rotational_speed_rad_s: ";
    rosidl_generator_traits::value_to_yaml(msg.left_rear_wheel_rotational_speed_rad_s, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: left_front_wheel_encoder_imp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_front_wheel_encoder_imp: ";
    rosidl_generator_traits::value_to_yaml(msg.left_front_wheel_encoder_imp, out);
    out << "\n";
  }

  // member: right_front_wheel_encoder_imp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_front_wheel_encoder_imp: ";
    rosidl_generator_traits::value_to_yaml(msg.right_front_wheel_encoder_imp, out);
    out << "\n";
  }

  // member: left_rear_wheel_encoder_imp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_rear_wheel_encoder_imp: ";
    rosidl_generator_traits::value_to_yaml(msg.left_rear_wheel_encoder_imp, out);
    out << "\n";
  }

  // member: right_rear_wheel_encoder_imp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_rear_wheel_encoder_imp: ";
    rosidl_generator_traits::value_to_yaml(msg.right_rear_wheel_encoder_imp, out);
    out << "\n";
  }

  // member: right_front_wheel_turn_angle_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_front_wheel_turn_angle_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.right_front_wheel_turn_angle_rad, out);
    out << "\n";
  }

  // member: left_front_wheel_turn_angle_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_front_wheel_turn_angle_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.left_front_wheel_turn_angle_rad, out);
    out << "\n";
  }

  // member: right_rear_wheel_turn_angle_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_rear_wheel_turn_angle_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.right_rear_wheel_turn_angle_rad, out);
    out << "\n";
  }

  // member: left_rear_wheel_turn_angle_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_rear_wheel_turn_angle_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.left_rear_wheel_turn_angle_rad, out);
    out << "\n";
  }

  // member: right_front_wheel_rotational_speed_rad_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_front_wheel_rotational_speed_rad_s: ";
    rosidl_generator_traits::value_to_yaml(msg.right_front_wheel_rotational_speed_rad_s, out);
    out << "\n";
  }

  // member: left_front_wheel_rotational_speed_rad_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_front_wheel_rotational_speed_rad_s: ";
    rosidl_generator_traits::value_to_yaml(msg.left_front_wheel_rotational_speed_rad_s, out);
    out << "\n";
  }

  // member: right_rear_wheel_rotational_speed_rad_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_rear_wheel_rotational_speed_rad_s: ";
    rosidl_generator_traits::value_to_yaml(msg.right_rear_wheel_rotational_speed_rad_s, out);
    out << "\n";
  }

  // member: left_rear_wheel_rotational_speed_rad_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_rear_wheel_rotational_speed_rad_s: ";
    rosidl_generator_traits::value_to_yaml(msg.left_rear_wheel_rotational_speed_rad_s, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace agrorob_msgs

namespace rosidl_generator_traits
{

[[deprecated("use agrorob_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const agrorob_msgs::msg::RobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  agrorob_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use agrorob_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const agrorob_msgs::msg::RobotState & msg)
{
  return agrorob_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<agrorob_msgs::msg::RobotState>()
{
  return "agrorob_msgs::msg::RobotState";
}

template<>
inline const char * name<agrorob_msgs::msg::RobotState>()
{
  return "agrorob_msgs/msg/RobotState";
}

template<>
struct has_fixed_size<agrorob_msgs::msg::RobotState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<agrorob_msgs::msg::RobotState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<agrorob_msgs::msg::RobotState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AGROROB_MSGS__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
