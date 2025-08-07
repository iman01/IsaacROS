// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from agrorob_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef AGROROB_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
#define AGROROB_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RobotState in the package agrorob_msgs.
typedef struct agrorob_msgs__msg__RobotState
{
  /// ID31: rotating wheel ( imp / s) * 100; 1 revolution of the wheel equals 54 pulses
  double left_front_wheel_encoder_imp;
  /// ID31: rotating wheel ( imp / s) * 100; 1 revolution of the wheel equals 54 pulses
  double right_front_wheel_encoder_imp;
  /// ID31: rotating wheel ( imp / s) * 100; 1 revolution of the wheel equals 54 pulses
  double left_rear_wheel_encoder_imp;
  /// ID31: rotating wheel ( imp / s) * 100; 1 revolution of the wheel equals 54 pulses
  double right_rear_wheel_encoder_imp;
  double right_front_wheel_turn_angle_rad;
  double left_front_wheel_turn_angle_rad;
  double right_rear_wheel_turn_angle_rad;
  double left_rear_wheel_turn_angle_rad;
  /// left -90 degrees; max turn right 90 degrees
  int32_t right_front_wheel_rotational_speed_rad_s;
  /// ID32: rotational speed( rad/s );
  int32_t left_front_wheel_rotational_speed_rad_s;
  int32_t right_rear_wheel_rotational_speed_rad_s;
  int32_t left_rear_wheel_rotational_speed_rad_s;
} agrorob_msgs__msg__RobotState;

// Struct for a sequence of agrorob_msgs__msg__RobotState.
typedef struct agrorob_msgs__msg__RobotState__Sequence
{
  agrorob_msgs__msg__RobotState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} agrorob_msgs__msg__RobotState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AGROROB_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
