// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from agrorob_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice
#include "agrorob_msgs/msg/detail/robot_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "agrorob_msgs/msg/detail/robot_state__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace agrorob_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_agrorob_msgs
cdr_serialize(
  const agrorob_msgs::msg::RobotState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: left_front_wheel_encoder_imp
  cdr << ros_message.left_front_wheel_encoder_imp;
  // Member: right_front_wheel_encoder_imp
  cdr << ros_message.right_front_wheel_encoder_imp;
  // Member: left_rear_wheel_encoder_imp
  cdr << ros_message.left_rear_wheel_encoder_imp;
  // Member: right_rear_wheel_encoder_imp
  cdr << ros_message.right_rear_wheel_encoder_imp;
  // Member: right_front_wheel_turn_angle_rad
  cdr << ros_message.right_front_wheel_turn_angle_rad;
  // Member: left_front_wheel_turn_angle_rad
  cdr << ros_message.left_front_wheel_turn_angle_rad;
  // Member: right_rear_wheel_turn_angle_rad
  cdr << ros_message.right_rear_wheel_turn_angle_rad;
  // Member: left_rear_wheel_turn_angle_rad
  cdr << ros_message.left_rear_wheel_turn_angle_rad;
  // Member: right_front_wheel_rotational_speed_rad_s
  cdr << ros_message.right_front_wheel_rotational_speed_rad_s;
  // Member: left_front_wheel_rotational_speed_rad_s
  cdr << ros_message.left_front_wheel_rotational_speed_rad_s;
  // Member: right_rear_wheel_rotational_speed_rad_s
  cdr << ros_message.right_rear_wheel_rotational_speed_rad_s;
  // Member: left_rear_wheel_rotational_speed_rad_s
  cdr << ros_message.left_rear_wheel_rotational_speed_rad_s;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_agrorob_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  agrorob_msgs::msg::RobotState & ros_message)
{
  // Member: left_front_wheel_encoder_imp
  cdr >> ros_message.left_front_wheel_encoder_imp;

  // Member: right_front_wheel_encoder_imp
  cdr >> ros_message.right_front_wheel_encoder_imp;

  // Member: left_rear_wheel_encoder_imp
  cdr >> ros_message.left_rear_wheel_encoder_imp;

  // Member: right_rear_wheel_encoder_imp
  cdr >> ros_message.right_rear_wheel_encoder_imp;

  // Member: right_front_wheel_turn_angle_rad
  cdr >> ros_message.right_front_wheel_turn_angle_rad;

  // Member: left_front_wheel_turn_angle_rad
  cdr >> ros_message.left_front_wheel_turn_angle_rad;

  // Member: right_rear_wheel_turn_angle_rad
  cdr >> ros_message.right_rear_wheel_turn_angle_rad;

  // Member: left_rear_wheel_turn_angle_rad
  cdr >> ros_message.left_rear_wheel_turn_angle_rad;

  // Member: right_front_wheel_rotational_speed_rad_s
  cdr >> ros_message.right_front_wheel_rotational_speed_rad_s;

  // Member: left_front_wheel_rotational_speed_rad_s
  cdr >> ros_message.left_front_wheel_rotational_speed_rad_s;

  // Member: right_rear_wheel_rotational_speed_rad_s
  cdr >> ros_message.right_rear_wheel_rotational_speed_rad_s;

  // Member: left_rear_wheel_rotational_speed_rad_s
  cdr >> ros_message.left_rear_wheel_rotational_speed_rad_s;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_agrorob_msgs
get_serialized_size(
  const agrorob_msgs::msg::RobotState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: left_front_wheel_encoder_imp
  {
    size_t item_size = sizeof(ros_message.left_front_wheel_encoder_imp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: right_front_wheel_encoder_imp
  {
    size_t item_size = sizeof(ros_message.right_front_wheel_encoder_imp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: left_rear_wheel_encoder_imp
  {
    size_t item_size = sizeof(ros_message.left_rear_wheel_encoder_imp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: right_rear_wheel_encoder_imp
  {
    size_t item_size = sizeof(ros_message.right_rear_wheel_encoder_imp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: right_front_wheel_turn_angle_rad
  {
    size_t item_size = sizeof(ros_message.right_front_wheel_turn_angle_rad);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: left_front_wheel_turn_angle_rad
  {
    size_t item_size = sizeof(ros_message.left_front_wheel_turn_angle_rad);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: right_rear_wheel_turn_angle_rad
  {
    size_t item_size = sizeof(ros_message.right_rear_wheel_turn_angle_rad);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: left_rear_wheel_turn_angle_rad
  {
    size_t item_size = sizeof(ros_message.left_rear_wheel_turn_angle_rad);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: right_front_wheel_rotational_speed_rad_s
  {
    size_t item_size = sizeof(ros_message.right_front_wheel_rotational_speed_rad_s);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: left_front_wheel_rotational_speed_rad_s
  {
    size_t item_size = sizeof(ros_message.left_front_wheel_rotational_speed_rad_s);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: right_rear_wheel_rotational_speed_rad_s
  {
    size_t item_size = sizeof(ros_message.right_rear_wheel_rotational_speed_rad_s);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: left_rear_wheel_rotational_speed_rad_s
  {
    size_t item_size = sizeof(ros_message.left_rear_wheel_rotational_speed_rad_s);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_agrorob_msgs
max_serialized_size_RobotState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: left_front_wheel_encoder_imp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: right_front_wheel_encoder_imp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: left_rear_wheel_encoder_imp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: right_rear_wheel_encoder_imp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: right_front_wheel_turn_angle_rad
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: left_front_wheel_turn_angle_rad
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: right_rear_wheel_turn_angle_rad
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: left_rear_wheel_turn_angle_rad
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: right_front_wheel_rotational_speed_rad_s
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: left_front_wheel_rotational_speed_rad_s
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: right_rear_wheel_rotational_speed_rad_s
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: left_rear_wheel_rotational_speed_rad_s
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = agrorob_msgs::msg::RobotState;
    is_plain =
      (
      offsetof(DataType, left_rear_wheel_rotational_speed_rad_s) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _RobotState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const agrorob_msgs::msg::RobotState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _RobotState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<agrorob_msgs::msg::RobotState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _RobotState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const agrorob_msgs::msg::RobotState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _RobotState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_RobotState(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _RobotState__callbacks = {
  "agrorob_msgs::msg",
  "RobotState",
  _RobotState__cdr_serialize,
  _RobotState__cdr_deserialize,
  _RobotState__get_serialized_size,
  _RobotState__max_serialized_size
};

static rosidl_message_type_support_t _RobotState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_RobotState__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace agrorob_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_agrorob_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<agrorob_msgs::msg::RobotState>()
{
  return &agrorob_msgs::msg::typesupport_fastrtps_cpp::_RobotState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, agrorob_msgs, msg, RobotState)() {
  return &agrorob_msgs::msg::typesupport_fastrtps_cpp::_RobotState__handle;
}

#ifdef __cplusplus
}
#endif
