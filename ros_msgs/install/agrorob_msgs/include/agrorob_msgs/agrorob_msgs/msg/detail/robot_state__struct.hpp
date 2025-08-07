// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from agrorob_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef AGROROB_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_HPP_
#define AGROROB_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__agrorob_msgs__msg__RobotState __attribute__((deprecated))
#else
# define DEPRECATED__agrorob_msgs__msg__RobotState __declspec(deprecated)
#endif

namespace agrorob_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotState_
{
  using Type = RobotState_<ContainerAllocator>;

  explicit RobotState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_front_wheel_encoder_imp = 0.0;
      this->right_front_wheel_encoder_imp = 0.0;
      this->left_rear_wheel_encoder_imp = 0.0;
      this->right_rear_wheel_encoder_imp = 0.0;
      this->right_front_wheel_turn_angle_rad = 0.0;
      this->left_front_wheel_turn_angle_rad = 0.0;
      this->right_rear_wheel_turn_angle_rad = 0.0;
      this->left_rear_wheel_turn_angle_rad = 0.0;
      this->right_front_wheel_rotational_speed_rad_s = 0l;
      this->left_front_wheel_rotational_speed_rad_s = 0l;
      this->right_rear_wheel_rotational_speed_rad_s = 0l;
      this->left_rear_wheel_rotational_speed_rad_s = 0l;
    }
  }

  explicit RobotState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_front_wheel_encoder_imp = 0.0;
      this->right_front_wheel_encoder_imp = 0.0;
      this->left_rear_wheel_encoder_imp = 0.0;
      this->right_rear_wheel_encoder_imp = 0.0;
      this->right_front_wheel_turn_angle_rad = 0.0;
      this->left_front_wheel_turn_angle_rad = 0.0;
      this->right_rear_wheel_turn_angle_rad = 0.0;
      this->left_rear_wheel_turn_angle_rad = 0.0;
      this->right_front_wheel_rotational_speed_rad_s = 0l;
      this->left_front_wheel_rotational_speed_rad_s = 0l;
      this->right_rear_wheel_rotational_speed_rad_s = 0l;
      this->left_rear_wheel_rotational_speed_rad_s = 0l;
    }
  }

  // field types and members
  using _left_front_wheel_encoder_imp_type =
    double;
  _left_front_wheel_encoder_imp_type left_front_wheel_encoder_imp;
  using _right_front_wheel_encoder_imp_type =
    double;
  _right_front_wheel_encoder_imp_type right_front_wheel_encoder_imp;
  using _left_rear_wheel_encoder_imp_type =
    double;
  _left_rear_wheel_encoder_imp_type left_rear_wheel_encoder_imp;
  using _right_rear_wheel_encoder_imp_type =
    double;
  _right_rear_wheel_encoder_imp_type right_rear_wheel_encoder_imp;
  using _right_front_wheel_turn_angle_rad_type =
    double;
  _right_front_wheel_turn_angle_rad_type right_front_wheel_turn_angle_rad;
  using _left_front_wheel_turn_angle_rad_type =
    double;
  _left_front_wheel_turn_angle_rad_type left_front_wheel_turn_angle_rad;
  using _right_rear_wheel_turn_angle_rad_type =
    double;
  _right_rear_wheel_turn_angle_rad_type right_rear_wheel_turn_angle_rad;
  using _left_rear_wheel_turn_angle_rad_type =
    double;
  _left_rear_wheel_turn_angle_rad_type left_rear_wheel_turn_angle_rad;
  using _right_front_wheel_rotational_speed_rad_s_type =
    int32_t;
  _right_front_wheel_rotational_speed_rad_s_type right_front_wheel_rotational_speed_rad_s;
  using _left_front_wheel_rotational_speed_rad_s_type =
    int32_t;
  _left_front_wheel_rotational_speed_rad_s_type left_front_wheel_rotational_speed_rad_s;
  using _right_rear_wheel_rotational_speed_rad_s_type =
    int32_t;
  _right_rear_wheel_rotational_speed_rad_s_type right_rear_wheel_rotational_speed_rad_s;
  using _left_rear_wheel_rotational_speed_rad_s_type =
    int32_t;
  _left_rear_wheel_rotational_speed_rad_s_type left_rear_wheel_rotational_speed_rad_s;

  // setters for named parameter idiom
  Type & set__left_front_wheel_encoder_imp(
    const double & _arg)
  {
    this->left_front_wheel_encoder_imp = _arg;
    return *this;
  }
  Type & set__right_front_wheel_encoder_imp(
    const double & _arg)
  {
    this->right_front_wheel_encoder_imp = _arg;
    return *this;
  }
  Type & set__left_rear_wheel_encoder_imp(
    const double & _arg)
  {
    this->left_rear_wheel_encoder_imp = _arg;
    return *this;
  }
  Type & set__right_rear_wheel_encoder_imp(
    const double & _arg)
  {
    this->right_rear_wheel_encoder_imp = _arg;
    return *this;
  }
  Type & set__right_front_wheel_turn_angle_rad(
    const double & _arg)
  {
    this->right_front_wheel_turn_angle_rad = _arg;
    return *this;
  }
  Type & set__left_front_wheel_turn_angle_rad(
    const double & _arg)
  {
    this->left_front_wheel_turn_angle_rad = _arg;
    return *this;
  }
  Type & set__right_rear_wheel_turn_angle_rad(
    const double & _arg)
  {
    this->right_rear_wheel_turn_angle_rad = _arg;
    return *this;
  }
  Type & set__left_rear_wheel_turn_angle_rad(
    const double & _arg)
  {
    this->left_rear_wheel_turn_angle_rad = _arg;
    return *this;
  }
  Type & set__right_front_wheel_rotational_speed_rad_s(
    const int32_t & _arg)
  {
    this->right_front_wheel_rotational_speed_rad_s = _arg;
    return *this;
  }
  Type & set__left_front_wheel_rotational_speed_rad_s(
    const int32_t & _arg)
  {
    this->left_front_wheel_rotational_speed_rad_s = _arg;
    return *this;
  }
  Type & set__right_rear_wheel_rotational_speed_rad_s(
    const int32_t & _arg)
  {
    this->right_rear_wheel_rotational_speed_rad_s = _arg;
    return *this;
  }
  Type & set__left_rear_wheel_rotational_speed_rad_s(
    const int32_t & _arg)
  {
    this->left_rear_wheel_rotational_speed_rad_s = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    agrorob_msgs::msg::RobotState_<ContainerAllocator> *;
  using ConstRawPtr =
    const agrorob_msgs::msg::RobotState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<agrorob_msgs::msg::RobotState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<agrorob_msgs::msg::RobotState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      agrorob_msgs::msg::RobotState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<agrorob_msgs::msg::RobotState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      agrorob_msgs::msg::RobotState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<agrorob_msgs::msg::RobotState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<agrorob_msgs::msg::RobotState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<agrorob_msgs::msg::RobotState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__agrorob_msgs__msg__RobotState
    std::shared_ptr<agrorob_msgs::msg::RobotState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__agrorob_msgs__msg__RobotState
    std::shared_ptr<agrorob_msgs::msg::RobotState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotState_ & other) const
  {
    if (this->left_front_wheel_encoder_imp != other.left_front_wheel_encoder_imp) {
      return false;
    }
    if (this->right_front_wheel_encoder_imp != other.right_front_wheel_encoder_imp) {
      return false;
    }
    if (this->left_rear_wheel_encoder_imp != other.left_rear_wheel_encoder_imp) {
      return false;
    }
    if (this->right_rear_wheel_encoder_imp != other.right_rear_wheel_encoder_imp) {
      return false;
    }
    if (this->right_front_wheel_turn_angle_rad != other.right_front_wheel_turn_angle_rad) {
      return false;
    }
    if (this->left_front_wheel_turn_angle_rad != other.left_front_wheel_turn_angle_rad) {
      return false;
    }
    if (this->right_rear_wheel_turn_angle_rad != other.right_rear_wheel_turn_angle_rad) {
      return false;
    }
    if (this->left_rear_wheel_turn_angle_rad != other.left_rear_wheel_turn_angle_rad) {
      return false;
    }
    if (this->right_front_wheel_rotational_speed_rad_s != other.right_front_wheel_rotational_speed_rad_s) {
      return false;
    }
    if (this->left_front_wheel_rotational_speed_rad_s != other.left_front_wheel_rotational_speed_rad_s) {
      return false;
    }
    if (this->right_rear_wheel_rotational_speed_rad_s != other.right_rear_wheel_rotational_speed_rad_s) {
      return false;
    }
    if (this->left_rear_wheel_rotational_speed_rad_s != other.left_rear_wheel_rotational_speed_rad_s) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotState_

// alias to use template instance with default allocator
using RobotState =
  agrorob_msgs::msg::RobotState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace agrorob_msgs

#endif  // AGROROB_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_HPP_
