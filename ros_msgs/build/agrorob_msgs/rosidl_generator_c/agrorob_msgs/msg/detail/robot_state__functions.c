// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from agrorob_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice
#include "agrorob_msgs/msg/detail/robot_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
agrorob_msgs__msg__RobotState__init(agrorob_msgs__msg__RobotState * msg)
{
  if (!msg) {
    return false;
  }
  // left_front_wheel_encoder_imp
  // right_front_wheel_encoder_imp
  // left_rear_wheel_encoder_imp
  // right_rear_wheel_encoder_imp
  // right_front_wheel_turn_angle_rad
  // left_front_wheel_turn_angle_rad
  // right_rear_wheel_turn_angle_rad
  // left_rear_wheel_turn_angle_rad
  // right_front_wheel_rotational_speed_rad_s
  // left_front_wheel_rotational_speed_rad_s
  // right_rear_wheel_rotational_speed_rad_s
  // left_rear_wheel_rotational_speed_rad_s
  return true;
}

void
agrorob_msgs__msg__RobotState__fini(agrorob_msgs__msg__RobotState * msg)
{
  if (!msg) {
    return;
  }
  // left_front_wheel_encoder_imp
  // right_front_wheel_encoder_imp
  // left_rear_wheel_encoder_imp
  // right_rear_wheel_encoder_imp
  // right_front_wheel_turn_angle_rad
  // left_front_wheel_turn_angle_rad
  // right_rear_wheel_turn_angle_rad
  // left_rear_wheel_turn_angle_rad
  // right_front_wheel_rotational_speed_rad_s
  // left_front_wheel_rotational_speed_rad_s
  // right_rear_wheel_rotational_speed_rad_s
  // left_rear_wheel_rotational_speed_rad_s
}

bool
agrorob_msgs__msg__RobotState__are_equal(const agrorob_msgs__msg__RobotState * lhs, const agrorob_msgs__msg__RobotState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // left_front_wheel_encoder_imp
  if (lhs->left_front_wheel_encoder_imp != rhs->left_front_wheel_encoder_imp) {
    return false;
  }
  // right_front_wheel_encoder_imp
  if (lhs->right_front_wheel_encoder_imp != rhs->right_front_wheel_encoder_imp) {
    return false;
  }
  // left_rear_wheel_encoder_imp
  if (lhs->left_rear_wheel_encoder_imp != rhs->left_rear_wheel_encoder_imp) {
    return false;
  }
  // right_rear_wheel_encoder_imp
  if (lhs->right_rear_wheel_encoder_imp != rhs->right_rear_wheel_encoder_imp) {
    return false;
  }
  // right_front_wheel_turn_angle_rad
  if (lhs->right_front_wheel_turn_angle_rad != rhs->right_front_wheel_turn_angle_rad) {
    return false;
  }
  // left_front_wheel_turn_angle_rad
  if (lhs->left_front_wheel_turn_angle_rad != rhs->left_front_wheel_turn_angle_rad) {
    return false;
  }
  // right_rear_wheel_turn_angle_rad
  if (lhs->right_rear_wheel_turn_angle_rad != rhs->right_rear_wheel_turn_angle_rad) {
    return false;
  }
  // left_rear_wheel_turn_angle_rad
  if (lhs->left_rear_wheel_turn_angle_rad != rhs->left_rear_wheel_turn_angle_rad) {
    return false;
  }
  // right_front_wheel_rotational_speed_rad_s
  if (lhs->right_front_wheel_rotational_speed_rad_s != rhs->right_front_wheel_rotational_speed_rad_s) {
    return false;
  }
  // left_front_wheel_rotational_speed_rad_s
  if (lhs->left_front_wheel_rotational_speed_rad_s != rhs->left_front_wheel_rotational_speed_rad_s) {
    return false;
  }
  // right_rear_wheel_rotational_speed_rad_s
  if (lhs->right_rear_wheel_rotational_speed_rad_s != rhs->right_rear_wheel_rotational_speed_rad_s) {
    return false;
  }
  // left_rear_wheel_rotational_speed_rad_s
  if (lhs->left_rear_wheel_rotational_speed_rad_s != rhs->left_rear_wheel_rotational_speed_rad_s) {
    return false;
  }
  return true;
}

bool
agrorob_msgs__msg__RobotState__copy(
  const agrorob_msgs__msg__RobotState * input,
  agrorob_msgs__msg__RobotState * output)
{
  if (!input || !output) {
    return false;
  }
  // left_front_wheel_encoder_imp
  output->left_front_wheel_encoder_imp = input->left_front_wheel_encoder_imp;
  // right_front_wheel_encoder_imp
  output->right_front_wheel_encoder_imp = input->right_front_wheel_encoder_imp;
  // left_rear_wheel_encoder_imp
  output->left_rear_wheel_encoder_imp = input->left_rear_wheel_encoder_imp;
  // right_rear_wheel_encoder_imp
  output->right_rear_wheel_encoder_imp = input->right_rear_wheel_encoder_imp;
  // right_front_wheel_turn_angle_rad
  output->right_front_wheel_turn_angle_rad = input->right_front_wheel_turn_angle_rad;
  // left_front_wheel_turn_angle_rad
  output->left_front_wheel_turn_angle_rad = input->left_front_wheel_turn_angle_rad;
  // right_rear_wheel_turn_angle_rad
  output->right_rear_wheel_turn_angle_rad = input->right_rear_wheel_turn_angle_rad;
  // left_rear_wheel_turn_angle_rad
  output->left_rear_wheel_turn_angle_rad = input->left_rear_wheel_turn_angle_rad;
  // right_front_wheel_rotational_speed_rad_s
  output->right_front_wheel_rotational_speed_rad_s = input->right_front_wheel_rotational_speed_rad_s;
  // left_front_wheel_rotational_speed_rad_s
  output->left_front_wheel_rotational_speed_rad_s = input->left_front_wheel_rotational_speed_rad_s;
  // right_rear_wheel_rotational_speed_rad_s
  output->right_rear_wheel_rotational_speed_rad_s = input->right_rear_wheel_rotational_speed_rad_s;
  // left_rear_wheel_rotational_speed_rad_s
  output->left_rear_wheel_rotational_speed_rad_s = input->left_rear_wheel_rotational_speed_rad_s;
  return true;
}

agrorob_msgs__msg__RobotState *
agrorob_msgs__msg__RobotState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  agrorob_msgs__msg__RobotState * msg = (agrorob_msgs__msg__RobotState *)allocator.allocate(sizeof(agrorob_msgs__msg__RobotState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(agrorob_msgs__msg__RobotState));
  bool success = agrorob_msgs__msg__RobotState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
agrorob_msgs__msg__RobotState__destroy(agrorob_msgs__msg__RobotState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    agrorob_msgs__msg__RobotState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
agrorob_msgs__msg__RobotState__Sequence__init(agrorob_msgs__msg__RobotState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  agrorob_msgs__msg__RobotState * data = NULL;

  if (size) {
    data = (agrorob_msgs__msg__RobotState *)allocator.zero_allocate(size, sizeof(agrorob_msgs__msg__RobotState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = agrorob_msgs__msg__RobotState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        agrorob_msgs__msg__RobotState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
agrorob_msgs__msg__RobotState__Sequence__fini(agrorob_msgs__msg__RobotState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      agrorob_msgs__msg__RobotState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

agrorob_msgs__msg__RobotState__Sequence *
agrorob_msgs__msg__RobotState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  agrorob_msgs__msg__RobotState__Sequence * array = (agrorob_msgs__msg__RobotState__Sequence *)allocator.allocate(sizeof(agrorob_msgs__msg__RobotState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = agrorob_msgs__msg__RobotState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
agrorob_msgs__msg__RobotState__Sequence__destroy(agrorob_msgs__msg__RobotState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    agrorob_msgs__msg__RobotState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
agrorob_msgs__msg__RobotState__Sequence__are_equal(const agrorob_msgs__msg__RobotState__Sequence * lhs, const agrorob_msgs__msg__RobotState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!agrorob_msgs__msg__RobotState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
agrorob_msgs__msg__RobotState__Sequence__copy(
  const agrorob_msgs__msg__RobotState__Sequence * input,
  agrorob_msgs__msg__RobotState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(agrorob_msgs__msg__RobotState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    agrorob_msgs__msg__RobotState * data =
      (agrorob_msgs__msg__RobotState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!agrorob_msgs__msg__RobotState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          agrorob_msgs__msg__RobotState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!agrorob_msgs__msg__RobotState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
