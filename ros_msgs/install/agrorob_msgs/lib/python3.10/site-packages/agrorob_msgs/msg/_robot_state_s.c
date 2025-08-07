// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from agrorob_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "agrorob_msgs/msg/detail/robot_state__struct.h"
#include "agrorob_msgs/msg/detail/robot_state__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool agrorob_msgs__msg__robot_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[41];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("agrorob_msgs.msg._robot_state.RobotState", full_classname_dest, 40) == 0);
  }
  agrorob_msgs__msg__RobotState * ros_message = _ros_message;
  {  // left_front_wheel_encoder_imp
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_front_wheel_encoder_imp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->left_front_wheel_encoder_imp = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // right_front_wheel_encoder_imp
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_front_wheel_encoder_imp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->right_front_wheel_encoder_imp = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // left_rear_wheel_encoder_imp
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_rear_wheel_encoder_imp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->left_rear_wheel_encoder_imp = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // right_rear_wheel_encoder_imp
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_rear_wheel_encoder_imp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->right_rear_wheel_encoder_imp = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // right_front_wheel_turn_angle_rad
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_front_wheel_turn_angle_rad");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->right_front_wheel_turn_angle_rad = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // left_front_wheel_turn_angle_rad
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_front_wheel_turn_angle_rad");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->left_front_wheel_turn_angle_rad = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // right_rear_wheel_turn_angle_rad
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_rear_wheel_turn_angle_rad");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->right_rear_wheel_turn_angle_rad = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // left_rear_wheel_turn_angle_rad
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_rear_wheel_turn_angle_rad");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->left_rear_wheel_turn_angle_rad = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // right_front_wheel_rotational_speed_rad_s
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_front_wheel_rotational_speed_rad_s");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->right_front_wheel_rotational_speed_rad_s = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // left_front_wheel_rotational_speed_rad_s
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_front_wheel_rotational_speed_rad_s");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->left_front_wheel_rotational_speed_rad_s = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // right_rear_wheel_rotational_speed_rad_s
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_rear_wheel_rotational_speed_rad_s");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->right_rear_wheel_rotational_speed_rad_s = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // left_rear_wheel_rotational_speed_rad_s
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_rear_wheel_rotational_speed_rad_s");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->left_rear_wheel_rotational_speed_rad_s = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * agrorob_msgs__msg__robot_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RobotState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("agrorob_msgs.msg._robot_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RobotState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  agrorob_msgs__msg__RobotState * ros_message = (agrorob_msgs__msg__RobotState *)raw_ros_message;
  {  // left_front_wheel_encoder_imp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->left_front_wheel_encoder_imp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_front_wheel_encoder_imp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_front_wheel_encoder_imp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->right_front_wheel_encoder_imp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_front_wheel_encoder_imp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_rear_wheel_encoder_imp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->left_rear_wheel_encoder_imp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_rear_wheel_encoder_imp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_rear_wheel_encoder_imp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->right_rear_wheel_encoder_imp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_rear_wheel_encoder_imp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_front_wheel_turn_angle_rad
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->right_front_wheel_turn_angle_rad);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_front_wheel_turn_angle_rad", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_front_wheel_turn_angle_rad
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->left_front_wheel_turn_angle_rad);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_front_wheel_turn_angle_rad", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_rear_wheel_turn_angle_rad
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->right_rear_wheel_turn_angle_rad);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_rear_wheel_turn_angle_rad", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_rear_wheel_turn_angle_rad
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->left_rear_wheel_turn_angle_rad);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_rear_wheel_turn_angle_rad", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_front_wheel_rotational_speed_rad_s
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->right_front_wheel_rotational_speed_rad_s);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_front_wheel_rotational_speed_rad_s", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_front_wheel_rotational_speed_rad_s
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->left_front_wheel_rotational_speed_rad_s);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_front_wheel_rotational_speed_rad_s", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_rear_wheel_rotational_speed_rad_s
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->right_rear_wheel_rotational_speed_rad_s);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_rear_wheel_rotational_speed_rad_s", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_rear_wheel_rotational_speed_rad_s
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->left_rear_wheel_rotational_speed_rad_s);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_rear_wheel_rotational_speed_rad_s", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
