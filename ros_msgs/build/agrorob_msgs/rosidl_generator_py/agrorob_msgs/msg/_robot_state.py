# generated from rosidl_generator_py/resource/_idl.py.em
# with input from agrorob_msgs:msg/RobotState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RobotState(type):
    """Metaclass of message 'RobotState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('agrorob_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'agrorob_msgs.msg.RobotState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__robot_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__robot_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__robot_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__robot_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__robot_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RobotState(metaclass=Metaclass_RobotState):
    """Message class 'RobotState'."""

    __slots__ = [
        '_left_front_wheel_encoder_imp',
        '_right_front_wheel_encoder_imp',
        '_left_rear_wheel_encoder_imp',
        '_right_rear_wheel_encoder_imp',
        '_right_front_wheel_turn_angle_rad',
        '_left_front_wheel_turn_angle_rad',
        '_right_rear_wheel_turn_angle_rad',
        '_left_rear_wheel_turn_angle_rad',
        '_right_front_wheel_rotational_speed_rad_s',
        '_left_front_wheel_rotational_speed_rad_s',
        '_right_rear_wheel_rotational_speed_rad_s',
        '_left_rear_wheel_rotational_speed_rad_s',
    ]

    _fields_and_field_types = {
        'left_front_wheel_encoder_imp': 'double',
        'right_front_wheel_encoder_imp': 'double',
        'left_rear_wheel_encoder_imp': 'double',
        'right_rear_wheel_encoder_imp': 'double',
        'right_front_wheel_turn_angle_rad': 'double',
        'left_front_wheel_turn_angle_rad': 'double',
        'right_rear_wheel_turn_angle_rad': 'double',
        'left_rear_wheel_turn_angle_rad': 'double',
        'right_front_wheel_rotational_speed_rad_s': 'int32',
        'left_front_wheel_rotational_speed_rad_s': 'int32',
        'right_rear_wheel_rotational_speed_rad_s': 'int32',
        'left_rear_wheel_rotational_speed_rad_s': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.left_front_wheel_encoder_imp = kwargs.get('left_front_wheel_encoder_imp', float())
        self.right_front_wheel_encoder_imp = kwargs.get('right_front_wheel_encoder_imp', float())
        self.left_rear_wheel_encoder_imp = kwargs.get('left_rear_wheel_encoder_imp', float())
        self.right_rear_wheel_encoder_imp = kwargs.get('right_rear_wheel_encoder_imp', float())
        self.right_front_wheel_turn_angle_rad = kwargs.get('right_front_wheel_turn_angle_rad', float())
        self.left_front_wheel_turn_angle_rad = kwargs.get('left_front_wheel_turn_angle_rad', float())
        self.right_rear_wheel_turn_angle_rad = kwargs.get('right_rear_wheel_turn_angle_rad', float())
        self.left_rear_wheel_turn_angle_rad = kwargs.get('left_rear_wheel_turn_angle_rad', float())
        self.right_front_wheel_rotational_speed_rad_s = kwargs.get('right_front_wheel_rotational_speed_rad_s', int())
        self.left_front_wheel_rotational_speed_rad_s = kwargs.get('left_front_wheel_rotational_speed_rad_s', int())
        self.right_rear_wheel_rotational_speed_rad_s = kwargs.get('right_rear_wheel_rotational_speed_rad_s', int())
        self.left_rear_wheel_rotational_speed_rad_s = kwargs.get('left_rear_wheel_rotational_speed_rad_s', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.left_front_wheel_encoder_imp != other.left_front_wheel_encoder_imp:
            return False
        if self.right_front_wheel_encoder_imp != other.right_front_wheel_encoder_imp:
            return False
        if self.left_rear_wheel_encoder_imp != other.left_rear_wheel_encoder_imp:
            return False
        if self.right_rear_wheel_encoder_imp != other.right_rear_wheel_encoder_imp:
            return False
        if self.right_front_wheel_turn_angle_rad != other.right_front_wheel_turn_angle_rad:
            return False
        if self.left_front_wheel_turn_angle_rad != other.left_front_wheel_turn_angle_rad:
            return False
        if self.right_rear_wheel_turn_angle_rad != other.right_rear_wheel_turn_angle_rad:
            return False
        if self.left_rear_wheel_turn_angle_rad != other.left_rear_wheel_turn_angle_rad:
            return False
        if self.right_front_wheel_rotational_speed_rad_s != other.right_front_wheel_rotational_speed_rad_s:
            return False
        if self.left_front_wheel_rotational_speed_rad_s != other.left_front_wheel_rotational_speed_rad_s:
            return False
        if self.right_rear_wheel_rotational_speed_rad_s != other.right_rear_wheel_rotational_speed_rad_s:
            return False
        if self.left_rear_wheel_rotational_speed_rad_s != other.left_rear_wheel_rotational_speed_rad_s:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def left_front_wheel_encoder_imp(self):
        """Message field 'left_front_wheel_encoder_imp'."""
        return self._left_front_wheel_encoder_imp

    @left_front_wheel_encoder_imp.setter
    def left_front_wheel_encoder_imp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'left_front_wheel_encoder_imp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'left_front_wheel_encoder_imp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._left_front_wheel_encoder_imp = value

    @builtins.property
    def right_front_wheel_encoder_imp(self):
        """Message field 'right_front_wheel_encoder_imp'."""
        return self._right_front_wheel_encoder_imp

    @right_front_wheel_encoder_imp.setter
    def right_front_wheel_encoder_imp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'right_front_wheel_encoder_imp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'right_front_wheel_encoder_imp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._right_front_wheel_encoder_imp = value

    @builtins.property
    def left_rear_wheel_encoder_imp(self):
        """Message field 'left_rear_wheel_encoder_imp'."""
        return self._left_rear_wheel_encoder_imp

    @left_rear_wheel_encoder_imp.setter
    def left_rear_wheel_encoder_imp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'left_rear_wheel_encoder_imp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'left_rear_wheel_encoder_imp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._left_rear_wheel_encoder_imp = value

    @builtins.property
    def right_rear_wheel_encoder_imp(self):
        """Message field 'right_rear_wheel_encoder_imp'."""
        return self._right_rear_wheel_encoder_imp

    @right_rear_wheel_encoder_imp.setter
    def right_rear_wheel_encoder_imp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'right_rear_wheel_encoder_imp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'right_rear_wheel_encoder_imp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._right_rear_wheel_encoder_imp = value

    @builtins.property
    def right_front_wheel_turn_angle_rad(self):
        """Message field 'right_front_wheel_turn_angle_rad'."""
        return self._right_front_wheel_turn_angle_rad

    @right_front_wheel_turn_angle_rad.setter
    def right_front_wheel_turn_angle_rad(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'right_front_wheel_turn_angle_rad' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'right_front_wheel_turn_angle_rad' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._right_front_wheel_turn_angle_rad = value

    @builtins.property
    def left_front_wheel_turn_angle_rad(self):
        """Message field 'left_front_wheel_turn_angle_rad'."""
        return self._left_front_wheel_turn_angle_rad

    @left_front_wheel_turn_angle_rad.setter
    def left_front_wheel_turn_angle_rad(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'left_front_wheel_turn_angle_rad' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'left_front_wheel_turn_angle_rad' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._left_front_wheel_turn_angle_rad = value

    @builtins.property
    def right_rear_wheel_turn_angle_rad(self):
        """Message field 'right_rear_wheel_turn_angle_rad'."""
        return self._right_rear_wheel_turn_angle_rad

    @right_rear_wheel_turn_angle_rad.setter
    def right_rear_wheel_turn_angle_rad(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'right_rear_wheel_turn_angle_rad' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'right_rear_wheel_turn_angle_rad' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._right_rear_wheel_turn_angle_rad = value

    @builtins.property
    def left_rear_wheel_turn_angle_rad(self):
        """Message field 'left_rear_wheel_turn_angle_rad'."""
        return self._left_rear_wheel_turn_angle_rad

    @left_rear_wheel_turn_angle_rad.setter
    def left_rear_wheel_turn_angle_rad(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'left_rear_wheel_turn_angle_rad' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'left_rear_wheel_turn_angle_rad' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._left_rear_wheel_turn_angle_rad = value

    @builtins.property
    def right_front_wheel_rotational_speed_rad_s(self):
        """Message field 'right_front_wheel_rotational_speed_rad_s'."""
        return self._right_front_wheel_rotational_speed_rad_s

    @right_front_wheel_rotational_speed_rad_s.setter
    def right_front_wheel_rotational_speed_rad_s(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'right_front_wheel_rotational_speed_rad_s' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'right_front_wheel_rotational_speed_rad_s' field must be an integer in [-2147483648, 2147483647]"
        self._right_front_wheel_rotational_speed_rad_s = value

    @builtins.property
    def left_front_wheel_rotational_speed_rad_s(self):
        """Message field 'left_front_wheel_rotational_speed_rad_s'."""
        return self._left_front_wheel_rotational_speed_rad_s

    @left_front_wheel_rotational_speed_rad_s.setter
    def left_front_wheel_rotational_speed_rad_s(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'left_front_wheel_rotational_speed_rad_s' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'left_front_wheel_rotational_speed_rad_s' field must be an integer in [-2147483648, 2147483647]"
        self._left_front_wheel_rotational_speed_rad_s = value

    @builtins.property
    def right_rear_wheel_rotational_speed_rad_s(self):
        """Message field 'right_rear_wheel_rotational_speed_rad_s'."""
        return self._right_rear_wheel_rotational_speed_rad_s

    @right_rear_wheel_rotational_speed_rad_s.setter
    def right_rear_wheel_rotational_speed_rad_s(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'right_rear_wheel_rotational_speed_rad_s' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'right_rear_wheel_rotational_speed_rad_s' field must be an integer in [-2147483648, 2147483647]"
        self._right_rear_wheel_rotational_speed_rad_s = value

    @builtins.property
    def left_rear_wheel_rotational_speed_rad_s(self):
        """Message field 'left_rear_wheel_rotational_speed_rad_s'."""
        return self._left_rear_wheel_rotational_speed_rad_s

    @left_rear_wheel_rotational_speed_rad_s.setter
    def left_rear_wheel_rotational_speed_rad_s(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'left_rear_wheel_rotational_speed_rad_s' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'left_rear_wheel_rotational_speed_rad_s' field must be an integer in [-2147483648, 2147483647]"
        self._left_rear_wheel_rotational_speed_rad_s = value
