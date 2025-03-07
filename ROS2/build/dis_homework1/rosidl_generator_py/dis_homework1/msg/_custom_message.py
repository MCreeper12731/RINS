# generated from rosidl_generator_py/resource/_idl.py.em
# with input from dis_homework1:msg/CustomMessage.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CustomMessage(type):
    """Metaclass of message 'CustomMessage'."""

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
            module = import_type_support('dis_homework1')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'dis_homework1.msg.CustomMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__custom_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__custom_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__custom_message
            cls._TYPE_SUPPORT = module.type_support_msg__msg__custom_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__custom_message

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CustomMessage(metaclass=Metaclass_CustomMessage):
    """Message class 'CustomMessage'."""

    __slots__ = [
        '_field1',
        '_field2',
        '_field3',
    ]

    _fields_and_field_types = {
        'field1': 'string',
        'field2': 'uint8',
        'field3': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.field1 = kwargs.get('field1', str())
        self.field2 = kwargs.get('field2', int())
        self.field3 = kwargs.get('field3', bool())

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
        if self.field1 != other.field1:
            return False
        if self.field2 != other.field2:
            return False
        if self.field3 != other.field3:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def field1(self):
        """Message field 'field1'."""
        return self._field1

    @field1.setter
    def field1(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'field1' field must be of type 'str'"
        self._field1 = value

    @builtins.property
    def field2(self):
        """Message field 'field2'."""
        return self._field2

    @field2.setter
    def field2(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'field2' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'field2' field must be an unsigned integer in [0, 255]"
        self._field2 = value

    @builtins.property
    def field3(self):
        """Message field 'field3'."""
        return self._field3

    @field3.setter
    def field3(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'field3' field must be of type 'bool'"
        self._field3 = value
