# generated from rosidl_generator_py/resource/_idl.py.em
# with input from dis_homework1:srv/CustomService.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

# Member 'field2'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CustomService_Request(type):
    """Metaclass of message 'CustomService_Request'."""

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
                'dis_homework1.srv.CustomService_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__custom_service__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__custom_service__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__custom_service__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__custom_service__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__custom_service__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CustomService_Request(metaclass=Metaclass_CustomService_Request):
    """Message class 'CustomService_Request'."""

    __slots__ = [
        '_field1',
        '_field2',
    ]

    _fields_and_field_types = {
        'field1': 'string',
        'field2': 'int64[10]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int64'), 10),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.field1 = kwargs.get('field1', str())
        if 'field2' not in kwargs:
            self.field2 = numpy.zeros(10, dtype=numpy.int64)
        else:
            self.field2 = numpy.array(kwargs.get('field2'), dtype=numpy.int64)
            assert self.field2.shape == (10, )

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
        if all(self.field2 != other.field2):
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
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int64, \
                "The 'field2' numpy.ndarray() must have the dtype of 'numpy.int64'"
            assert value.size == 10, \
                "The 'field2' numpy.ndarray() must have a size of 10"
            self._field2 = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 10 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -9223372036854775808 and val < 9223372036854775808 for val in value)), \
                "The 'field2' field must be a set or sequence with length 10 and each value of type 'int' and each integer in [-9223372036854775808, 9223372036854775807]"
        self._field2 = numpy.array(value, dtype=numpy.int64)


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_CustomService_Response(type):
    """Metaclass of message 'CustomService_Response'."""

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
                'dis_homework1.srv.CustomService_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__custom_service__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__custom_service__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__custom_service__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__custom_service__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__custom_service__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CustomService_Response(metaclass=Metaclass_CustomService_Response):
    """Message class 'CustomService_Response'."""

    __slots__ = [
        '_field3',
        '_field4',
    ]

    _fields_and_field_types = {
        'field3': 'string',
        'field4': 'int64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.field3 = kwargs.get('field3', str())
        self.field4 = kwargs.get('field4', int())

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
        if self.field3 != other.field3:
            return False
        if self.field4 != other.field4:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def field3(self):
        """Message field 'field3'."""
        return self._field3

    @field3.setter
    def field3(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'field3' field must be of type 'str'"
        self._field3 = value

    @builtins.property
    def field4(self):
        """Message field 'field4'."""
        return self._field4

    @field4.setter
    def field4(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'field4' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'field4' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._field4 = value


class Metaclass_CustomService(type):
    """Metaclass of service 'CustomService'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('dis_homework1')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'dis_homework1.srv.CustomService')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__custom_service

            from dis_homework1.srv import _custom_service
            if _custom_service.Metaclass_CustomService_Request._TYPE_SUPPORT is None:
                _custom_service.Metaclass_CustomService_Request.__import_type_support__()
            if _custom_service.Metaclass_CustomService_Response._TYPE_SUPPORT is None:
                _custom_service.Metaclass_CustomService_Response.__import_type_support__()


class CustomService(metaclass=Metaclass_CustomService):
    from dis_homework1.srv._custom_service import CustomService_Request as Request
    from dis_homework1.srv._custom_service import CustomService_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
