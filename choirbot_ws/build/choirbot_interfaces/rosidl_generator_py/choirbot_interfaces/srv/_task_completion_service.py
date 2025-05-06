# generated from rosidl_generator_py/resource/_idl.py.em
# with input from choirbot_interfaces:srv/TaskCompletionService.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TaskCompletionService_Request(type):
    """Metaclass of message 'TaskCompletionService_Request'."""

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
            module = import_type_support('choirbot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'choirbot_interfaces.srv.TaskCompletionService_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__task_completion_service__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__task_completion_service__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__task_completion_service__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__task_completion_service__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__task_completion_service__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TaskCompletionService_Request(metaclass=Metaclass_TaskCompletionService_Request):
    """Message class 'TaskCompletionService_Request'."""

    __slots__ = [
        '_agent_id',
        '_task_seq_num',
    ]

    _fields_and_field_types = {
        'agent_id': 'uint32',
        'task_seq_num': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.agent_id = kwargs.get('agent_id', int())
        self.task_seq_num = kwargs.get('task_seq_num', int())

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
        if self.agent_id != other.agent_id:
            return False
        if self.task_seq_num != other.task_seq_num:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def agent_id(self):
        """Message field 'agent_id'."""
        return self._agent_id

    @agent_id.setter
    def agent_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'agent_id' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'agent_id' field must be an unsigned integer in [0, 4294967295]"
        self._agent_id = value

    @property
    def task_seq_num(self):
        """Message field 'task_seq_num'."""
        return self._task_seq_num

    @task_seq_num.setter
    def task_seq_num(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'task_seq_num' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'task_seq_num' field must be an unsigned integer in [0, 4294967295]"
        self._task_seq_num = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_TaskCompletionService_Response(type):
    """Metaclass of message 'TaskCompletionService_Response'."""

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
            module = import_type_support('choirbot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'choirbot_interfaces.srv.TaskCompletionService_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__task_completion_service__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__task_completion_service__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__task_completion_service__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__task_completion_service__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__task_completion_service__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TaskCompletionService_Response(metaclass=Metaclass_TaskCompletionService_Response):
    """Message class 'TaskCompletionService_Response'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


class Metaclass_TaskCompletionService(type):
    """Metaclass of service 'TaskCompletionService'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('choirbot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'choirbot_interfaces.srv.TaskCompletionService')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__task_completion_service

            from choirbot_interfaces.srv import _task_completion_service
            if _task_completion_service.Metaclass_TaskCompletionService_Request._TYPE_SUPPORT is None:
                _task_completion_service.Metaclass_TaskCompletionService_Request.__import_type_support__()
            if _task_completion_service.Metaclass_TaskCompletionService_Response._TYPE_SUPPORT is None:
                _task_completion_service.Metaclass_TaskCompletionService_Response.__import_type_support__()


class TaskCompletionService(metaclass=Metaclass_TaskCompletionService):
    from choirbot_interfaces.srv._task_completion_service import TaskCompletionService_Request as Request
    from choirbot_interfaces.srv._task_completion_service import TaskCompletionService_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
