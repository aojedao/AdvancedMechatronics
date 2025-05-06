// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from choirbot_interfaces:msg/PositionTask.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "choirbot_interfaces/msg/detail/position_task__rosidl_typesupport_introspection_c.h"
#include "choirbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "choirbot_interfaces/msg/detail/position_task__functions.h"
#include "choirbot_interfaces/msg/detail/position_task__struct.h"


// Include directives for member types
// Member `coordinates`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void PositionTask__rosidl_typesupport_introspection_c__PositionTask_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  choirbot_interfaces__msg__PositionTask__init(message_memory);
}

void PositionTask__rosidl_typesupport_introspection_c__PositionTask_fini_function(void * message_memory)
{
  choirbot_interfaces__msg__PositionTask__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember PositionTask__rosidl_typesupport_introspection_c__PositionTask_message_member_array[3] = {
  {
    "coordinates",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__msg__PositionTask, coordinates),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__msg__PositionTask, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "seq_num",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__msg__PositionTask, seq_num),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PositionTask__rosidl_typesupport_introspection_c__PositionTask_message_members = {
  "choirbot_interfaces__msg",  // message namespace
  "PositionTask",  // message name
  3,  // number of fields
  sizeof(choirbot_interfaces__msg__PositionTask),
  PositionTask__rosidl_typesupport_introspection_c__PositionTask_message_member_array,  // message members
  PositionTask__rosidl_typesupport_introspection_c__PositionTask_init_function,  // function to initialize message memory (memory has to be allocated)
  PositionTask__rosidl_typesupport_introspection_c__PositionTask_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PositionTask__rosidl_typesupport_introspection_c__PositionTask_message_type_support_handle = {
  0,
  &PositionTask__rosidl_typesupport_introspection_c__PositionTask_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_choirbot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, msg, PositionTask)() {
  if (!PositionTask__rosidl_typesupport_introspection_c__PositionTask_message_type_support_handle.typesupport_identifier) {
    PositionTask__rosidl_typesupport_introspection_c__PositionTask_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PositionTask__rosidl_typesupport_introspection_c__PositionTask_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
