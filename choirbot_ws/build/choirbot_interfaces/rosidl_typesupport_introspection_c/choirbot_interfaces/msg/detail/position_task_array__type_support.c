// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from choirbot_interfaces:msg/PositionTaskArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "choirbot_interfaces/msg/detail/position_task_array__rosidl_typesupport_introspection_c.h"
#include "choirbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "choirbot_interfaces/msg/detail/position_task_array__functions.h"
#include "choirbot_interfaces/msg/detail/position_task_array__struct.h"


// Include directives for member types
// Member `tasks`
#include "choirbot_interfaces/msg/position_task.h"
// Member `tasks`
#include "choirbot_interfaces/msg/detail/position_task__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void PositionTaskArray__rosidl_typesupport_introspection_c__PositionTaskArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  choirbot_interfaces__msg__PositionTaskArray__init(message_memory);
}

void PositionTaskArray__rosidl_typesupport_introspection_c__PositionTaskArray_fini_function(void * message_memory)
{
  choirbot_interfaces__msg__PositionTaskArray__fini(message_memory);
}

size_t PositionTaskArray__rosidl_typesupport_introspection_c__size_function__PositionTask__tasks(
  const void * untyped_member)
{
  const choirbot_interfaces__msg__PositionTask__Sequence * member =
    (const choirbot_interfaces__msg__PositionTask__Sequence *)(untyped_member);
  return member->size;
}

const void * PositionTaskArray__rosidl_typesupport_introspection_c__get_const_function__PositionTask__tasks(
  const void * untyped_member, size_t index)
{
  const choirbot_interfaces__msg__PositionTask__Sequence * member =
    (const choirbot_interfaces__msg__PositionTask__Sequence *)(untyped_member);
  return &member->data[index];
}

void * PositionTaskArray__rosidl_typesupport_introspection_c__get_function__PositionTask__tasks(
  void * untyped_member, size_t index)
{
  choirbot_interfaces__msg__PositionTask__Sequence * member =
    (choirbot_interfaces__msg__PositionTask__Sequence *)(untyped_member);
  return &member->data[index];
}

bool PositionTaskArray__rosidl_typesupport_introspection_c__resize_function__PositionTask__tasks(
  void * untyped_member, size_t size)
{
  choirbot_interfaces__msg__PositionTask__Sequence * member =
    (choirbot_interfaces__msg__PositionTask__Sequence *)(untyped_member);
  choirbot_interfaces__msg__PositionTask__Sequence__fini(member);
  return choirbot_interfaces__msg__PositionTask__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember PositionTaskArray__rosidl_typesupport_introspection_c__PositionTaskArray_message_member_array[3] = {
  {
    "tasks",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__msg__PositionTaskArray, tasks),  // bytes offset in struct
    NULL,  // default value
    PositionTaskArray__rosidl_typesupport_introspection_c__size_function__PositionTask__tasks,  // size() function pointer
    PositionTaskArray__rosidl_typesupport_introspection_c__get_const_function__PositionTask__tasks,  // get_const(index) function pointer
    PositionTaskArray__rosidl_typesupport_introspection_c__get_function__PositionTask__tasks,  // get(index) function pointer
    PositionTaskArray__rosidl_typesupport_introspection_c__resize_function__PositionTask__tasks  // resize(index) function pointer
  },
  {
    "all_tasks_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__msg__PositionTaskArray, all_tasks_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "label",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__msg__PositionTaskArray, label),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PositionTaskArray__rosidl_typesupport_introspection_c__PositionTaskArray_message_members = {
  "choirbot_interfaces__msg",  // message namespace
  "PositionTaskArray",  // message name
  3,  // number of fields
  sizeof(choirbot_interfaces__msg__PositionTaskArray),
  PositionTaskArray__rosidl_typesupport_introspection_c__PositionTaskArray_message_member_array,  // message members
  PositionTaskArray__rosidl_typesupport_introspection_c__PositionTaskArray_init_function,  // function to initialize message memory (memory has to be allocated)
  PositionTaskArray__rosidl_typesupport_introspection_c__PositionTaskArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PositionTaskArray__rosidl_typesupport_introspection_c__PositionTaskArray_message_type_support_handle = {
  0,
  &PositionTaskArray__rosidl_typesupport_introspection_c__PositionTaskArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_choirbot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, msg, PositionTaskArray)() {
  PositionTaskArray__rosidl_typesupport_introspection_c__PositionTaskArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, msg, PositionTask)();
  if (!PositionTaskArray__rosidl_typesupport_introspection_c__PositionTaskArray_message_type_support_handle.typesupport_identifier) {
    PositionTaskArray__rosidl_typesupport_introspection_c__PositionTaskArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PositionTaskArray__rosidl_typesupport_introspection_c__PositionTaskArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
