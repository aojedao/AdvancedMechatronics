// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from choirbot_interfaces:msg/PositionTaskArray.idl
// generated code does not contain a copyright notice

#ifndef CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK_ARRAY__STRUCT_H_
#define CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'tasks'
#include "choirbot_interfaces/msg/detail/position_task__struct.h"

// Struct defined in msg/PositionTaskArray in the package choirbot_interfaces.
typedef struct choirbot_interfaces__msg__PositionTaskArray
{
  choirbot_interfaces__msg__PositionTask__Sequence tasks;
  uint32_t all_tasks_count;
  uint32_t label;
} choirbot_interfaces__msg__PositionTaskArray;

// Struct for a sequence of choirbot_interfaces__msg__PositionTaskArray.
typedef struct choirbot_interfaces__msg__PositionTaskArray__Sequence
{
  choirbot_interfaces__msg__PositionTaskArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} choirbot_interfaces__msg__PositionTaskArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK_ARRAY__STRUCT_H_
