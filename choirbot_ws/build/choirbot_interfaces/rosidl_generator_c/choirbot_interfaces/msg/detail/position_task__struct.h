// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from choirbot_interfaces:msg/PositionTask.idl
// generated code does not contain a copyright notice

#ifndef CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK__STRUCT_H_
#define CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'coordinates'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/PositionTask in the package choirbot_interfaces.
typedef struct choirbot_interfaces__msg__PositionTask
{
  rosidl_runtime_c__double__Sequence coordinates;
  uint32_t id;
  uint32_t seq_num;
} choirbot_interfaces__msg__PositionTask;

// Struct for a sequence of choirbot_interfaces__msg__PositionTask.
typedef struct choirbot_interfaces__msg__PositionTask__Sequence
{
  choirbot_interfaces__msg__PositionTask * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} choirbot_interfaces__msg__PositionTask__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK__STRUCT_H_
