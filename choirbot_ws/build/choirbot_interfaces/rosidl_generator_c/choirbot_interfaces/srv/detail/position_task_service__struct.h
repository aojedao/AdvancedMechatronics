// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from choirbot_interfaces:srv/PositionTaskService.idl
// generated code does not contain a copyright notice

#ifndef CHOIRBOT_INTERFACES__SRV__DETAIL__POSITION_TASK_SERVICE__STRUCT_H_
#define CHOIRBOT_INTERFACES__SRV__DETAIL__POSITION_TASK_SERVICE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/PositionTaskService in the package choirbot_interfaces.
typedef struct choirbot_interfaces__srv__PositionTaskService_Request
{
  uint32_t agent_id;
} choirbot_interfaces__srv__PositionTaskService_Request;

// Struct for a sequence of choirbot_interfaces__srv__PositionTaskService_Request.
typedef struct choirbot_interfaces__srv__PositionTaskService_Request__Sequence
{
  choirbot_interfaces__srv__PositionTaskService_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} choirbot_interfaces__srv__PositionTaskService_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'tasks'
#include "choirbot_interfaces/msg/detail/position_task_array__struct.h"

// Struct defined in srv/PositionTaskService in the package choirbot_interfaces.
typedef struct choirbot_interfaces__srv__PositionTaskService_Response
{
  choirbot_interfaces__msg__PositionTaskArray tasks;
} choirbot_interfaces__srv__PositionTaskService_Response;

// Struct for a sequence of choirbot_interfaces__srv__PositionTaskService_Response.
typedef struct choirbot_interfaces__srv__PositionTaskService_Response__Sequence
{
  choirbot_interfaces__srv__PositionTaskService_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} choirbot_interfaces__srv__PositionTaskService_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHOIRBOT_INTERFACES__SRV__DETAIL__POSITION_TASK_SERVICE__STRUCT_H_
