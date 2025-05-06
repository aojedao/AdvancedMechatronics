// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from choirbot_interfaces:srv/TaskCompletionService.idl
// generated code does not contain a copyright notice

#ifndef CHOIRBOT_INTERFACES__SRV__DETAIL__TASK_COMPLETION_SERVICE__STRUCT_H_
#define CHOIRBOT_INTERFACES__SRV__DETAIL__TASK_COMPLETION_SERVICE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/TaskCompletionService in the package choirbot_interfaces.
typedef struct choirbot_interfaces__srv__TaskCompletionService_Request
{
  uint32_t agent_id;
  uint32_t task_seq_num;
} choirbot_interfaces__srv__TaskCompletionService_Request;

// Struct for a sequence of choirbot_interfaces__srv__TaskCompletionService_Request.
typedef struct choirbot_interfaces__srv__TaskCompletionService_Request__Sequence
{
  choirbot_interfaces__srv__TaskCompletionService_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} choirbot_interfaces__srv__TaskCompletionService_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/TaskCompletionService in the package choirbot_interfaces.
typedef struct choirbot_interfaces__srv__TaskCompletionService_Response
{
  uint8_t structure_needs_at_least_one_member;
} choirbot_interfaces__srv__TaskCompletionService_Response;

// Struct for a sequence of choirbot_interfaces__srv__TaskCompletionService_Response.
typedef struct choirbot_interfaces__srv__TaskCompletionService_Response__Sequence
{
  choirbot_interfaces__srv__TaskCompletionService_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} choirbot_interfaces__srv__TaskCompletionService_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHOIRBOT_INTERFACES__SRV__DETAIL__TASK_COMPLETION_SERVICE__STRUCT_H_
