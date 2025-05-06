// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from choirbot_interfaces:srv/TaskCompletionService.idl
// generated code does not contain a copyright notice
#include "choirbot_interfaces/srv/detail/task_completion_service__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "choirbot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "choirbot_interfaces/srv/detail/task_completion_service__struct.h"
#include "choirbot_interfaces/srv/detail/task_completion_service__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _TaskCompletionService_Request__ros_msg_type = choirbot_interfaces__srv__TaskCompletionService_Request;

static bool _TaskCompletionService_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _TaskCompletionService_Request__ros_msg_type * ros_message = static_cast<const _TaskCompletionService_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: agent_id
  {
    cdr << ros_message->agent_id;
  }

  // Field name: task_seq_num
  {
    cdr << ros_message->task_seq_num;
  }

  return true;
}

static bool _TaskCompletionService_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _TaskCompletionService_Request__ros_msg_type * ros_message = static_cast<_TaskCompletionService_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: agent_id
  {
    cdr >> ros_message->agent_id;
  }

  // Field name: task_seq_num
  {
    cdr >> ros_message->task_seq_num;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_choirbot_interfaces
size_t get_serialized_size_choirbot_interfaces__srv__TaskCompletionService_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TaskCompletionService_Request__ros_msg_type * ros_message = static_cast<const _TaskCompletionService_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name agent_id
  {
    size_t item_size = sizeof(ros_message->agent_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name task_seq_num
  {
    size_t item_size = sizeof(ros_message->task_seq_num);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _TaskCompletionService_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_choirbot_interfaces__srv__TaskCompletionService_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_choirbot_interfaces
size_t max_serialized_size_choirbot_interfaces__srv__TaskCompletionService_Request(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: agent_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: task_seq_num
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _TaskCompletionService_Request__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_choirbot_interfaces__srv__TaskCompletionService_Request(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_TaskCompletionService_Request = {
  "choirbot_interfaces::srv",
  "TaskCompletionService_Request",
  _TaskCompletionService_Request__cdr_serialize,
  _TaskCompletionService_Request__cdr_deserialize,
  _TaskCompletionService_Request__get_serialized_size,
  _TaskCompletionService_Request__max_serialized_size
};

static rosidl_message_type_support_t _TaskCompletionService_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TaskCompletionService_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, choirbot_interfaces, srv, TaskCompletionService_Request)() {
  return &_TaskCompletionService_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "choirbot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "choirbot_interfaces/srv/detail/task_completion_service__struct.h"
// already included above
// #include "choirbot_interfaces/srv/detail/task_completion_service__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _TaskCompletionService_Response__ros_msg_type = choirbot_interfaces__srv__TaskCompletionService_Response;

static bool _TaskCompletionService_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _TaskCompletionService_Response__ros_msg_type * ros_message = static_cast<const _TaskCompletionService_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr << ros_message->structure_needs_at_least_one_member;
  }

  return true;
}

static bool _TaskCompletionService_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _TaskCompletionService_Response__ros_msg_type * ros_message = static_cast<_TaskCompletionService_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr >> ros_message->structure_needs_at_least_one_member;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_choirbot_interfaces
size_t get_serialized_size_choirbot_interfaces__srv__TaskCompletionService_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TaskCompletionService_Response__ros_msg_type * ros_message = static_cast<const _TaskCompletionService_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name structure_needs_at_least_one_member
  {
    size_t item_size = sizeof(ros_message->structure_needs_at_least_one_member);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _TaskCompletionService_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_choirbot_interfaces__srv__TaskCompletionService_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_choirbot_interfaces
size_t max_serialized_size_choirbot_interfaces__srv__TaskCompletionService_Response(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: structure_needs_at_least_one_member
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _TaskCompletionService_Response__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_choirbot_interfaces__srv__TaskCompletionService_Response(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_TaskCompletionService_Response = {
  "choirbot_interfaces::srv",
  "TaskCompletionService_Response",
  _TaskCompletionService_Response__cdr_serialize,
  _TaskCompletionService_Response__cdr_deserialize,
  _TaskCompletionService_Response__get_serialized_size,
  _TaskCompletionService_Response__max_serialized_size
};

static rosidl_message_type_support_t _TaskCompletionService_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TaskCompletionService_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, choirbot_interfaces, srv, TaskCompletionService_Response)() {
  return &_TaskCompletionService_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "choirbot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "choirbot_interfaces/srv/task_completion_service.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t TaskCompletionService__callbacks = {
  "choirbot_interfaces::srv",
  "TaskCompletionService",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, choirbot_interfaces, srv, TaskCompletionService_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, choirbot_interfaces, srv, TaskCompletionService_Response)(),
};

static rosidl_service_type_support_t TaskCompletionService__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &TaskCompletionService__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, choirbot_interfaces, srv, TaskCompletionService)() {
  return &TaskCompletionService__handle;
}

#if defined(__cplusplus)
}
#endif
