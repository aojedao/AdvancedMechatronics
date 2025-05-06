// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from choirbot_interfaces:msg/PositionTaskArray.idl
// generated code does not contain a copyright notice
#include "choirbot_interfaces/msg/detail/position_task_array__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "choirbot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "choirbot_interfaces/msg/detail/position_task_array__struct.h"
#include "choirbot_interfaces/msg/detail/position_task_array__functions.h"
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

#include "choirbot_interfaces/msg/detail/position_task__functions.h"  // tasks

// forward declare type support functions
size_t get_serialized_size_choirbot_interfaces__msg__PositionTask(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_choirbot_interfaces__msg__PositionTask(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, choirbot_interfaces, msg, PositionTask)();


using _PositionTaskArray__ros_msg_type = choirbot_interfaces__msg__PositionTaskArray;

static bool _PositionTaskArray__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _PositionTaskArray__ros_msg_type * ros_message = static_cast<const _PositionTaskArray__ros_msg_type *>(untyped_ros_message);
  // Field name: tasks
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, choirbot_interfaces, msg, PositionTask
      )()->data);
    size_t size = ros_message->tasks.size;
    auto array_ptr = ros_message->tasks.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  // Field name: all_tasks_count
  {
    cdr << ros_message->all_tasks_count;
  }

  // Field name: label
  {
    cdr << ros_message->label;
  }

  return true;
}

static bool _PositionTaskArray__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _PositionTaskArray__ros_msg_type * ros_message = static_cast<_PositionTaskArray__ros_msg_type *>(untyped_ros_message);
  // Field name: tasks
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, choirbot_interfaces, msg, PositionTask
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->tasks.data) {
      choirbot_interfaces__msg__PositionTask__Sequence__fini(&ros_message->tasks);
    }
    if (!choirbot_interfaces__msg__PositionTask__Sequence__init(&ros_message->tasks, size)) {
      return "failed to create array for field 'tasks'";
    }
    auto array_ptr = ros_message->tasks.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  // Field name: all_tasks_count
  {
    cdr >> ros_message->all_tasks_count;
  }

  // Field name: label
  {
    cdr >> ros_message->label;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_choirbot_interfaces
size_t get_serialized_size_choirbot_interfaces__msg__PositionTaskArray(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PositionTaskArray__ros_msg_type * ros_message = static_cast<const _PositionTaskArray__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name tasks
  {
    size_t array_size = ros_message->tasks.size;
    auto array_ptr = ros_message->tasks.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_choirbot_interfaces__msg__PositionTask(
        &array_ptr[index], current_alignment);
    }
  }
  // field.name all_tasks_count
  {
    size_t item_size = sizeof(ros_message->all_tasks_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name label
  {
    size_t item_size = sizeof(ros_message->label);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _PositionTaskArray__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_choirbot_interfaces__msg__PositionTaskArray(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_choirbot_interfaces
size_t max_serialized_size_choirbot_interfaces__msg__PositionTaskArray(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: tasks
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_choirbot_interfaces__msg__PositionTask(
        full_bounded, current_alignment);
    }
  }
  // member: all_tasks_count
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: label
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _PositionTaskArray__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_choirbot_interfaces__msg__PositionTaskArray(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_PositionTaskArray = {
  "choirbot_interfaces::msg",
  "PositionTaskArray",
  _PositionTaskArray__cdr_serialize,
  _PositionTaskArray__cdr_deserialize,
  _PositionTaskArray__get_serialized_size,
  _PositionTaskArray__max_serialized_size
};

static rosidl_message_type_support_t _PositionTaskArray__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PositionTaskArray,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, choirbot_interfaces, msg, PositionTaskArray)() {
  return &_PositionTaskArray__type_support;
}

#if defined(__cplusplus)
}
#endif
