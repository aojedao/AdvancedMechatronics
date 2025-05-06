// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from choirbot_interfaces:msg/PositionTaskArray.idl
// generated code does not contain a copyright notice
#include "choirbot_interfaces/msg/detail/position_task_array__rosidl_typesupport_fastrtps_cpp.hpp"
#include "choirbot_interfaces/msg/detail/position_task_array__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace choirbot_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const choirbot_interfaces::msg::PositionTask &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  choirbot_interfaces::msg::PositionTask &);
size_t get_serialized_size(
  const choirbot_interfaces::msg::PositionTask &,
  size_t current_alignment);
size_t
max_serialized_size_PositionTask(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace choirbot_interfaces


namespace choirbot_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_choirbot_interfaces
cdr_serialize(
  const choirbot_interfaces::msg::PositionTaskArray & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: tasks
  {
    size_t size = ros_message.tasks.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      choirbot_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.tasks[i],
        cdr);
    }
  }
  // Member: all_tasks_count
  cdr << ros_message.all_tasks_count;
  // Member: label
  cdr << ros_message.label;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_choirbot_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  choirbot_interfaces::msg::PositionTaskArray & ros_message)
{
  // Member: tasks
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.tasks.resize(size);
    for (size_t i = 0; i < size; i++) {
      choirbot_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.tasks[i]);
    }
  }

  // Member: all_tasks_count
  cdr >> ros_message.all_tasks_count;

  // Member: label
  cdr >> ros_message.label;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_choirbot_interfaces
get_serialized_size(
  const choirbot_interfaces::msg::PositionTaskArray & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: tasks
  {
    size_t array_size = ros_message.tasks.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        choirbot_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.tasks[index], current_alignment);
    }
  }
  // Member: all_tasks_count
  {
    size_t item_size = sizeof(ros_message.all_tasks_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: label
  {
    size_t item_size = sizeof(ros_message.label);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_choirbot_interfaces
max_serialized_size_PositionTaskArray(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: tasks
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        choirbot_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_PositionTask(
        full_bounded, current_alignment);
    }
  }

  // Member: all_tasks_count
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: label
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _PositionTaskArray__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const choirbot_interfaces::msg::PositionTaskArray *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _PositionTaskArray__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<choirbot_interfaces::msg::PositionTaskArray *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _PositionTaskArray__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const choirbot_interfaces::msg::PositionTaskArray *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _PositionTaskArray__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_PositionTaskArray(full_bounded, 0);
}

static message_type_support_callbacks_t _PositionTaskArray__callbacks = {
  "choirbot_interfaces::msg",
  "PositionTaskArray",
  _PositionTaskArray__cdr_serialize,
  _PositionTaskArray__cdr_deserialize,
  _PositionTaskArray__get_serialized_size,
  _PositionTaskArray__max_serialized_size
};

static rosidl_message_type_support_t _PositionTaskArray__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_PositionTaskArray__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace choirbot_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_choirbot_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<choirbot_interfaces::msg::PositionTaskArray>()
{
  return &choirbot_interfaces::msg::typesupport_fastrtps_cpp::_PositionTaskArray__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, choirbot_interfaces, msg, PositionTaskArray)() {
  return &choirbot_interfaces::msg::typesupport_fastrtps_cpp::_PositionTaskArray__handle;
}

#ifdef __cplusplus
}
#endif
