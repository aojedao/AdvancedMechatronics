// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from choirbot_interfaces:msg/PositionTask.idl
// generated code does not contain a copyright notice
#include "choirbot_interfaces/msg/detail/position_task__rosidl_typesupport_fastrtps_cpp.hpp"
#include "choirbot_interfaces/msg/detail/position_task__struct.hpp"

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

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_choirbot_interfaces
cdr_serialize(
  const choirbot_interfaces::msg::PositionTask & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: coordinates
  {
    cdr << ros_message.coordinates;
  }
  // Member: id
  cdr << ros_message.id;
  // Member: seq_num
  cdr << ros_message.seq_num;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_choirbot_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  choirbot_interfaces::msg::PositionTask & ros_message)
{
  // Member: coordinates
  {
    cdr >> ros_message.coordinates;
  }

  // Member: id
  cdr >> ros_message.id;

  // Member: seq_num
  cdr >> ros_message.seq_num;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_choirbot_interfaces
get_serialized_size(
  const choirbot_interfaces::msg::PositionTask & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: coordinates
  {
    size_t array_size = ros_message.coordinates.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.coordinates[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: id
  {
    size_t item_size = sizeof(ros_message.id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: seq_num
  {
    size_t item_size = sizeof(ros_message.seq_num);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_choirbot_interfaces
max_serialized_size_PositionTask(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: coordinates
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: seq_num
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _PositionTask__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const choirbot_interfaces::msg::PositionTask *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _PositionTask__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<choirbot_interfaces::msg::PositionTask *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _PositionTask__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const choirbot_interfaces::msg::PositionTask *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _PositionTask__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_PositionTask(full_bounded, 0);
}

static message_type_support_callbacks_t _PositionTask__callbacks = {
  "choirbot_interfaces::msg",
  "PositionTask",
  _PositionTask__cdr_serialize,
  _PositionTask__cdr_deserialize,
  _PositionTask__get_serialized_size,
  _PositionTask__max_serialized_size
};

static rosidl_message_type_support_t _PositionTask__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_PositionTask__callbacks,
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
get_message_type_support_handle<choirbot_interfaces::msg::PositionTask>()
{
  return &choirbot_interfaces::msg::typesupport_fastrtps_cpp::_PositionTask__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, choirbot_interfaces, msg, PositionTask)() {
  return &choirbot_interfaces::msg::typesupport_fastrtps_cpp::_PositionTask__handle;
}

#ifdef __cplusplus
}
#endif
