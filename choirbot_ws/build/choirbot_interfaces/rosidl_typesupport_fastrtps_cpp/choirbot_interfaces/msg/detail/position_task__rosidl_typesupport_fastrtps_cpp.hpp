// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from choirbot_interfaces:msg/PositionTask.idl
// generated code does not contain a copyright notice

#ifndef CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "choirbot_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "choirbot_interfaces/msg/detail/position_task__struct.hpp"

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

#include "fastcdr/Cdr.h"

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
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_choirbot_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  choirbot_interfaces::msg::PositionTask & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_choirbot_interfaces
get_serialized_size(
  const choirbot_interfaces::msg::PositionTask & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_choirbot_interfaces
max_serialized_size_PositionTask(
  bool & full_bounded,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace choirbot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_choirbot_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, choirbot_interfaces, msg, PositionTask)();

#ifdef __cplusplus
}
#endif

#endif  // CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
