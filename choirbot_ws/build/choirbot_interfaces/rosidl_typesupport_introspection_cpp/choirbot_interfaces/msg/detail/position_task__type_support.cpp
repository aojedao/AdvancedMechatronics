// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from choirbot_interfaces:msg/PositionTask.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "choirbot_interfaces/msg/detail/position_task__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace choirbot_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void PositionTask_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) choirbot_interfaces::msg::PositionTask(_init);
}

void PositionTask_fini_function(void * message_memory)
{
  auto typed_message = static_cast<choirbot_interfaces::msg::PositionTask *>(message_memory);
  typed_message->~PositionTask();
}

size_t size_function__PositionTask__coordinates(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__PositionTask__coordinates(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__PositionTask__coordinates(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void resize_function__PositionTask__coordinates(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PositionTask_message_member_array[3] = {
  {
    "coordinates",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces::msg::PositionTask, coordinates),  // bytes offset in struct
    nullptr,  // default value
    size_function__PositionTask__coordinates,  // size() function pointer
    get_const_function__PositionTask__coordinates,  // get_const(index) function pointer
    get_function__PositionTask__coordinates,  // get(index) function pointer
    resize_function__PositionTask__coordinates  // resize(index) function pointer
  },
  {
    "id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces::msg::PositionTask, id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "seq_num",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces::msg::PositionTask, seq_num),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PositionTask_message_members = {
  "choirbot_interfaces::msg",  // message namespace
  "PositionTask",  // message name
  3,  // number of fields
  sizeof(choirbot_interfaces::msg::PositionTask),
  PositionTask_message_member_array,  // message members
  PositionTask_init_function,  // function to initialize message memory (memory has to be allocated)
  PositionTask_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PositionTask_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PositionTask_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace choirbot_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<choirbot_interfaces::msg::PositionTask>()
{
  return &::choirbot_interfaces::msg::rosidl_typesupport_introspection_cpp::PositionTask_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, choirbot_interfaces, msg, PositionTask)() {
  return &::choirbot_interfaces::msg::rosidl_typesupport_introspection_cpp::PositionTask_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
