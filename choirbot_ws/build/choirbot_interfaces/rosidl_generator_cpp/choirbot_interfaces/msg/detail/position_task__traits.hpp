// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from choirbot_interfaces:msg/PositionTask.idl
// generated code does not contain a copyright notice

#ifndef CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK__TRAITS_HPP_
#define CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK__TRAITS_HPP_

#include "choirbot_interfaces/msg/detail/position_task__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<choirbot_interfaces::msg::PositionTask>()
{
  return "choirbot_interfaces::msg::PositionTask";
}

template<>
inline const char * name<choirbot_interfaces::msg::PositionTask>()
{
  return "choirbot_interfaces/msg/PositionTask";
}

template<>
struct has_fixed_size<choirbot_interfaces::msg::PositionTask>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<choirbot_interfaces::msg::PositionTask>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<choirbot_interfaces::msg::PositionTask>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK__TRAITS_HPP_
