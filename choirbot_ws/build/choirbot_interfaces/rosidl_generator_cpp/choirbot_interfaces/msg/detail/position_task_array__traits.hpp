// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from choirbot_interfaces:msg/PositionTaskArray.idl
// generated code does not contain a copyright notice

#ifndef CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK_ARRAY__TRAITS_HPP_
#define CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK_ARRAY__TRAITS_HPP_

#include "choirbot_interfaces/msg/detail/position_task_array__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<choirbot_interfaces::msg::PositionTaskArray>()
{
  return "choirbot_interfaces::msg::PositionTaskArray";
}

template<>
inline const char * name<choirbot_interfaces::msg::PositionTaskArray>()
{
  return "choirbot_interfaces/msg/PositionTaskArray";
}

template<>
struct has_fixed_size<choirbot_interfaces::msg::PositionTaskArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<choirbot_interfaces::msg::PositionTaskArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<choirbot_interfaces::msg::PositionTaskArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK_ARRAY__TRAITS_HPP_
