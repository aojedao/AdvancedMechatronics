// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from choirbot_interfaces:srv/TaskCompletionService.idl
// generated code does not contain a copyright notice

#ifndef CHOIRBOT_INTERFACES__SRV__DETAIL__TASK_COMPLETION_SERVICE__TRAITS_HPP_
#define CHOIRBOT_INTERFACES__SRV__DETAIL__TASK_COMPLETION_SERVICE__TRAITS_HPP_

#include "choirbot_interfaces/srv/detail/task_completion_service__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<choirbot_interfaces::srv::TaskCompletionService_Request>()
{
  return "choirbot_interfaces::srv::TaskCompletionService_Request";
}

template<>
inline const char * name<choirbot_interfaces::srv::TaskCompletionService_Request>()
{
  return "choirbot_interfaces/srv/TaskCompletionService_Request";
}

template<>
struct has_fixed_size<choirbot_interfaces::srv::TaskCompletionService_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<choirbot_interfaces::srv::TaskCompletionService_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<choirbot_interfaces::srv::TaskCompletionService_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<choirbot_interfaces::srv::TaskCompletionService_Response>()
{
  return "choirbot_interfaces::srv::TaskCompletionService_Response";
}

template<>
inline const char * name<choirbot_interfaces::srv::TaskCompletionService_Response>()
{
  return "choirbot_interfaces/srv/TaskCompletionService_Response";
}

template<>
struct has_fixed_size<choirbot_interfaces::srv::TaskCompletionService_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<choirbot_interfaces::srv::TaskCompletionService_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<choirbot_interfaces::srv::TaskCompletionService_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<choirbot_interfaces::srv::TaskCompletionService>()
{
  return "choirbot_interfaces::srv::TaskCompletionService";
}

template<>
inline const char * name<choirbot_interfaces::srv::TaskCompletionService>()
{
  return "choirbot_interfaces/srv/TaskCompletionService";
}

template<>
struct has_fixed_size<choirbot_interfaces::srv::TaskCompletionService>
  : std::integral_constant<
    bool,
    has_fixed_size<choirbot_interfaces::srv::TaskCompletionService_Request>::value &&
    has_fixed_size<choirbot_interfaces::srv::TaskCompletionService_Response>::value
  >
{
};

template<>
struct has_bounded_size<choirbot_interfaces::srv::TaskCompletionService>
  : std::integral_constant<
    bool,
    has_bounded_size<choirbot_interfaces::srv::TaskCompletionService_Request>::value &&
    has_bounded_size<choirbot_interfaces::srv::TaskCompletionService_Response>::value
  >
{
};

template<>
struct is_service<choirbot_interfaces::srv::TaskCompletionService>
  : std::true_type
{
};

template<>
struct is_service_request<choirbot_interfaces::srv::TaskCompletionService_Request>
  : std::true_type
{
};

template<>
struct is_service_response<choirbot_interfaces::srv::TaskCompletionService_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CHOIRBOT_INTERFACES__SRV__DETAIL__TASK_COMPLETION_SERVICE__TRAITS_HPP_
