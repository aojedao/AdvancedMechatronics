// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from choirbot_interfaces:srv/PositionTaskService.idl
// generated code does not contain a copyright notice

#ifndef CHOIRBOT_INTERFACES__SRV__DETAIL__POSITION_TASK_SERVICE__TRAITS_HPP_
#define CHOIRBOT_INTERFACES__SRV__DETAIL__POSITION_TASK_SERVICE__TRAITS_HPP_

#include "choirbot_interfaces/srv/detail/position_task_service__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<choirbot_interfaces::srv::PositionTaskService_Request>()
{
  return "choirbot_interfaces::srv::PositionTaskService_Request";
}

template<>
inline const char * name<choirbot_interfaces::srv::PositionTaskService_Request>()
{
  return "choirbot_interfaces/srv/PositionTaskService_Request";
}

template<>
struct has_fixed_size<choirbot_interfaces::srv::PositionTaskService_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<choirbot_interfaces::srv::PositionTaskService_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<choirbot_interfaces::srv::PositionTaskService_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'tasks'
#include "choirbot_interfaces/msg/detail/position_task_array__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<choirbot_interfaces::srv::PositionTaskService_Response>()
{
  return "choirbot_interfaces::srv::PositionTaskService_Response";
}

template<>
inline const char * name<choirbot_interfaces::srv::PositionTaskService_Response>()
{
  return "choirbot_interfaces/srv/PositionTaskService_Response";
}

template<>
struct has_fixed_size<choirbot_interfaces::srv::PositionTaskService_Response>
  : std::integral_constant<bool, has_fixed_size<choirbot_interfaces::msg::PositionTaskArray>::value> {};

template<>
struct has_bounded_size<choirbot_interfaces::srv::PositionTaskService_Response>
  : std::integral_constant<bool, has_bounded_size<choirbot_interfaces::msg::PositionTaskArray>::value> {};

template<>
struct is_message<choirbot_interfaces::srv::PositionTaskService_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<choirbot_interfaces::srv::PositionTaskService>()
{
  return "choirbot_interfaces::srv::PositionTaskService";
}

template<>
inline const char * name<choirbot_interfaces::srv::PositionTaskService>()
{
  return "choirbot_interfaces/srv/PositionTaskService";
}

template<>
struct has_fixed_size<choirbot_interfaces::srv::PositionTaskService>
  : std::integral_constant<
    bool,
    has_fixed_size<choirbot_interfaces::srv::PositionTaskService_Request>::value &&
    has_fixed_size<choirbot_interfaces::srv::PositionTaskService_Response>::value
  >
{
};

template<>
struct has_bounded_size<choirbot_interfaces::srv::PositionTaskService>
  : std::integral_constant<
    bool,
    has_bounded_size<choirbot_interfaces::srv::PositionTaskService_Request>::value &&
    has_bounded_size<choirbot_interfaces::srv::PositionTaskService_Response>::value
  >
{
};

template<>
struct is_service<choirbot_interfaces::srv::PositionTaskService>
  : std::true_type
{
};

template<>
struct is_service_request<choirbot_interfaces::srv::PositionTaskService_Request>
  : std::true_type
{
};

template<>
struct is_service_response<choirbot_interfaces::srv::PositionTaskService_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CHOIRBOT_INTERFACES__SRV__DETAIL__POSITION_TASK_SERVICE__TRAITS_HPP_
