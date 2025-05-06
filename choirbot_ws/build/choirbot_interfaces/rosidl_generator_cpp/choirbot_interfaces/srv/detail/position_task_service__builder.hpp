// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from choirbot_interfaces:srv/PositionTaskService.idl
// generated code does not contain a copyright notice

#ifndef CHOIRBOT_INTERFACES__SRV__DETAIL__POSITION_TASK_SERVICE__BUILDER_HPP_
#define CHOIRBOT_INTERFACES__SRV__DETAIL__POSITION_TASK_SERVICE__BUILDER_HPP_

#include "choirbot_interfaces/srv/detail/position_task_service__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace choirbot_interfaces
{

namespace srv
{

namespace builder
{

class Init_PositionTaskService_Request_agent_id
{
public:
  Init_PositionTaskService_Request_agent_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::choirbot_interfaces::srv::PositionTaskService_Request agent_id(::choirbot_interfaces::srv::PositionTaskService_Request::_agent_id_type arg)
  {
    msg_.agent_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::choirbot_interfaces::srv::PositionTaskService_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::choirbot_interfaces::srv::PositionTaskService_Request>()
{
  return choirbot_interfaces::srv::builder::Init_PositionTaskService_Request_agent_id();
}

}  // namespace choirbot_interfaces


namespace choirbot_interfaces
{

namespace srv
{

namespace builder
{

class Init_PositionTaskService_Response_tasks
{
public:
  Init_PositionTaskService_Response_tasks()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::choirbot_interfaces::srv::PositionTaskService_Response tasks(::choirbot_interfaces::srv::PositionTaskService_Response::_tasks_type arg)
  {
    msg_.tasks = std::move(arg);
    return std::move(msg_);
  }

private:
  ::choirbot_interfaces::srv::PositionTaskService_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::choirbot_interfaces::srv::PositionTaskService_Response>()
{
  return choirbot_interfaces::srv::builder::Init_PositionTaskService_Response_tasks();
}

}  // namespace choirbot_interfaces

#endif  // CHOIRBOT_INTERFACES__SRV__DETAIL__POSITION_TASK_SERVICE__BUILDER_HPP_
