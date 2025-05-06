// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from choirbot_interfaces:srv/TaskCompletionService.idl
// generated code does not contain a copyright notice

#ifndef CHOIRBOT_INTERFACES__SRV__DETAIL__TASK_COMPLETION_SERVICE__BUILDER_HPP_
#define CHOIRBOT_INTERFACES__SRV__DETAIL__TASK_COMPLETION_SERVICE__BUILDER_HPP_

#include "choirbot_interfaces/srv/detail/task_completion_service__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace choirbot_interfaces
{

namespace srv
{

namespace builder
{

class Init_TaskCompletionService_Request_task_seq_num
{
public:
  explicit Init_TaskCompletionService_Request_task_seq_num(::choirbot_interfaces::srv::TaskCompletionService_Request & msg)
  : msg_(msg)
  {}
  ::choirbot_interfaces::srv::TaskCompletionService_Request task_seq_num(::choirbot_interfaces::srv::TaskCompletionService_Request::_task_seq_num_type arg)
  {
    msg_.task_seq_num = std::move(arg);
    return std::move(msg_);
  }

private:
  ::choirbot_interfaces::srv::TaskCompletionService_Request msg_;
};

class Init_TaskCompletionService_Request_agent_id
{
public:
  Init_TaskCompletionService_Request_agent_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TaskCompletionService_Request_task_seq_num agent_id(::choirbot_interfaces::srv::TaskCompletionService_Request::_agent_id_type arg)
  {
    msg_.agent_id = std::move(arg);
    return Init_TaskCompletionService_Request_task_seq_num(msg_);
  }

private:
  ::choirbot_interfaces::srv::TaskCompletionService_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::choirbot_interfaces::srv::TaskCompletionService_Request>()
{
  return choirbot_interfaces::srv::builder::Init_TaskCompletionService_Request_agent_id();
}

}  // namespace choirbot_interfaces


namespace choirbot_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::choirbot_interfaces::srv::TaskCompletionService_Response>()
{
  return ::choirbot_interfaces::srv::TaskCompletionService_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace choirbot_interfaces

#endif  // CHOIRBOT_INTERFACES__SRV__DETAIL__TASK_COMPLETION_SERVICE__BUILDER_HPP_
