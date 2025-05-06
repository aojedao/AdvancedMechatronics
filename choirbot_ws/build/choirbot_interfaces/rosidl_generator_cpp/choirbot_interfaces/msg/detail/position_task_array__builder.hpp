// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from choirbot_interfaces:msg/PositionTaskArray.idl
// generated code does not contain a copyright notice

#ifndef CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK_ARRAY__BUILDER_HPP_
#define CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK_ARRAY__BUILDER_HPP_

#include "choirbot_interfaces/msg/detail/position_task_array__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace choirbot_interfaces
{

namespace msg
{

namespace builder
{

class Init_PositionTaskArray_label
{
public:
  explicit Init_PositionTaskArray_label(::choirbot_interfaces::msg::PositionTaskArray & msg)
  : msg_(msg)
  {}
  ::choirbot_interfaces::msg::PositionTaskArray label(::choirbot_interfaces::msg::PositionTaskArray::_label_type arg)
  {
    msg_.label = std::move(arg);
    return std::move(msg_);
  }

private:
  ::choirbot_interfaces::msg::PositionTaskArray msg_;
};

class Init_PositionTaskArray_all_tasks_count
{
public:
  explicit Init_PositionTaskArray_all_tasks_count(::choirbot_interfaces::msg::PositionTaskArray & msg)
  : msg_(msg)
  {}
  Init_PositionTaskArray_label all_tasks_count(::choirbot_interfaces::msg::PositionTaskArray::_all_tasks_count_type arg)
  {
    msg_.all_tasks_count = std::move(arg);
    return Init_PositionTaskArray_label(msg_);
  }

private:
  ::choirbot_interfaces::msg::PositionTaskArray msg_;
};

class Init_PositionTaskArray_tasks
{
public:
  Init_PositionTaskArray_tasks()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PositionTaskArray_all_tasks_count tasks(::choirbot_interfaces::msg::PositionTaskArray::_tasks_type arg)
  {
    msg_.tasks = std::move(arg);
    return Init_PositionTaskArray_all_tasks_count(msg_);
  }

private:
  ::choirbot_interfaces::msg::PositionTaskArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::choirbot_interfaces::msg::PositionTaskArray>()
{
  return choirbot_interfaces::msg::builder::Init_PositionTaskArray_tasks();
}

}  // namespace choirbot_interfaces

#endif  // CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK_ARRAY__BUILDER_HPP_
