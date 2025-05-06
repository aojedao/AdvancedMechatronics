// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from choirbot_interfaces:msg/PositionTask.idl
// generated code does not contain a copyright notice

#ifndef CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK__BUILDER_HPP_
#define CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK__BUILDER_HPP_

#include "choirbot_interfaces/msg/detail/position_task__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace choirbot_interfaces
{

namespace msg
{

namespace builder
{

class Init_PositionTask_seq_num
{
public:
  explicit Init_PositionTask_seq_num(::choirbot_interfaces::msg::PositionTask & msg)
  : msg_(msg)
  {}
  ::choirbot_interfaces::msg::PositionTask seq_num(::choirbot_interfaces::msg::PositionTask::_seq_num_type arg)
  {
    msg_.seq_num = std::move(arg);
    return std::move(msg_);
  }

private:
  ::choirbot_interfaces::msg::PositionTask msg_;
};

class Init_PositionTask_id
{
public:
  explicit Init_PositionTask_id(::choirbot_interfaces::msg::PositionTask & msg)
  : msg_(msg)
  {}
  Init_PositionTask_seq_num id(::choirbot_interfaces::msg::PositionTask::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_PositionTask_seq_num(msg_);
  }

private:
  ::choirbot_interfaces::msg::PositionTask msg_;
};

class Init_PositionTask_coordinates
{
public:
  Init_PositionTask_coordinates()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PositionTask_id coordinates(::choirbot_interfaces::msg::PositionTask::_coordinates_type arg)
  {
    msg_.coordinates = std::move(arg);
    return Init_PositionTask_id(msg_);
  }

private:
  ::choirbot_interfaces::msg::PositionTask msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::choirbot_interfaces::msg::PositionTask>()
{
  return choirbot_interfaces::msg::builder::Init_PositionTask_coordinates();
}

}  // namespace choirbot_interfaces

#endif  // CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK__BUILDER_HPP_
