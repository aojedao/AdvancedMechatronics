// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from choirbot_interfaces:action/PositionAction.idl
// generated code does not contain a copyright notice

#ifndef CHOIRBOT_INTERFACES__ACTION__DETAIL__POSITION_ACTION__BUILDER_HPP_
#define CHOIRBOT_INTERFACES__ACTION__DETAIL__POSITION_ACTION__BUILDER_HPP_

#include "choirbot_interfaces/action/detail/position_action__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace choirbot_interfaces
{

namespace action
{

namespace builder
{

class Init_PositionAction_Goal_goal_position
{
public:
  Init_PositionAction_Goal_goal_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::choirbot_interfaces::action::PositionAction_Goal goal_position(::choirbot_interfaces::action::PositionAction_Goal::_goal_position_type arg)
  {
    msg_.goal_position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::choirbot_interfaces::action::PositionAction_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::choirbot_interfaces::action::PositionAction_Goal>()
{
  return choirbot_interfaces::action::builder::Init_PositionAction_Goal_goal_position();
}

}  // namespace choirbot_interfaces


namespace choirbot_interfaces
{

namespace action
{

namespace builder
{

class Init_PositionAction_Result_final_position
{
public:
  Init_PositionAction_Result_final_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::choirbot_interfaces::action::PositionAction_Result final_position(::choirbot_interfaces::action::PositionAction_Result::_final_position_type arg)
  {
    msg_.final_position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::choirbot_interfaces::action::PositionAction_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::choirbot_interfaces::action::PositionAction_Result>()
{
  return choirbot_interfaces::action::builder::Init_PositionAction_Result_final_position();
}

}  // namespace choirbot_interfaces


namespace choirbot_interfaces
{

namespace action
{

namespace builder
{

class Init_PositionAction_Feedback_distance
{
public:
  explicit Init_PositionAction_Feedback_distance(::choirbot_interfaces::action::PositionAction_Feedback & msg)
  : msg_(msg)
  {}
  ::choirbot_interfaces::action::PositionAction_Feedback distance(::choirbot_interfaces::action::PositionAction_Feedback::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::choirbot_interfaces::action::PositionAction_Feedback msg_;
};

class Init_PositionAction_Feedback_current_position
{
public:
  Init_PositionAction_Feedback_current_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PositionAction_Feedback_distance current_position(::choirbot_interfaces::action::PositionAction_Feedback::_current_position_type arg)
  {
    msg_.current_position = std::move(arg);
    return Init_PositionAction_Feedback_distance(msg_);
  }

private:
  ::choirbot_interfaces::action::PositionAction_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::choirbot_interfaces::action::PositionAction_Feedback>()
{
  return choirbot_interfaces::action::builder::Init_PositionAction_Feedback_current_position();
}

}  // namespace choirbot_interfaces


namespace choirbot_interfaces
{

namespace action
{

namespace builder
{

class Init_PositionAction_SendGoal_Request_goal
{
public:
  explicit Init_PositionAction_SendGoal_Request_goal(::choirbot_interfaces::action::PositionAction_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::choirbot_interfaces::action::PositionAction_SendGoal_Request goal(::choirbot_interfaces::action::PositionAction_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::choirbot_interfaces::action::PositionAction_SendGoal_Request msg_;
};

class Init_PositionAction_SendGoal_Request_goal_id
{
public:
  Init_PositionAction_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PositionAction_SendGoal_Request_goal goal_id(::choirbot_interfaces::action::PositionAction_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PositionAction_SendGoal_Request_goal(msg_);
  }

private:
  ::choirbot_interfaces::action::PositionAction_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::choirbot_interfaces::action::PositionAction_SendGoal_Request>()
{
  return choirbot_interfaces::action::builder::Init_PositionAction_SendGoal_Request_goal_id();
}

}  // namespace choirbot_interfaces


namespace choirbot_interfaces
{

namespace action
{

namespace builder
{

class Init_PositionAction_SendGoal_Response_stamp
{
public:
  explicit Init_PositionAction_SendGoal_Response_stamp(::choirbot_interfaces::action::PositionAction_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::choirbot_interfaces::action::PositionAction_SendGoal_Response stamp(::choirbot_interfaces::action::PositionAction_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::choirbot_interfaces::action::PositionAction_SendGoal_Response msg_;
};

class Init_PositionAction_SendGoal_Response_accepted
{
public:
  Init_PositionAction_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PositionAction_SendGoal_Response_stamp accepted(::choirbot_interfaces::action::PositionAction_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_PositionAction_SendGoal_Response_stamp(msg_);
  }

private:
  ::choirbot_interfaces::action::PositionAction_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::choirbot_interfaces::action::PositionAction_SendGoal_Response>()
{
  return choirbot_interfaces::action::builder::Init_PositionAction_SendGoal_Response_accepted();
}

}  // namespace choirbot_interfaces


namespace choirbot_interfaces
{

namespace action
{

namespace builder
{

class Init_PositionAction_GetResult_Request_goal_id
{
public:
  Init_PositionAction_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::choirbot_interfaces::action::PositionAction_GetResult_Request goal_id(::choirbot_interfaces::action::PositionAction_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::choirbot_interfaces::action::PositionAction_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::choirbot_interfaces::action::PositionAction_GetResult_Request>()
{
  return choirbot_interfaces::action::builder::Init_PositionAction_GetResult_Request_goal_id();
}

}  // namespace choirbot_interfaces


namespace choirbot_interfaces
{

namespace action
{

namespace builder
{

class Init_PositionAction_GetResult_Response_result
{
public:
  explicit Init_PositionAction_GetResult_Response_result(::choirbot_interfaces::action::PositionAction_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::choirbot_interfaces::action::PositionAction_GetResult_Response result(::choirbot_interfaces::action::PositionAction_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::choirbot_interfaces::action::PositionAction_GetResult_Response msg_;
};

class Init_PositionAction_GetResult_Response_status
{
public:
  Init_PositionAction_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PositionAction_GetResult_Response_result status(::choirbot_interfaces::action::PositionAction_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_PositionAction_GetResult_Response_result(msg_);
  }

private:
  ::choirbot_interfaces::action::PositionAction_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::choirbot_interfaces::action::PositionAction_GetResult_Response>()
{
  return choirbot_interfaces::action::builder::Init_PositionAction_GetResult_Response_status();
}

}  // namespace choirbot_interfaces


namespace choirbot_interfaces
{

namespace action
{

namespace builder
{

class Init_PositionAction_FeedbackMessage_feedback
{
public:
  explicit Init_PositionAction_FeedbackMessage_feedback(::choirbot_interfaces::action::PositionAction_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::choirbot_interfaces::action::PositionAction_FeedbackMessage feedback(::choirbot_interfaces::action::PositionAction_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::choirbot_interfaces::action::PositionAction_FeedbackMessage msg_;
};

class Init_PositionAction_FeedbackMessage_goal_id
{
public:
  Init_PositionAction_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PositionAction_FeedbackMessage_feedback goal_id(::choirbot_interfaces::action::PositionAction_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PositionAction_FeedbackMessage_feedback(msg_);
  }

private:
  ::choirbot_interfaces::action::PositionAction_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::choirbot_interfaces::action::PositionAction_FeedbackMessage>()
{
  return choirbot_interfaces::action::builder::Init_PositionAction_FeedbackMessage_goal_id();
}

}  // namespace choirbot_interfaces

#endif  // CHOIRBOT_INTERFACES__ACTION__DETAIL__POSITION_ACTION__BUILDER_HPP_
