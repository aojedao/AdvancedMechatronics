// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from choirbot_interfaces:action/PositionAction.idl
// generated code does not contain a copyright notice

#ifndef CHOIRBOT_INTERFACES__ACTION__DETAIL__POSITION_ACTION__TRAITS_HPP_
#define CHOIRBOT_INTERFACES__ACTION__DETAIL__POSITION_ACTION__TRAITS_HPP_

#include "choirbot_interfaces/action/detail/position_action__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<choirbot_interfaces::action::PositionAction_Goal>()
{
  return "choirbot_interfaces::action::PositionAction_Goal";
}

template<>
inline const char * name<choirbot_interfaces::action::PositionAction_Goal>()
{
  return "choirbot_interfaces/action/PositionAction_Goal";
}

template<>
struct has_fixed_size<choirbot_interfaces::action::PositionAction_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<choirbot_interfaces::action::PositionAction_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<choirbot_interfaces::action::PositionAction_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<choirbot_interfaces::action::PositionAction_Result>()
{
  return "choirbot_interfaces::action::PositionAction_Result";
}

template<>
inline const char * name<choirbot_interfaces::action::PositionAction_Result>()
{
  return "choirbot_interfaces/action/PositionAction_Result";
}

template<>
struct has_fixed_size<choirbot_interfaces::action::PositionAction_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<choirbot_interfaces::action::PositionAction_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<choirbot_interfaces::action::PositionAction_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<choirbot_interfaces::action::PositionAction_Feedback>()
{
  return "choirbot_interfaces::action::PositionAction_Feedback";
}

template<>
inline const char * name<choirbot_interfaces::action::PositionAction_Feedback>()
{
  return "choirbot_interfaces/action/PositionAction_Feedback";
}

template<>
struct has_fixed_size<choirbot_interfaces::action::PositionAction_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<choirbot_interfaces::action::PositionAction_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<choirbot_interfaces::action::PositionAction_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "choirbot_interfaces/action/detail/position_action__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<choirbot_interfaces::action::PositionAction_SendGoal_Request>()
{
  return "choirbot_interfaces::action::PositionAction_SendGoal_Request";
}

template<>
inline const char * name<choirbot_interfaces::action::PositionAction_SendGoal_Request>()
{
  return "choirbot_interfaces/action/PositionAction_SendGoal_Request";
}

template<>
struct has_fixed_size<choirbot_interfaces::action::PositionAction_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<choirbot_interfaces::action::PositionAction_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<choirbot_interfaces::action::PositionAction_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<choirbot_interfaces::action::PositionAction_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<choirbot_interfaces::action::PositionAction_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<choirbot_interfaces::action::PositionAction_SendGoal_Response>()
{
  return "choirbot_interfaces::action::PositionAction_SendGoal_Response";
}

template<>
inline const char * name<choirbot_interfaces::action::PositionAction_SendGoal_Response>()
{
  return "choirbot_interfaces/action/PositionAction_SendGoal_Response";
}

template<>
struct has_fixed_size<choirbot_interfaces::action::PositionAction_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<choirbot_interfaces::action::PositionAction_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<choirbot_interfaces::action::PositionAction_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<choirbot_interfaces::action::PositionAction_SendGoal>()
{
  return "choirbot_interfaces::action::PositionAction_SendGoal";
}

template<>
inline const char * name<choirbot_interfaces::action::PositionAction_SendGoal>()
{
  return "choirbot_interfaces/action/PositionAction_SendGoal";
}

template<>
struct has_fixed_size<choirbot_interfaces::action::PositionAction_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<choirbot_interfaces::action::PositionAction_SendGoal_Request>::value &&
    has_fixed_size<choirbot_interfaces::action::PositionAction_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<choirbot_interfaces::action::PositionAction_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<choirbot_interfaces::action::PositionAction_SendGoal_Request>::value &&
    has_bounded_size<choirbot_interfaces::action::PositionAction_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<choirbot_interfaces::action::PositionAction_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<choirbot_interfaces::action::PositionAction_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<choirbot_interfaces::action::PositionAction_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<choirbot_interfaces::action::PositionAction_GetResult_Request>()
{
  return "choirbot_interfaces::action::PositionAction_GetResult_Request";
}

template<>
inline const char * name<choirbot_interfaces::action::PositionAction_GetResult_Request>()
{
  return "choirbot_interfaces/action/PositionAction_GetResult_Request";
}

template<>
struct has_fixed_size<choirbot_interfaces::action::PositionAction_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<choirbot_interfaces::action::PositionAction_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<choirbot_interfaces::action::PositionAction_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "choirbot_interfaces/action/detail/position_action__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<choirbot_interfaces::action::PositionAction_GetResult_Response>()
{
  return "choirbot_interfaces::action::PositionAction_GetResult_Response";
}

template<>
inline const char * name<choirbot_interfaces::action::PositionAction_GetResult_Response>()
{
  return "choirbot_interfaces/action/PositionAction_GetResult_Response";
}

template<>
struct has_fixed_size<choirbot_interfaces::action::PositionAction_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<choirbot_interfaces::action::PositionAction_Result>::value> {};

template<>
struct has_bounded_size<choirbot_interfaces::action::PositionAction_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<choirbot_interfaces::action::PositionAction_Result>::value> {};

template<>
struct is_message<choirbot_interfaces::action::PositionAction_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<choirbot_interfaces::action::PositionAction_GetResult>()
{
  return "choirbot_interfaces::action::PositionAction_GetResult";
}

template<>
inline const char * name<choirbot_interfaces::action::PositionAction_GetResult>()
{
  return "choirbot_interfaces/action/PositionAction_GetResult";
}

template<>
struct has_fixed_size<choirbot_interfaces::action::PositionAction_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<choirbot_interfaces::action::PositionAction_GetResult_Request>::value &&
    has_fixed_size<choirbot_interfaces::action::PositionAction_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<choirbot_interfaces::action::PositionAction_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<choirbot_interfaces::action::PositionAction_GetResult_Request>::value &&
    has_bounded_size<choirbot_interfaces::action::PositionAction_GetResult_Response>::value
  >
{
};

template<>
struct is_service<choirbot_interfaces::action::PositionAction_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<choirbot_interfaces::action::PositionAction_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<choirbot_interfaces::action::PositionAction_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "choirbot_interfaces/action/detail/position_action__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<choirbot_interfaces::action::PositionAction_FeedbackMessage>()
{
  return "choirbot_interfaces::action::PositionAction_FeedbackMessage";
}

template<>
inline const char * name<choirbot_interfaces::action::PositionAction_FeedbackMessage>()
{
  return "choirbot_interfaces/action/PositionAction_FeedbackMessage";
}

template<>
struct has_fixed_size<choirbot_interfaces::action::PositionAction_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<choirbot_interfaces::action::PositionAction_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<choirbot_interfaces::action::PositionAction_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<choirbot_interfaces::action::PositionAction_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<choirbot_interfaces::action::PositionAction_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<choirbot_interfaces::action::PositionAction>
  : std::true_type
{
};

template<>
struct is_action_goal<choirbot_interfaces::action::PositionAction_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<choirbot_interfaces::action::PositionAction_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<choirbot_interfaces::action::PositionAction_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // CHOIRBOT_INTERFACES__ACTION__DETAIL__POSITION_ACTION__TRAITS_HPP_
