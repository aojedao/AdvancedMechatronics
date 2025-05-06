// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from choirbot_interfaces:action/PositionAction.idl
// generated code does not contain a copyright notice

#ifndef CHOIRBOT_INTERFACES__ACTION__DETAIL__POSITION_ACTION__STRUCT_H_
#define CHOIRBOT_INTERFACES__ACTION__DETAIL__POSITION_ACTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'goal_position'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in action/PositionAction in the package choirbot_interfaces.
typedef struct choirbot_interfaces__action__PositionAction_Goal
{
  rosidl_runtime_c__double__Sequence goal_position;
} choirbot_interfaces__action__PositionAction_Goal;

// Struct for a sequence of choirbot_interfaces__action__PositionAction_Goal.
typedef struct choirbot_interfaces__action__PositionAction_Goal__Sequence
{
  choirbot_interfaces__action__PositionAction_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} choirbot_interfaces__action__PositionAction_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'final_position'
// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in action/PositionAction in the package choirbot_interfaces.
typedef struct choirbot_interfaces__action__PositionAction_Result
{
  rosidl_runtime_c__double__Sequence final_position;
} choirbot_interfaces__action__PositionAction_Result;

// Struct for a sequence of choirbot_interfaces__action__PositionAction_Result.
typedef struct choirbot_interfaces__action__PositionAction_Result__Sequence
{
  choirbot_interfaces__action__PositionAction_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} choirbot_interfaces__action__PositionAction_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'current_position'
// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in action/PositionAction in the package choirbot_interfaces.
typedef struct choirbot_interfaces__action__PositionAction_Feedback
{
  rosidl_runtime_c__double__Sequence current_position;
  double distance;
} choirbot_interfaces__action__PositionAction_Feedback;

// Struct for a sequence of choirbot_interfaces__action__PositionAction_Feedback.
typedef struct choirbot_interfaces__action__PositionAction_Feedback__Sequence
{
  choirbot_interfaces__action__PositionAction_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} choirbot_interfaces__action__PositionAction_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "choirbot_interfaces/action/detail/position_action__struct.h"

// Struct defined in action/PositionAction in the package choirbot_interfaces.
typedef struct choirbot_interfaces__action__PositionAction_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  choirbot_interfaces__action__PositionAction_Goal goal;
} choirbot_interfaces__action__PositionAction_SendGoal_Request;

// Struct for a sequence of choirbot_interfaces__action__PositionAction_SendGoal_Request.
typedef struct choirbot_interfaces__action__PositionAction_SendGoal_Request__Sequence
{
  choirbot_interfaces__action__PositionAction_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} choirbot_interfaces__action__PositionAction_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

// Struct defined in action/PositionAction in the package choirbot_interfaces.
typedef struct choirbot_interfaces__action__PositionAction_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} choirbot_interfaces__action__PositionAction_SendGoal_Response;

// Struct for a sequence of choirbot_interfaces__action__PositionAction_SendGoal_Response.
typedef struct choirbot_interfaces__action__PositionAction_SendGoal_Response__Sequence
{
  choirbot_interfaces__action__PositionAction_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} choirbot_interfaces__action__PositionAction_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

// Struct defined in action/PositionAction in the package choirbot_interfaces.
typedef struct choirbot_interfaces__action__PositionAction_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} choirbot_interfaces__action__PositionAction_GetResult_Request;

// Struct for a sequence of choirbot_interfaces__action__PositionAction_GetResult_Request.
typedef struct choirbot_interfaces__action__PositionAction_GetResult_Request__Sequence
{
  choirbot_interfaces__action__PositionAction_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} choirbot_interfaces__action__PositionAction_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "choirbot_interfaces/action/detail/position_action__struct.h"

// Struct defined in action/PositionAction in the package choirbot_interfaces.
typedef struct choirbot_interfaces__action__PositionAction_GetResult_Response
{
  int8_t status;
  choirbot_interfaces__action__PositionAction_Result result;
} choirbot_interfaces__action__PositionAction_GetResult_Response;

// Struct for a sequence of choirbot_interfaces__action__PositionAction_GetResult_Response.
typedef struct choirbot_interfaces__action__PositionAction_GetResult_Response__Sequence
{
  choirbot_interfaces__action__PositionAction_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} choirbot_interfaces__action__PositionAction_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "choirbot_interfaces/action/detail/position_action__struct.h"

// Struct defined in action/PositionAction in the package choirbot_interfaces.
typedef struct choirbot_interfaces__action__PositionAction_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  choirbot_interfaces__action__PositionAction_Feedback feedback;
} choirbot_interfaces__action__PositionAction_FeedbackMessage;

// Struct for a sequence of choirbot_interfaces__action__PositionAction_FeedbackMessage.
typedef struct choirbot_interfaces__action__PositionAction_FeedbackMessage__Sequence
{
  choirbot_interfaces__action__PositionAction_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} choirbot_interfaces__action__PositionAction_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHOIRBOT_INTERFACES__ACTION__DETAIL__POSITION_ACTION__STRUCT_H_
