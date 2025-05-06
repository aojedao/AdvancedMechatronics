// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from choirbot_interfaces:action/PositionAction.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "choirbot_interfaces/action/detail/position_action__rosidl_typesupport_introspection_c.h"
#include "choirbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "choirbot_interfaces/action/detail/position_action__functions.h"
#include "choirbot_interfaces/action/detail/position_action__struct.h"


// Include directives for member types
// Member `goal_position`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void PositionAction_Goal__rosidl_typesupport_introspection_c__PositionAction_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  choirbot_interfaces__action__PositionAction_Goal__init(message_memory);
}

void PositionAction_Goal__rosidl_typesupport_introspection_c__PositionAction_Goal_fini_function(void * message_memory)
{
  choirbot_interfaces__action__PositionAction_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember PositionAction_Goal__rosidl_typesupport_introspection_c__PositionAction_Goal_message_member_array[1] = {
  {
    "goal_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__action__PositionAction_Goal, goal_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PositionAction_Goal__rosidl_typesupport_introspection_c__PositionAction_Goal_message_members = {
  "choirbot_interfaces__action",  // message namespace
  "PositionAction_Goal",  // message name
  1,  // number of fields
  sizeof(choirbot_interfaces__action__PositionAction_Goal),
  PositionAction_Goal__rosidl_typesupport_introspection_c__PositionAction_Goal_message_member_array,  // message members
  PositionAction_Goal__rosidl_typesupport_introspection_c__PositionAction_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  PositionAction_Goal__rosidl_typesupport_introspection_c__PositionAction_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PositionAction_Goal__rosidl_typesupport_introspection_c__PositionAction_Goal_message_type_support_handle = {
  0,
  &PositionAction_Goal__rosidl_typesupport_introspection_c__PositionAction_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_choirbot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, action, PositionAction_Goal)() {
  if (!PositionAction_Goal__rosidl_typesupport_introspection_c__PositionAction_Goal_message_type_support_handle.typesupport_identifier) {
    PositionAction_Goal__rosidl_typesupport_introspection_c__PositionAction_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PositionAction_Goal__rosidl_typesupport_introspection_c__PositionAction_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "choirbot_interfaces/action/detail/position_action__rosidl_typesupport_introspection_c.h"
// already included above
// #include "choirbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "choirbot_interfaces/action/detail/position_action__functions.h"
// already included above
// #include "choirbot_interfaces/action/detail/position_action__struct.h"


// Include directives for member types
// Member `final_position`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void PositionAction_Result__rosidl_typesupport_introspection_c__PositionAction_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  choirbot_interfaces__action__PositionAction_Result__init(message_memory);
}

void PositionAction_Result__rosidl_typesupport_introspection_c__PositionAction_Result_fini_function(void * message_memory)
{
  choirbot_interfaces__action__PositionAction_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember PositionAction_Result__rosidl_typesupport_introspection_c__PositionAction_Result_message_member_array[1] = {
  {
    "final_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__action__PositionAction_Result, final_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PositionAction_Result__rosidl_typesupport_introspection_c__PositionAction_Result_message_members = {
  "choirbot_interfaces__action",  // message namespace
  "PositionAction_Result",  // message name
  1,  // number of fields
  sizeof(choirbot_interfaces__action__PositionAction_Result),
  PositionAction_Result__rosidl_typesupport_introspection_c__PositionAction_Result_message_member_array,  // message members
  PositionAction_Result__rosidl_typesupport_introspection_c__PositionAction_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  PositionAction_Result__rosidl_typesupport_introspection_c__PositionAction_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PositionAction_Result__rosidl_typesupport_introspection_c__PositionAction_Result_message_type_support_handle = {
  0,
  &PositionAction_Result__rosidl_typesupport_introspection_c__PositionAction_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_choirbot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, action, PositionAction_Result)() {
  if (!PositionAction_Result__rosidl_typesupport_introspection_c__PositionAction_Result_message_type_support_handle.typesupport_identifier) {
    PositionAction_Result__rosidl_typesupport_introspection_c__PositionAction_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PositionAction_Result__rosidl_typesupport_introspection_c__PositionAction_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "choirbot_interfaces/action/detail/position_action__rosidl_typesupport_introspection_c.h"
// already included above
// #include "choirbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "choirbot_interfaces/action/detail/position_action__functions.h"
// already included above
// #include "choirbot_interfaces/action/detail/position_action__struct.h"


// Include directives for member types
// Member `current_position`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void PositionAction_Feedback__rosidl_typesupport_introspection_c__PositionAction_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  choirbot_interfaces__action__PositionAction_Feedback__init(message_memory);
}

void PositionAction_Feedback__rosidl_typesupport_introspection_c__PositionAction_Feedback_fini_function(void * message_memory)
{
  choirbot_interfaces__action__PositionAction_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember PositionAction_Feedback__rosidl_typesupport_introspection_c__PositionAction_Feedback_message_member_array[2] = {
  {
    "current_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__action__PositionAction_Feedback, current_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__action__PositionAction_Feedback, distance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PositionAction_Feedback__rosidl_typesupport_introspection_c__PositionAction_Feedback_message_members = {
  "choirbot_interfaces__action",  // message namespace
  "PositionAction_Feedback",  // message name
  2,  // number of fields
  sizeof(choirbot_interfaces__action__PositionAction_Feedback),
  PositionAction_Feedback__rosidl_typesupport_introspection_c__PositionAction_Feedback_message_member_array,  // message members
  PositionAction_Feedback__rosidl_typesupport_introspection_c__PositionAction_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  PositionAction_Feedback__rosidl_typesupport_introspection_c__PositionAction_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PositionAction_Feedback__rosidl_typesupport_introspection_c__PositionAction_Feedback_message_type_support_handle = {
  0,
  &PositionAction_Feedback__rosidl_typesupport_introspection_c__PositionAction_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_choirbot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, action, PositionAction_Feedback)() {
  if (!PositionAction_Feedback__rosidl_typesupport_introspection_c__PositionAction_Feedback_message_type_support_handle.typesupport_identifier) {
    PositionAction_Feedback__rosidl_typesupport_introspection_c__PositionAction_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PositionAction_Feedback__rosidl_typesupport_introspection_c__PositionAction_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "choirbot_interfaces/action/detail/position_action__rosidl_typesupport_introspection_c.h"
// already included above
// #include "choirbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "choirbot_interfaces/action/detail/position_action__functions.h"
// already included above
// #include "choirbot_interfaces/action/detail/position_action__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "choirbot_interfaces/action/position_action.h"
// Member `goal`
// already included above
// #include "choirbot_interfaces/action/detail/position_action__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void PositionAction_SendGoal_Request__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  choirbot_interfaces__action__PositionAction_SendGoal_Request__init(message_memory);
}

void PositionAction_SendGoal_Request__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Request_fini_function(void * message_memory)
{
  choirbot_interfaces__action__PositionAction_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember PositionAction_SendGoal_Request__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__action__PositionAction_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__action__PositionAction_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PositionAction_SendGoal_Request__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Request_message_members = {
  "choirbot_interfaces__action",  // message namespace
  "PositionAction_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(choirbot_interfaces__action__PositionAction_SendGoal_Request),
  PositionAction_SendGoal_Request__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Request_message_member_array,  // message members
  PositionAction_SendGoal_Request__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  PositionAction_SendGoal_Request__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PositionAction_SendGoal_Request__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Request_message_type_support_handle = {
  0,
  &PositionAction_SendGoal_Request__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_choirbot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, action, PositionAction_SendGoal_Request)() {
  PositionAction_SendGoal_Request__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  PositionAction_SendGoal_Request__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, action, PositionAction_Goal)();
  if (!PositionAction_SendGoal_Request__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    PositionAction_SendGoal_Request__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PositionAction_SendGoal_Request__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "choirbot_interfaces/action/detail/position_action__rosidl_typesupport_introspection_c.h"
// already included above
// #include "choirbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "choirbot_interfaces/action/detail/position_action__functions.h"
// already included above
// #include "choirbot_interfaces/action/detail/position_action__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void PositionAction_SendGoal_Response__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  choirbot_interfaces__action__PositionAction_SendGoal_Response__init(message_memory);
}

void PositionAction_SendGoal_Response__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Response_fini_function(void * message_memory)
{
  choirbot_interfaces__action__PositionAction_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember PositionAction_SendGoal_Response__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__action__PositionAction_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__action__PositionAction_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PositionAction_SendGoal_Response__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Response_message_members = {
  "choirbot_interfaces__action",  // message namespace
  "PositionAction_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(choirbot_interfaces__action__PositionAction_SendGoal_Response),
  PositionAction_SendGoal_Response__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Response_message_member_array,  // message members
  PositionAction_SendGoal_Response__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  PositionAction_SendGoal_Response__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PositionAction_SendGoal_Response__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Response_message_type_support_handle = {
  0,
  &PositionAction_SendGoal_Response__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_choirbot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, action, PositionAction_SendGoal_Response)() {
  PositionAction_SendGoal_Response__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!PositionAction_SendGoal_Response__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    PositionAction_SendGoal_Response__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PositionAction_SendGoal_Response__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "choirbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "choirbot_interfaces/action/detail/position_action__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers choirbot_interfaces__action__detail__position_action__rosidl_typesupport_introspection_c__PositionAction_SendGoal_service_members = {
  "choirbot_interfaces__action",  // service namespace
  "PositionAction_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // choirbot_interfaces__action__detail__position_action__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // choirbot_interfaces__action__detail__position_action__rosidl_typesupport_introspection_c__PositionAction_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t choirbot_interfaces__action__detail__position_action__rosidl_typesupport_introspection_c__PositionAction_SendGoal_service_type_support_handle = {
  0,
  &choirbot_interfaces__action__detail__position_action__rosidl_typesupport_introspection_c__PositionAction_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, action, PositionAction_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, action, PositionAction_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_choirbot_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, action, PositionAction_SendGoal)() {
  if (!choirbot_interfaces__action__detail__position_action__rosidl_typesupport_introspection_c__PositionAction_SendGoal_service_type_support_handle.typesupport_identifier) {
    choirbot_interfaces__action__detail__position_action__rosidl_typesupport_introspection_c__PositionAction_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)choirbot_interfaces__action__detail__position_action__rosidl_typesupport_introspection_c__PositionAction_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, action, PositionAction_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, action, PositionAction_SendGoal_Response)()->data;
  }

  return &choirbot_interfaces__action__detail__position_action__rosidl_typesupport_introspection_c__PositionAction_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "choirbot_interfaces/action/detail/position_action__rosidl_typesupport_introspection_c.h"
// already included above
// #include "choirbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "choirbot_interfaces/action/detail/position_action__functions.h"
// already included above
// #include "choirbot_interfaces/action/detail/position_action__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void PositionAction_GetResult_Request__rosidl_typesupport_introspection_c__PositionAction_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  choirbot_interfaces__action__PositionAction_GetResult_Request__init(message_memory);
}

void PositionAction_GetResult_Request__rosidl_typesupport_introspection_c__PositionAction_GetResult_Request_fini_function(void * message_memory)
{
  choirbot_interfaces__action__PositionAction_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember PositionAction_GetResult_Request__rosidl_typesupport_introspection_c__PositionAction_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__action__PositionAction_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PositionAction_GetResult_Request__rosidl_typesupport_introspection_c__PositionAction_GetResult_Request_message_members = {
  "choirbot_interfaces__action",  // message namespace
  "PositionAction_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(choirbot_interfaces__action__PositionAction_GetResult_Request),
  PositionAction_GetResult_Request__rosidl_typesupport_introspection_c__PositionAction_GetResult_Request_message_member_array,  // message members
  PositionAction_GetResult_Request__rosidl_typesupport_introspection_c__PositionAction_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  PositionAction_GetResult_Request__rosidl_typesupport_introspection_c__PositionAction_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PositionAction_GetResult_Request__rosidl_typesupport_introspection_c__PositionAction_GetResult_Request_message_type_support_handle = {
  0,
  &PositionAction_GetResult_Request__rosidl_typesupport_introspection_c__PositionAction_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_choirbot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, action, PositionAction_GetResult_Request)() {
  PositionAction_GetResult_Request__rosidl_typesupport_introspection_c__PositionAction_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!PositionAction_GetResult_Request__rosidl_typesupport_introspection_c__PositionAction_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    PositionAction_GetResult_Request__rosidl_typesupport_introspection_c__PositionAction_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PositionAction_GetResult_Request__rosidl_typesupport_introspection_c__PositionAction_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "choirbot_interfaces/action/detail/position_action__rosidl_typesupport_introspection_c.h"
// already included above
// #include "choirbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "choirbot_interfaces/action/detail/position_action__functions.h"
// already included above
// #include "choirbot_interfaces/action/detail/position_action__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "choirbot_interfaces/action/position_action.h"
// Member `result`
// already included above
// #include "choirbot_interfaces/action/detail/position_action__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void PositionAction_GetResult_Response__rosidl_typesupport_introspection_c__PositionAction_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  choirbot_interfaces__action__PositionAction_GetResult_Response__init(message_memory);
}

void PositionAction_GetResult_Response__rosidl_typesupport_introspection_c__PositionAction_GetResult_Response_fini_function(void * message_memory)
{
  choirbot_interfaces__action__PositionAction_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember PositionAction_GetResult_Response__rosidl_typesupport_introspection_c__PositionAction_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__action__PositionAction_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__action__PositionAction_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PositionAction_GetResult_Response__rosidl_typesupport_introspection_c__PositionAction_GetResult_Response_message_members = {
  "choirbot_interfaces__action",  // message namespace
  "PositionAction_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(choirbot_interfaces__action__PositionAction_GetResult_Response),
  PositionAction_GetResult_Response__rosidl_typesupport_introspection_c__PositionAction_GetResult_Response_message_member_array,  // message members
  PositionAction_GetResult_Response__rosidl_typesupport_introspection_c__PositionAction_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  PositionAction_GetResult_Response__rosidl_typesupport_introspection_c__PositionAction_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PositionAction_GetResult_Response__rosidl_typesupport_introspection_c__PositionAction_GetResult_Response_message_type_support_handle = {
  0,
  &PositionAction_GetResult_Response__rosidl_typesupport_introspection_c__PositionAction_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_choirbot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, action, PositionAction_GetResult_Response)() {
  PositionAction_GetResult_Response__rosidl_typesupport_introspection_c__PositionAction_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, action, PositionAction_Result)();
  if (!PositionAction_GetResult_Response__rosidl_typesupport_introspection_c__PositionAction_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    PositionAction_GetResult_Response__rosidl_typesupport_introspection_c__PositionAction_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PositionAction_GetResult_Response__rosidl_typesupport_introspection_c__PositionAction_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "choirbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "choirbot_interfaces/action/detail/position_action__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers choirbot_interfaces__action__detail__position_action__rosidl_typesupport_introspection_c__PositionAction_GetResult_service_members = {
  "choirbot_interfaces__action",  // service namespace
  "PositionAction_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // choirbot_interfaces__action__detail__position_action__rosidl_typesupport_introspection_c__PositionAction_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // choirbot_interfaces__action__detail__position_action__rosidl_typesupport_introspection_c__PositionAction_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t choirbot_interfaces__action__detail__position_action__rosidl_typesupport_introspection_c__PositionAction_GetResult_service_type_support_handle = {
  0,
  &choirbot_interfaces__action__detail__position_action__rosidl_typesupport_introspection_c__PositionAction_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, action, PositionAction_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, action, PositionAction_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_choirbot_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, action, PositionAction_GetResult)() {
  if (!choirbot_interfaces__action__detail__position_action__rosidl_typesupport_introspection_c__PositionAction_GetResult_service_type_support_handle.typesupport_identifier) {
    choirbot_interfaces__action__detail__position_action__rosidl_typesupport_introspection_c__PositionAction_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)choirbot_interfaces__action__detail__position_action__rosidl_typesupport_introspection_c__PositionAction_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, action, PositionAction_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, action, PositionAction_GetResult_Response)()->data;
  }

  return &choirbot_interfaces__action__detail__position_action__rosidl_typesupport_introspection_c__PositionAction_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "choirbot_interfaces/action/detail/position_action__rosidl_typesupport_introspection_c.h"
// already included above
// #include "choirbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "choirbot_interfaces/action/detail/position_action__functions.h"
// already included above
// #include "choirbot_interfaces/action/detail/position_action__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "choirbot_interfaces/action/position_action.h"
// Member `feedback`
// already included above
// #include "choirbot_interfaces/action/detail/position_action__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void PositionAction_FeedbackMessage__rosidl_typesupport_introspection_c__PositionAction_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  choirbot_interfaces__action__PositionAction_FeedbackMessage__init(message_memory);
}

void PositionAction_FeedbackMessage__rosidl_typesupport_introspection_c__PositionAction_FeedbackMessage_fini_function(void * message_memory)
{
  choirbot_interfaces__action__PositionAction_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember PositionAction_FeedbackMessage__rosidl_typesupport_introspection_c__PositionAction_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__action__PositionAction_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__action__PositionAction_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PositionAction_FeedbackMessage__rosidl_typesupport_introspection_c__PositionAction_FeedbackMessage_message_members = {
  "choirbot_interfaces__action",  // message namespace
  "PositionAction_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(choirbot_interfaces__action__PositionAction_FeedbackMessage),
  PositionAction_FeedbackMessage__rosidl_typesupport_introspection_c__PositionAction_FeedbackMessage_message_member_array,  // message members
  PositionAction_FeedbackMessage__rosidl_typesupport_introspection_c__PositionAction_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  PositionAction_FeedbackMessage__rosidl_typesupport_introspection_c__PositionAction_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PositionAction_FeedbackMessage__rosidl_typesupport_introspection_c__PositionAction_FeedbackMessage_message_type_support_handle = {
  0,
  &PositionAction_FeedbackMessage__rosidl_typesupport_introspection_c__PositionAction_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_choirbot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, action, PositionAction_FeedbackMessage)() {
  PositionAction_FeedbackMessage__rosidl_typesupport_introspection_c__PositionAction_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  PositionAction_FeedbackMessage__rosidl_typesupport_introspection_c__PositionAction_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, action, PositionAction_Feedback)();
  if (!PositionAction_FeedbackMessage__rosidl_typesupport_introspection_c__PositionAction_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    PositionAction_FeedbackMessage__rosidl_typesupport_introspection_c__PositionAction_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PositionAction_FeedbackMessage__rosidl_typesupport_introspection_c__PositionAction_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
