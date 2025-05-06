// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from choirbot_interfaces:srv/PositionTaskService.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "choirbot_interfaces/srv/detail/position_task_service__rosidl_typesupport_introspection_c.h"
#include "choirbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "choirbot_interfaces/srv/detail/position_task_service__functions.h"
#include "choirbot_interfaces/srv/detail/position_task_service__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void PositionTaskService_Request__rosidl_typesupport_introspection_c__PositionTaskService_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  choirbot_interfaces__srv__PositionTaskService_Request__init(message_memory);
}

void PositionTaskService_Request__rosidl_typesupport_introspection_c__PositionTaskService_Request_fini_function(void * message_memory)
{
  choirbot_interfaces__srv__PositionTaskService_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember PositionTaskService_Request__rosidl_typesupport_introspection_c__PositionTaskService_Request_message_member_array[1] = {
  {
    "agent_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__srv__PositionTaskService_Request, agent_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PositionTaskService_Request__rosidl_typesupport_introspection_c__PositionTaskService_Request_message_members = {
  "choirbot_interfaces__srv",  // message namespace
  "PositionTaskService_Request",  // message name
  1,  // number of fields
  sizeof(choirbot_interfaces__srv__PositionTaskService_Request),
  PositionTaskService_Request__rosidl_typesupport_introspection_c__PositionTaskService_Request_message_member_array,  // message members
  PositionTaskService_Request__rosidl_typesupport_introspection_c__PositionTaskService_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  PositionTaskService_Request__rosidl_typesupport_introspection_c__PositionTaskService_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PositionTaskService_Request__rosidl_typesupport_introspection_c__PositionTaskService_Request_message_type_support_handle = {
  0,
  &PositionTaskService_Request__rosidl_typesupport_introspection_c__PositionTaskService_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_choirbot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, srv, PositionTaskService_Request)() {
  if (!PositionTaskService_Request__rosidl_typesupport_introspection_c__PositionTaskService_Request_message_type_support_handle.typesupport_identifier) {
    PositionTaskService_Request__rosidl_typesupport_introspection_c__PositionTaskService_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PositionTaskService_Request__rosidl_typesupport_introspection_c__PositionTaskService_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "choirbot_interfaces/srv/detail/position_task_service__rosidl_typesupport_introspection_c.h"
// already included above
// #include "choirbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "choirbot_interfaces/srv/detail/position_task_service__functions.h"
// already included above
// #include "choirbot_interfaces/srv/detail/position_task_service__struct.h"


// Include directives for member types
// Member `tasks`
#include "choirbot_interfaces/msg/position_task_array.h"
// Member `tasks`
#include "choirbot_interfaces/msg/detail/position_task_array__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void PositionTaskService_Response__rosidl_typesupport_introspection_c__PositionTaskService_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  choirbot_interfaces__srv__PositionTaskService_Response__init(message_memory);
}

void PositionTaskService_Response__rosidl_typesupport_introspection_c__PositionTaskService_Response_fini_function(void * message_memory)
{
  choirbot_interfaces__srv__PositionTaskService_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember PositionTaskService_Response__rosidl_typesupport_introspection_c__PositionTaskService_Response_message_member_array[1] = {
  {
    "tasks",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__srv__PositionTaskService_Response, tasks),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PositionTaskService_Response__rosidl_typesupport_introspection_c__PositionTaskService_Response_message_members = {
  "choirbot_interfaces__srv",  // message namespace
  "PositionTaskService_Response",  // message name
  1,  // number of fields
  sizeof(choirbot_interfaces__srv__PositionTaskService_Response),
  PositionTaskService_Response__rosidl_typesupport_introspection_c__PositionTaskService_Response_message_member_array,  // message members
  PositionTaskService_Response__rosidl_typesupport_introspection_c__PositionTaskService_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  PositionTaskService_Response__rosidl_typesupport_introspection_c__PositionTaskService_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PositionTaskService_Response__rosidl_typesupport_introspection_c__PositionTaskService_Response_message_type_support_handle = {
  0,
  &PositionTaskService_Response__rosidl_typesupport_introspection_c__PositionTaskService_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_choirbot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, srv, PositionTaskService_Response)() {
  PositionTaskService_Response__rosidl_typesupport_introspection_c__PositionTaskService_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, msg, PositionTaskArray)();
  if (!PositionTaskService_Response__rosidl_typesupport_introspection_c__PositionTaskService_Response_message_type_support_handle.typesupport_identifier) {
    PositionTaskService_Response__rosidl_typesupport_introspection_c__PositionTaskService_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PositionTaskService_Response__rosidl_typesupport_introspection_c__PositionTaskService_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "choirbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "choirbot_interfaces/srv/detail/position_task_service__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers choirbot_interfaces__srv__detail__position_task_service__rosidl_typesupport_introspection_c__PositionTaskService_service_members = {
  "choirbot_interfaces__srv",  // service namespace
  "PositionTaskService",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // choirbot_interfaces__srv__detail__position_task_service__rosidl_typesupport_introspection_c__PositionTaskService_Request_message_type_support_handle,
  NULL  // response message
  // choirbot_interfaces__srv__detail__position_task_service__rosidl_typesupport_introspection_c__PositionTaskService_Response_message_type_support_handle
};

static rosidl_service_type_support_t choirbot_interfaces__srv__detail__position_task_service__rosidl_typesupport_introspection_c__PositionTaskService_service_type_support_handle = {
  0,
  &choirbot_interfaces__srv__detail__position_task_service__rosidl_typesupport_introspection_c__PositionTaskService_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, srv, PositionTaskService_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, srv, PositionTaskService_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_choirbot_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, srv, PositionTaskService)() {
  if (!choirbot_interfaces__srv__detail__position_task_service__rosidl_typesupport_introspection_c__PositionTaskService_service_type_support_handle.typesupport_identifier) {
    choirbot_interfaces__srv__detail__position_task_service__rosidl_typesupport_introspection_c__PositionTaskService_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)choirbot_interfaces__srv__detail__position_task_service__rosidl_typesupport_introspection_c__PositionTaskService_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, srv, PositionTaskService_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, srv, PositionTaskService_Response)()->data;
  }

  return &choirbot_interfaces__srv__detail__position_task_service__rosidl_typesupport_introspection_c__PositionTaskService_service_type_support_handle;
}
