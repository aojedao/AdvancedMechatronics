// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from choirbot_interfaces:srv/TaskCompletionService.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "choirbot_interfaces/srv/detail/task_completion_service__rosidl_typesupport_introspection_c.h"
#include "choirbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "choirbot_interfaces/srv/detail/task_completion_service__functions.h"
#include "choirbot_interfaces/srv/detail/task_completion_service__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void TaskCompletionService_Request__rosidl_typesupport_introspection_c__TaskCompletionService_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  choirbot_interfaces__srv__TaskCompletionService_Request__init(message_memory);
}

void TaskCompletionService_Request__rosidl_typesupport_introspection_c__TaskCompletionService_Request_fini_function(void * message_memory)
{
  choirbot_interfaces__srv__TaskCompletionService_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TaskCompletionService_Request__rosidl_typesupport_introspection_c__TaskCompletionService_Request_message_member_array[2] = {
  {
    "agent_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__srv__TaskCompletionService_Request, agent_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "task_seq_num",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__srv__TaskCompletionService_Request, task_seq_num),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TaskCompletionService_Request__rosidl_typesupport_introspection_c__TaskCompletionService_Request_message_members = {
  "choirbot_interfaces__srv",  // message namespace
  "TaskCompletionService_Request",  // message name
  2,  // number of fields
  sizeof(choirbot_interfaces__srv__TaskCompletionService_Request),
  TaskCompletionService_Request__rosidl_typesupport_introspection_c__TaskCompletionService_Request_message_member_array,  // message members
  TaskCompletionService_Request__rosidl_typesupport_introspection_c__TaskCompletionService_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  TaskCompletionService_Request__rosidl_typesupport_introspection_c__TaskCompletionService_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TaskCompletionService_Request__rosidl_typesupport_introspection_c__TaskCompletionService_Request_message_type_support_handle = {
  0,
  &TaskCompletionService_Request__rosidl_typesupport_introspection_c__TaskCompletionService_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_choirbot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, srv, TaskCompletionService_Request)() {
  if (!TaskCompletionService_Request__rosidl_typesupport_introspection_c__TaskCompletionService_Request_message_type_support_handle.typesupport_identifier) {
    TaskCompletionService_Request__rosidl_typesupport_introspection_c__TaskCompletionService_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TaskCompletionService_Request__rosidl_typesupport_introspection_c__TaskCompletionService_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "choirbot_interfaces/srv/detail/task_completion_service__rosidl_typesupport_introspection_c.h"
// already included above
// #include "choirbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "choirbot_interfaces/srv/detail/task_completion_service__functions.h"
// already included above
// #include "choirbot_interfaces/srv/detail/task_completion_service__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void TaskCompletionService_Response__rosidl_typesupport_introspection_c__TaskCompletionService_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  choirbot_interfaces__srv__TaskCompletionService_Response__init(message_memory);
}

void TaskCompletionService_Response__rosidl_typesupport_introspection_c__TaskCompletionService_Response_fini_function(void * message_memory)
{
  choirbot_interfaces__srv__TaskCompletionService_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TaskCompletionService_Response__rosidl_typesupport_introspection_c__TaskCompletionService_Response_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(choirbot_interfaces__srv__TaskCompletionService_Response, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TaskCompletionService_Response__rosidl_typesupport_introspection_c__TaskCompletionService_Response_message_members = {
  "choirbot_interfaces__srv",  // message namespace
  "TaskCompletionService_Response",  // message name
  1,  // number of fields
  sizeof(choirbot_interfaces__srv__TaskCompletionService_Response),
  TaskCompletionService_Response__rosidl_typesupport_introspection_c__TaskCompletionService_Response_message_member_array,  // message members
  TaskCompletionService_Response__rosidl_typesupport_introspection_c__TaskCompletionService_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  TaskCompletionService_Response__rosidl_typesupport_introspection_c__TaskCompletionService_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TaskCompletionService_Response__rosidl_typesupport_introspection_c__TaskCompletionService_Response_message_type_support_handle = {
  0,
  &TaskCompletionService_Response__rosidl_typesupport_introspection_c__TaskCompletionService_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_choirbot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, srv, TaskCompletionService_Response)() {
  if (!TaskCompletionService_Response__rosidl_typesupport_introspection_c__TaskCompletionService_Response_message_type_support_handle.typesupport_identifier) {
    TaskCompletionService_Response__rosidl_typesupport_introspection_c__TaskCompletionService_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TaskCompletionService_Response__rosidl_typesupport_introspection_c__TaskCompletionService_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "choirbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "choirbot_interfaces/srv/detail/task_completion_service__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers choirbot_interfaces__srv__detail__task_completion_service__rosidl_typesupport_introspection_c__TaskCompletionService_service_members = {
  "choirbot_interfaces__srv",  // service namespace
  "TaskCompletionService",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // choirbot_interfaces__srv__detail__task_completion_service__rosidl_typesupport_introspection_c__TaskCompletionService_Request_message_type_support_handle,
  NULL  // response message
  // choirbot_interfaces__srv__detail__task_completion_service__rosidl_typesupport_introspection_c__TaskCompletionService_Response_message_type_support_handle
};

static rosidl_service_type_support_t choirbot_interfaces__srv__detail__task_completion_service__rosidl_typesupport_introspection_c__TaskCompletionService_service_type_support_handle = {
  0,
  &choirbot_interfaces__srv__detail__task_completion_service__rosidl_typesupport_introspection_c__TaskCompletionService_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, srv, TaskCompletionService_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, srv, TaskCompletionService_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_choirbot_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, srv, TaskCompletionService)() {
  if (!choirbot_interfaces__srv__detail__task_completion_service__rosidl_typesupport_introspection_c__TaskCompletionService_service_type_support_handle.typesupport_identifier) {
    choirbot_interfaces__srv__detail__task_completion_service__rosidl_typesupport_introspection_c__TaskCompletionService_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)choirbot_interfaces__srv__detail__task_completion_service__rosidl_typesupport_introspection_c__TaskCompletionService_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, srv, TaskCompletionService_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, choirbot_interfaces, srv, TaskCompletionService_Response)()->data;
  }

  return &choirbot_interfaces__srv__detail__task_completion_service__rosidl_typesupport_introspection_c__TaskCompletionService_service_type_support_handle;
}
