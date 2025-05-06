// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from choirbot_interfaces:srv/PositionTaskService.idl
// generated code does not contain a copyright notice

#ifndef CHOIRBOT_INTERFACES__SRV__DETAIL__POSITION_TASK_SERVICE__STRUCT_HPP_
#define CHOIRBOT_INTERFACES__SRV__DETAIL__POSITION_TASK_SERVICE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__choirbot_interfaces__srv__PositionTaskService_Request __attribute__((deprecated))
#else
# define DEPRECATED__choirbot_interfaces__srv__PositionTaskService_Request __declspec(deprecated)
#endif

namespace choirbot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PositionTaskService_Request_
{
  using Type = PositionTaskService_Request_<ContainerAllocator>;

  explicit PositionTaskService_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->agent_id = 0ul;
    }
  }

  explicit PositionTaskService_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->agent_id = 0ul;
    }
  }

  // field types and members
  using _agent_id_type =
    uint32_t;
  _agent_id_type agent_id;

  // setters for named parameter idiom
  Type & set__agent_id(
    const uint32_t & _arg)
  {
    this->agent_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    choirbot_interfaces::srv::PositionTaskService_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const choirbot_interfaces::srv::PositionTaskService_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<choirbot_interfaces::srv::PositionTaskService_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<choirbot_interfaces::srv::PositionTaskService_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      choirbot_interfaces::srv::PositionTaskService_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<choirbot_interfaces::srv::PositionTaskService_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      choirbot_interfaces::srv::PositionTaskService_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<choirbot_interfaces::srv::PositionTaskService_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<choirbot_interfaces::srv::PositionTaskService_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<choirbot_interfaces::srv::PositionTaskService_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__choirbot_interfaces__srv__PositionTaskService_Request
    std::shared_ptr<choirbot_interfaces::srv::PositionTaskService_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__choirbot_interfaces__srv__PositionTaskService_Request
    std::shared_ptr<choirbot_interfaces::srv::PositionTaskService_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PositionTaskService_Request_ & other) const
  {
    if (this->agent_id != other.agent_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const PositionTaskService_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PositionTaskService_Request_

// alias to use template instance with default allocator
using PositionTaskService_Request =
  choirbot_interfaces::srv::PositionTaskService_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace choirbot_interfaces


// Include directives for member types
// Member 'tasks'
#include "choirbot_interfaces/msg/detail/position_task_array__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__choirbot_interfaces__srv__PositionTaskService_Response __attribute__((deprecated))
#else
# define DEPRECATED__choirbot_interfaces__srv__PositionTaskService_Response __declspec(deprecated)
#endif

namespace choirbot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PositionTaskService_Response_
{
  using Type = PositionTaskService_Response_<ContainerAllocator>;

  explicit PositionTaskService_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : tasks(_init)
  {
    (void)_init;
  }

  explicit PositionTaskService_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : tasks(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _tasks_type =
    choirbot_interfaces::msg::PositionTaskArray_<ContainerAllocator>;
  _tasks_type tasks;

  // setters for named parameter idiom
  Type & set__tasks(
    const choirbot_interfaces::msg::PositionTaskArray_<ContainerAllocator> & _arg)
  {
    this->tasks = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    choirbot_interfaces::srv::PositionTaskService_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const choirbot_interfaces::srv::PositionTaskService_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<choirbot_interfaces::srv::PositionTaskService_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<choirbot_interfaces::srv::PositionTaskService_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      choirbot_interfaces::srv::PositionTaskService_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<choirbot_interfaces::srv::PositionTaskService_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      choirbot_interfaces::srv::PositionTaskService_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<choirbot_interfaces::srv::PositionTaskService_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<choirbot_interfaces::srv::PositionTaskService_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<choirbot_interfaces::srv::PositionTaskService_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__choirbot_interfaces__srv__PositionTaskService_Response
    std::shared_ptr<choirbot_interfaces::srv::PositionTaskService_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__choirbot_interfaces__srv__PositionTaskService_Response
    std::shared_ptr<choirbot_interfaces::srv::PositionTaskService_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PositionTaskService_Response_ & other) const
  {
    if (this->tasks != other.tasks) {
      return false;
    }
    return true;
  }
  bool operator!=(const PositionTaskService_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PositionTaskService_Response_

// alias to use template instance with default allocator
using PositionTaskService_Response =
  choirbot_interfaces::srv::PositionTaskService_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace choirbot_interfaces

namespace choirbot_interfaces
{

namespace srv
{

struct PositionTaskService
{
  using Request = choirbot_interfaces::srv::PositionTaskService_Request;
  using Response = choirbot_interfaces::srv::PositionTaskService_Response;
};

}  // namespace srv

}  // namespace choirbot_interfaces

#endif  // CHOIRBOT_INTERFACES__SRV__DETAIL__POSITION_TASK_SERVICE__STRUCT_HPP_
