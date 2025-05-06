// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from choirbot_interfaces:srv/TaskCompletionService.idl
// generated code does not contain a copyright notice

#ifndef CHOIRBOT_INTERFACES__SRV__DETAIL__TASK_COMPLETION_SERVICE__STRUCT_HPP_
#define CHOIRBOT_INTERFACES__SRV__DETAIL__TASK_COMPLETION_SERVICE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__choirbot_interfaces__srv__TaskCompletionService_Request __attribute__((deprecated))
#else
# define DEPRECATED__choirbot_interfaces__srv__TaskCompletionService_Request __declspec(deprecated)
#endif

namespace choirbot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TaskCompletionService_Request_
{
  using Type = TaskCompletionService_Request_<ContainerAllocator>;

  explicit TaskCompletionService_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->agent_id = 0ul;
      this->task_seq_num = 0ul;
    }
  }

  explicit TaskCompletionService_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->agent_id = 0ul;
      this->task_seq_num = 0ul;
    }
  }

  // field types and members
  using _agent_id_type =
    uint32_t;
  _agent_id_type agent_id;
  using _task_seq_num_type =
    uint32_t;
  _task_seq_num_type task_seq_num;

  // setters for named parameter idiom
  Type & set__agent_id(
    const uint32_t & _arg)
  {
    this->agent_id = _arg;
    return *this;
  }
  Type & set__task_seq_num(
    const uint32_t & _arg)
  {
    this->task_seq_num = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    choirbot_interfaces::srv::TaskCompletionService_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const choirbot_interfaces::srv::TaskCompletionService_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<choirbot_interfaces::srv::TaskCompletionService_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<choirbot_interfaces::srv::TaskCompletionService_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      choirbot_interfaces::srv::TaskCompletionService_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<choirbot_interfaces::srv::TaskCompletionService_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      choirbot_interfaces::srv::TaskCompletionService_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<choirbot_interfaces::srv::TaskCompletionService_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<choirbot_interfaces::srv::TaskCompletionService_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<choirbot_interfaces::srv::TaskCompletionService_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__choirbot_interfaces__srv__TaskCompletionService_Request
    std::shared_ptr<choirbot_interfaces::srv::TaskCompletionService_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__choirbot_interfaces__srv__TaskCompletionService_Request
    std::shared_ptr<choirbot_interfaces::srv::TaskCompletionService_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TaskCompletionService_Request_ & other) const
  {
    if (this->agent_id != other.agent_id) {
      return false;
    }
    if (this->task_seq_num != other.task_seq_num) {
      return false;
    }
    return true;
  }
  bool operator!=(const TaskCompletionService_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TaskCompletionService_Request_

// alias to use template instance with default allocator
using TaskCompletionService_Request =
  choirbot_interfaces::srv::TaskCompletionService_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace choirbot_interfaces


#ifndef _WIN32
# define DEPRECATED__choirbot_interfaces__srv__TaskCompletionService_Response __attribute__((deprecated))
#else
# define DEPRECATED__choirbot_interfaces__srv__TaskCompletionService_Response __declspec(deprecated)
#endif

namespace choirbot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TaskCompletionService_Response_
{
  using Type = TaskCompletionService_Response_<ContainerAllocator>;

  explicit TaskCompletionService_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit TaskCompletionService_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    choirbot_interfaces::srv::TaskCompletionService_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const choirbot_interfaces::srv::TaskCompletionService_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<choirbot_interfaces::srv::TaskCompletionService_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<choirbot_interfaces::srv::TaskCompletionService_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      choirbot_interfaces::srv::TaskCompletionService_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<choirbot_interfaces::srv::TaskCompletionService_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      choirbot_interfaces::srv::TaskCompletionService_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<choirbot_interfaces::srv::TaskCompletionService_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<choirbot_interfaces::srv::TaskCompletionService_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<choirbot_interfaces::srv::TaskCompletionService_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__choirbot_interfaces__srv__TaskCompletionService_Response
    std::shared_ptr<choirbot_interfaces::srv::TaskCompletionService_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__choirbot_interfaces__srv__TaskCompletionService_Response
    std::shared_ptr<choirbot_interfaces::srv::TaskCompletionService_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TaskCompletionService_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const TaskCompletionService_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TaskCompletionService_Response_

// alias to use template instance with default allocator
using TaskCompletionService_Response =
  choirbot_interfaces::srv::TaskCompletionService_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace choirbot_interfaces

namespace choirbot_interfaces
{

namespace srv
{

struct TaskCompletionService
{
  using Request = choirbot_interfaces::srv::TaskCompletionService_Request;
  using Response = choirbot_interfaces::srv::TaskCompletionService_Response;
};

}  // namespace srv

}  // namespace choirbot_interfaces

#endif  // CHOIRBOT_INTERFACES__SRV__DETAIL__TASK_COMPLETION_SERVICE__STRUCT_HPP_
