// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from choirbot_interfaces:msg/PositionTaskArray.idl
// generated code does not contain a copyright notice

#ifndef CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK_ARRAY__STRUCT_HPP_
#define CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK_ARRAY__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'tasks'
#include "choirbot_interfaces/msg/detail/position_task__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__choirbot_interfaces__msg__PositionTaskArray __attribute__((deprecated))
#else
# define DEPRECATED__choirbot_interfaces__msg__PositionTaskArray __declspec(deprecated)
#endif

namespace choirbot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PositionTaskArray_
{
  using Type = PositionTaskArray_<ContainerAllocator>;

  explicit PositionTaskArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->all_tasks_count = 0ul;
      this->label = 0ul;
    }
  }

  explicit PositionTaskArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->all_tasks_count = 0ul;
      this->label = 0ul;
    }
  }

  // field types and members
  using _tasks_type =
    std::vector<choirbot_interfaces::msg::PositionTask_<ContainerAllocator>, typename ContainerAllocator::template rebind<choirbot_interfaces::msg::PositionTask_<ContainerAllocator>>::other>;
  _tasks_type tasks;
  using _all_tasks_count_type =
    uint32_t;
  _all_tasks_count_type all_tasks_count;
  using _label_type =
    uint32_t;
  _label_type label;

  // setters for named parameter idiom
  Type & set__tasks(
    const std::vector<choirbot_interfaces::msg::PositionTask_<ContainerAllocator>, typename ContainerAllocator::template rebind<choirbot_interfaces::msg::PositionTask_<ContainerAllocator>>::other> & _arg)
  {
    this->tasks = _arg;
    return *this;
  }
  Type & set__all_tasks_count(
    const uint32_t & _arg)
  {
    this->all_tasks_count = _arg;
    return *this;
  }
  Type & set__label(
    const uint32_t & _arg)
  {
    this->label = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    choirbot_interfaces::msg::PositionTaskArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const choirbot_interfaces::msg::PositionTaskArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<choirbot_interfaces::msg::PositionTaskArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<choirbot_interfaces::msg::PositionTaskArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      choirbot_interfaces::msg::PositionTaskArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<choirbot_interfaces::msg::PositionTaskArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      choirbot_interfaces::msg::PositionTaskArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<choirbot_interfaces::msg::PositionTaskArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<choirbot_interfaces::msg::PositionTaskArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<choirbot_interfaces::msg::PositionTaskArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__choirbot_interfaces__msg__PositionTaskArray
    std::shared_ptr<choirbot_interfaces::msg::PositionTaskArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__choirbot_interfaces__msg__PositionTaskArray
    std::shared_ptr<choirbot_interfaces::msg::PositionTaskArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PositionTaskArray_ & other) const
  {
    if (this->tasks != other.tasks) {
      return false;
    }
    if (this->all_tasks_count != other.all_tasks_count) {
      return false;
    }
    if (this->label != other.label) {
      return false;
    }
    return true;
  }
  bool operator!=(const PositionTaskArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PositionTaskArray_

// alias to use template instance with default allocator
using PositionTaskArray =
  choirbot_interfaces::msg::PositionTaskArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace choirbot_interfaces

#endif  // CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK_ARRAY__STRUCT_HPP_
