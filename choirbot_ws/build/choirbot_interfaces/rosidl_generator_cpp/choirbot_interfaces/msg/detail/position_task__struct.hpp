// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from choirbot_interfaces:msg/PositionTask.idl
// generated code does not contain a copyright notice

#ifndef CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK__STRUCT_HPP_
#define CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__choirbot_interfaces__msg__PositionTask __attribute__((deprecated))
#else
# define DEPRECATED__choirbot_interfaces__msg__PositionTask __declspec(deprecated)
#endif

namespace choirbot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PositionTask_
{
  using Type = PositionTask_<ContainerAllocator>;

  explicit PositionTask_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ul;
      this->seq_num = 0ul;
    }
  }

  explicit PositionTask_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ul;
      this->seq_num = 0ul;
    }
  }

  // field types and members
  using _coordinates_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _coordinates_type coordinates;
  using _id_type =
    uint32_t;
  _id_type id;
  using _seq_num_type =
    uint32_t;
  _seq_num_type seq_num;

  // setters for named parameter idiom
  Type & set__coordinates(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->coordinates = _arg;
    return *this;
  }
  Type & set__id(
    const uint32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__seq_num(
    const uint32_t & _arg)
  {
    this->seq_num = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    choirbot_interfaces::msg::PositionTask_<ContainerAllocator> *;
  using ConstRawPtr =
    const choirbot_interfaces::msg::PositionTask_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<choirbot_interfaces::msg::PositionTask_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<choirbot_interfaces::msg::PositionTask_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      choirbot_interfaces::msg::PositionTask_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<choirbot_interfaces::msg::PositionTask_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      choirbot_interfaces::msg::PositionTask_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<choirbot_interfaces::msg::PositionTask_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<choirbot_interfaces::msg::PositionTask_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<choirbot_interfaces::msg::PositionTask_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__choirbot_interfaces__msg__PositionTask
    std::shared_ptr<choirbot_interfaces::msg::PositionTask_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__choirbot_interfaces__msg__PositionTask
    std::shared_ptr<choirbot_interfaces::msg::PositionTask_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PositionTask_ & other) const
  {
    if (this->coordinates != other.coordinates) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->seq_num != other.seq_num) {
      return false;
    }
    return true;
  }
  bool operator!=(const PositionTask_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PositionTask_

// alias to use template instance with default allocator
using PositionTask =
  choirbot_interfaces::msg::PositionTask_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace choirbot_interfaces

#endif  // CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK__STRUCT_HPP_
