// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robp_interfaces:msg/ObjectPosition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robp_interfaces/msg/object_position.hpp"


#ifndef ROBP_INTERFACES__MSG__DETAIL__OBJECT_POSITION__STRUCT_HPP_
#define ROBP_INTERFACES__MSG__DETAIL__OBJECT_POSITION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robp_interfaces__msg__ObjectPosition __attribute__((deprecated))
#else
# define DEPRECATED__robp_interfaces__msg__ObjectPosition __declspec(deprecated)
#endif

namespace robp_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObjectPosition_
{
  using Type = ObjectPosition_<ContainerAllocator>;

  explicit ObjectPosition_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->object_type = "";
    }
  }

  explicit ObjectPosition_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    object_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->object_type = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _object_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _object_type_type object_type;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__object_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->object_type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robp_interfaces::msg::ObjectPosition_<ContainerAllocator> *;
  using ConstRawPtr =
    const robp_interfaces::msg::ObjectPosition_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robp_interfaces::msg::ObjectPosition_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robp_interfaces::msg::ObjectPosition_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robp_interfaces::msg::ObjectPosition_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robp_interfaces::msg::ObjectPosition_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robp_interfaces::msg::ObjectPosition_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robp_interfaces::msg::ObjectPosition_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robp_interfaces::msg::ObjectPosition_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robp_interfaces::msg::ObjectPosition_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robp_interfaces__msg__ObjectPosition
    std::shared_ptr<robp_interfaces::msg::ObjectPosition_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robp_interfaces__msg__ObjectPosition
    std::shared_ptr<robp_interfaces::msg::ObjectPosition_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectPosition_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->object_type != other.object_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectPosition_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectPosition_

// alias to use template instance with default allocator
using ObjectPosition =
  robp_interfaces::msg::ObjectPosition_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robp_interfaces

#endif  // ROBP_INTERFACES__MSG__DETAIL__OBJECT_POSITION__STRUCT_HPP_
