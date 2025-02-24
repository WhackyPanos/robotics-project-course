// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robp_interfaces:msg/ObjectPosition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robp_interfaces/msg/object_position.hpp"


#ifndef ROBP_INTERFACES__MSG__DETAIL__OBJECT_POSITION__BUILDER_HPP_
#define ROBP_INTERFACES__MSG__DETAIL__OBJECT_POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robp_interfaces/msg/detail/object_position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robp_interfaces
{

namespace msg
{

namespace builder
{

class Init_ObjectPosition_object_type
{
public:
  explicit Init_ObjectPosition_object_type(::robp_interfaces::msg::ObjectPosition & msg)
  : msg_(msg)
  {}
  ::robp_interfaces::msg::ObjectPosition object_type(::robp_interfaces::msg::ObjectPosition::_object_type_type arg)
  {
    msg_.object_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robp_interfaces::msg::ObjectPosition msg_;
};

class Init_ObjectPosition_y
{
public:
  explicit Init_ObjectPosition_y(::robp_interfaces::msg::ObjectPosition & msg)
  : msg_(msg)
  {}
  Init_ObjectPosition_object_type y(::robp_interfaces::msg::ObjectPosition::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_ObjectPosition_object_type(msg_);
  }

private:
  ::robp_interfaces::msg::ObjectPosition msg_;
};

class Init_ObjectPosition_x
{
public:
  explicit Init_ObjectPosition_x(::robp_interfaces::msg::ObjectPosition & msg)
  : msg_(msg)
  {}
  Init_ObjectPosition_y x(::robp_interfaces::msg::ObjectPosition::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_ObjectPosition_y(msg_);
  }

private:
  ::robp_interfaces::msg::ObjectPosition msg_;
};

class Init_ObjectPosition_header
{
public:
  Init_ObjectPosition_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectPosition_x header(::robp_interfaces::msg::ObjectPosition::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ObjectPosition_x(msg_);
  }

private:
  ::robp_interfaces::msg::ObjectPosition msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robp_interfaces::msg::ObjectPosition>()
{
  return robp_interfaces::msg::builder::Init_ObjectPosition_header();
}

}  // namespace robp_interfaces

#endif  // ROBP_INTERFACES__MSG__DETAIL__OBJECT_POSITION__BUILDER_HPP_
