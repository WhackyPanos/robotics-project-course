// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robp_interfaces:msg/BoxPosition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robp_interfaces/msg/box_position.hpp"


#ifndef ROBP_INTERFACES__MSG__DETAIL__BOX_POSITION__BUILDER_HPP_
#define ROBP_INTERFACES__MSG__DETAIL__BOX_POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robp_interfaces/msg/detail/box_position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robp_interfaces
{

namespace msg
{

namespace builder
{

class Init_BoxPosition_orientation
{
public:
  explicit Init_BoxPosition_orientation(::robp_interfaces::msg::BoxPosition & msg)
  : msg_(msg)
  {}
  ::robp_interfaces::msg::BoxPosition orientation(::robp_interfaces::msg::BoxPosition::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robp_interfaces::msg::BoxPosition msg_;
};

class Init_BoxPosition_y
{
public:
  explicit Init_BoxPosition_y(::robp_interfaces::msg::BoxPosition & msg)
  : msg_(msg)
  {}
  Init_BoxPosition_orientation y(::robp_interfaces::msg::BoxPosition::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_BoxPosition_orientation(msg_);
  }

private:
  ::robp_interfaces::msg::BoxPosition msg_;
};

class Init_BoxPosition_x
{
public:
  explicit Init_BoxPosition_x(::robp_interfaces::msg::BoxPosition & msg)
  : msg_(msg)
  {}
  Init_BoxPosition_y x(::robp_interfaces::msg::BoxPosition::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_BoxPosition_y(msg_);
  }

private:
  ::robp_interfaces::msg::BoxPosition msg_;
};

class Init_BoxPosition_header
{
public:
  Init_BoxPosition_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoxPosition_x header(::robp_interfaces::msg::BoxPosition::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BoxPosition_x(msg_);
  }

private:
  ::robp_interfaces::msg::BoxPosition msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robp_interfaces::msg::BoxPosition>()
{
  return robp_interfaces::msg::builder::Init_BoxPosition_header();
}

}  // namespace robp_interfaces

#endif  // ROBP_INTERFACES__MSG__DETAIL__BOX_POSITION__BUILDER_HPP_
