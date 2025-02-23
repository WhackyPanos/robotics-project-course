// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robp_interfaces:srv/BoxPositionSrv.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robp_interfaces/srv/box_position_srv.hpp"


#ifndef ROBP_INTERFACES__SRV__DETAIL__BOX_POSITION_SRV__BUILDER_HPP_
#define ROBP_INTERFACES__SRV__DETAIL__BOX_POSITION_SRV__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robp_interfaces/srv/detail/box_position_srv__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robp_interfaces
{

namespace srv
{

namespace builder
{

class Init_BoxPositionSrv_Request_want_box_pose
{
public:
  Init_BoxPositionSrv_Request_want_box_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robp_interfaces::srv::BoxPositionSrv_Request want_box_pose(::robp_interfaces::srv::BoxPositionSrv_Request::_want_box_pose_type arg)
  {
    msg_.want_box_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robp_interfaces::srv::BoxPositionSrv_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robp_interfaces::srv::BoxPositionSrv_Request>()
{
  return robp_interfaces::srv::builder::Init_BoxPositionSrv_Request_want_box_pose();
}

}  // namespace robp_interfaces


namespace robp_interfaces
{

namespace srv
{

namespace builder
{

class Init_BoxPositionSrv_Response_orientation
{
public:
  explicit Init_BoxPositionSrv_Response_orientation(::robp_interfaces::srv::BoxPositionSrv_Response & msg)
  : msg_(msg)
  {}
  ::robp_interfaces::srv::BoxPositionSrv_Response orientation(::robp_interfaces::srv::BoxPositionSrv_Response::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robp_interfaces::srv::BoxPositionSrv_Response msg_;
};

class Init_BoxPositionSrv_Response_y
{
public:
  explicit Init_BoxPositionSrv_Response_y(::robp_interfaces::srv::BoxPositionSrv_Response & msg)
  : msg_(msg)
  {}
  Init_BoxPositionSrv_Response_orientation y(::robp_interfaces::srv::BoxPositionSrv_Response::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_BoxPositionSrv_Response_orientation(msg_);
  }

private:
  ::robp_interfaces::srv::BoxPositionSrv_Response msg_;
};

class Init_BoxPositionSrv_Response_x
{
public:
  Init_BoxPositionSrv_Response_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoxPositionSrv_Response_y x(::robp_interfaces::srv::BoxPositionSrv_Response::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_BoxPositionSrv_Response_y(msg_);
  }

private:
  ::robp_interfaces::srv::BoxPositionSrv_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robp_interfaces::srv::BoxPositionSrv_Response>()
{
  return robp_interfaces::srv::builder::Init_BoxPositionSrv_Response_x();
}

}  // namespace robp_interfaces


namespace robp_interfaces
{

namespace srv
{

namespace builder
{

class Init_BoxPositionSrv_Event_response
{
public:
  explicit Init_BoxPositionSrv_Event_response(::robp_interfaces::srv::BoxPositionSrv_Event & msg)
  : msg_(msg)
  {}
  ::robp_interfaces::srv::BoxPositionSrv_Event response(::robp_interfaces::srv::BoxPositionSrv_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robp_interfaces::srv::BoxPositionSrv_Event msg_;
};

class Init_BoxPositionSrv_Event_request
{
public:
  explicit Init_BoxPositionSrv_Event_request(::robp_interfaces::srv::BoxPositionSrv_Event & msg)
  : msg_(msg)
  {}
  Init_BoxPositionSrv_Event_response request(::robp_interfaces::srv::BoxPositionSrv_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_BoxPositionSrv_Event_response(msg_);
  }

private:
  ::robp_interfaces::srv::BoxPositionSrv_Event msg_;
};

class Init_BoxPositionSrv_Event_info
{
public:
  Init_BoxPositionSrv_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoxPositionSrv_Event_request info(::robp_interfaces::srv::BoxPositionSrv_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_BoxPositionSrv_Event_request(msg_);
  }

private:
  ::robp_interfaces::srv::BoxPositionSrv_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robp_interfaces::srv::BoxPositionSrv_Event>()
{
  return robp_interfaces::srv::builder::Init_BoxPositionSrv_Event_info();
}

}  // namespace robp_interfaces

#endif  // ROBP_INTERFACES__SRV__DETAIL__BOX_POSITION_SRV__BUILDER_HPP_
