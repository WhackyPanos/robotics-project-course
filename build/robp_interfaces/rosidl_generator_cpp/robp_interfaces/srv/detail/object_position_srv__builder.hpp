// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robp_interfaces:srv/ObjectPositionSrv.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robp_interfaces/srv/object_position_srv.hpp"


#ifndef ROBP_INTERFACES__SRV__DETAIL__OBJECT_POSITION_SRV__BUILDER_HPP_
#define ROBP_INTERFACES__SRV__DETAIL__OBJECT_POSITION_SRV__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robp_interfaces/srv/detail/object_position_srv__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robp_interfaces
{

namespace srv
{

namespace builder
{

class Init_ObjectPositionSrv_Request_want_object_position
{
public:
  Init_ObjectPositionSrv_Request_want_object_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robp_interfaces::srv::ObjectPositionSrv_Request want_object_position(::robp_interfaces::srv::ObjectPositionSrv_Request::_want_object_position_type arg)
  {
    msg_.want_object_position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robp_interfaces::srv::ObjectPositionSrv_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robp_interfaces::srv::ObjectPositionSrv_Request>()
{
  return robp_interfaces::srv::builder::Init_ObjectPositionSrv_Request_want_object_position();
}

}  // namespace robp_interfaces


namespace robp_interfaces
{

namespace srv
{

namespace builder
{

class Init_ObjectPositionSrv_Response_object_type
{
public:
  explicit Init_ObjectPositionSrv_Response_object_type(::robp_interfaces::srv::ObjectPositionSrv_Response & msg)
  : msg_(msg)
  {}
  ::robp_interfaces::srv::ObjectPositionSrv_Response object_type(::robp_interfaces::srv::ObjectPositionSrv_Response::_object_type_type arg)
  {
    msg_.object_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robp_interfaces::srv::ObjectPositionSrv_Response msg_;
};

class Init_ObjectPositionSrv_Response_y
{
public:
  explicit Init_ObjectPositionSrv_Response_y(::robp_interfaces::srv::ObjectPositionSrv_Response & msg)
  : msg_(msg)
  {}
  Init_ObjectPositionSrv_Response_object_type y(::robp_interfaces::srv::ObjectPositionSrv_Response::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_ObjectPositionSrv_Response_object_type(msg_);
  }

private:
  ::robp_interfaces::srv::ObjectPositionSrv_Response msg_;
};

class Init_ObjectPositionSrv_Response_x
{
public:
  Init_ObjectPositionSrv_Response_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectPositionSrv_Response_y x(::robp_interfaces::srv::ObjectPositionSrv_Response::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_ObjectPositionSrv_Response_y(msg_);
  }

private:
  ::robp_interfaces::srv::ObjectPositionSrv_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robp_interfaces::srv::ObjectPositionSrv_Response>()
{
  return robp_interfaces::srv::builder::Init_ObjectPositionSrv_Response_x();
}

}  // namespace robp_interfaces


namespace robp_interfaces
{

namespace srv
{

namespace builder
{

class Init_ObjectPositionSrv_Event_response
{
public:
  explicit Init_ObjectPositionSrv_Event_response(::robp_interfaces::srv::ObjectPositionSrv_Event & msg)
  : msg_(msg)
  {}
  ::robp_interfaces::srv::ObjectPositionSrv_Event response(::robp_interfaces::srv::ObjectPositionSrv_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robp_interfaces::srv::ObjectPositionSrv_Event msg_;
};

class Init_ObjectPositionSrv_Event_request
{
public:
  explicit Init_ObjectPositionSrv_Event_request(::robp_interfaces::srv::ObjectPositionSrv_Event & msg)
  : msg_(msg)
  {}
  Init_ObjectPositionSrv_Event_response request(::robp_interfaces::srv::ObjectPositionSrv_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ObjectPositionSrv_Event_response(msg_);
  }

private:
  ::robp_interfaces::srv::ObjectPositionSrv_Event msg_;
};

class Init_ObjectPositionSrv_Event_info
{
public:
  Init_ObjectPositionSrv_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectPositionSrv_Event_request info(::robp_interfaces::srv::ObjectPositionSrv_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ObjectPositionSrv_Event_request(msg_);
  }

private:
  ::robp_interfaces::srv::ObjectPositionSrv_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robp_interfaces::srv::ObjectPositionSrv_Event>()
{
  return robp_interfaces::srv::builder::Init_ObjectPositionSrv_Event_info();
}

}  // namespace robp_interfaces

#endif  // ROBP_INTERFACES__SRV__DETAIL__OBJECT_POSITION_SRV__BUILDER_HPP_
