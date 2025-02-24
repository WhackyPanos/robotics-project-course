// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robp_interfaces:srv/ObjectPositionSrv.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robp_interfaces/srv/object_position_srv.hpp"


#ifndef ROBP_INTERFACES__SRV__DETAIL__OBJECT_POSITION_SRV__TRAITS_HPP_
#define ROBP_INTERFACES__SRV__DETAIL__OBJECT_POSITION_SRV__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robp_interfaces/srv/detail/object_position_srv__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robp_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ObjectPositionSrv_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: want_object_position
  {
    out << "want_object_position: ";
    rosidl_generator_traits::value_to_yaml(msg.want_object_position, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ObjectPositionSrv_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: want_object_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "want_object_position: ";
    rosidl_generator_traits::value_to_yaml(msg.want_object_position, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObjectPositionSrv_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robp_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robp_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robp_interfaces::srv::ObjectPositionSrv_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robp_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robp_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const robp_interfaces::srv::ObjectPositionSrv_Request & msg)
{
  return robp_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robp_interfaces::srv::ObjectPositionSrv_Request>()
{
  return "robp_interfaces::srv::ObjectPositionSrv_Request";
}

template<>
inline const char * name<robp_interfaces::srv::ObjectPositionSrv_Request>()
{
  return "robp_interfaces/srv/ObjectPositionSrv_Request";
}

template<>
struct has_fixed_size<robp_interfaces::srv::ObjectPositionSrv_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robp_interfaces::srv::ObjectPositionSrv_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robp_interfaces::srv::ObjectPositionSrv_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace robp_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ObjectPositionSrv_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: object_type
  {
    out << "object_type: ";
    rosidl_generator_traits::value_to_yaml(msg.object_type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ObjectPositionSrv_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: object_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "object_type: ";
    rosidl_generator_traits::value_to_yaml(msg.object_type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObjectPositionSrv_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robp_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robp_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robp_interfaces::srv::ObjectPositionSrv_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robp_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robp_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const robp_interfaces::srv::ObjectPositionSrv_Response & msg)
{
  return robp_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robp_interfaces::srv::ObjectPositionSrv_Response>()
{
  return "robp_interfaces::srv::ObjectPositionSrv_Response";
}

template<>
inline const char * name<robp_interfaces::srv::ObjectPositionSrv_Response>()
{
  return "robp_interfaces/srv/ObjectPositionSrv_Response";
}

template<>
struct has_fixed_size<robp_interfaces::srv::ObjectPositionSrv_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robp_interfaces::srv::ObjectPositionSrv_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robp_interfaces::srv::ObjectPositionSrv_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace robp_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ObjectPositionSrv_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ObjectPositionSrv_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObjectPositionSrv_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robp_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robp_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robp_interfaces::srv::ObjectPositionSrv_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  robp_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robp_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const robp_interfaces::srv::ObjectPositionSrv_Event & msg)
{
  return robp_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robp_interfaces::srv::ObjectPositionSrv_Event>()
{
  return "robp_interfaces::srv::ObjectPositionSrv_Event";
}

template<>
inline const char * name<robp_interfaces::srv::ObjectPositionSrv_Event>()
{
  return "robp_interfaces/srv/ObjectPositionSrv_Event";
}

template<>
struct has_fixed_size<robp_interfaces::srv::ObjectPositionSrv_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robp_interfaces::srv::ObjectPositionSrv_Event>
  : std::integral_constant<bool, has_bounded_size<robp_interfaces::srv::ObjectPositionSrv_Request>::value && has_bounded_size<robp_interfaces::srv::ObjectPositionSrv_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<robp_interfaces::srv::ObjectPositionSrv_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robp_interfaces::srv::ObjectPositionSrv>()
{
  return "robp_interfaces::srv::ObjectPositionSrv";
}

template<>
inline const char * name<robp_interfaces::srv::ObjectPositionSrv>()
{
  return "robp_interfaces/srv/ObjectPositionSrv";
}

template<>
struct has_fixed_size<robp_interfaces::srv::ObjectPositionSrv>
  : std::integral_constant<
    bool,
    has_fixed_size<robp_interfaces::srv::ObjectPositionSrv_Request>::value &&
    has_fixed_size<robp_interfaces::srv::ObjectPositionSrv_Response>::value
  >
{
};

template<>
struct has_bounded_size<robp_interfaces::srv::ObjectPositionSrv>
  : std::integral_constant<
    bool,
    has_bounded_size<robp_interfaces::srv::ObjectPositionSrv_Request>::value &&
    has_bounded_size<robp_interfaces::srv::ObjectPositionSrv_Response>::value
  >
{
};

template<>
struct is_service<robp_interfaces::srv::ObjectPositionSrv>
  : std::true_type
{
};

template<>
struct is_service_request<robp_interfaces::srv::ObjectPositionSrv_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robp_interfaces::srv::ObjectPositionSrv_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBP_INTERFACES__SRV__DETAIL__OBJECT_POSITION_SRV__TRAITS_HPP_
