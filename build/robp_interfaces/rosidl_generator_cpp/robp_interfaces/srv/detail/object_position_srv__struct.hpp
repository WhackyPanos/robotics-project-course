// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robp_interfaces:srv/ObjectPositionSrv.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robp_interfaces/srv/object_position_srv.hpp"


#ifndef ROBP_INTERFACES__SRV__DETAIL__OBJECT_POSITION_SRV__STRUCT_HPP_
#define ROBP_INTERFACES__SRV__DETAIL__OBJECT_POSITION_SRV__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robp_interfaces__srv__ObjectPositionSrv_Request __attribute__((deprecated))
#else
# define DEPRECATED__robp_interfaces__srv__ObjectPositionSrv_Request __declspec(deprecated)
#endif

namespace robp_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ObjectPositionSrv_Request_
{
  using Type = ObjectPositionSrv_Request_<ContainerAllocator>;

  explicit ObjectPositionSrv_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->want_object_position = false;
    }
  }

  explicit ObjectPositionSrv_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->want_object_position = false;
    }
  }

  // field types and members
  using _want_object_position_type =
    bool;
  _want_object_position_type want_object_position;

  // setters for named parameter idiom
  Type & set__want_object_position(
    const bool & _arg)
  {
    this->want_object_position = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robp_interfaces::srv::ObjectPositionSrv_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robp_interfaces::srv::ObjectPositionSrv_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robp_interfaces::srv::ObjectPositionSrv_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robp_interfaces::srv::ObjectPositionSrv_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robp_interfaces::srv::ObjectPositionSrv_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robp_interfaces::srv::ObjectPositionSrv_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robp_interfaces::srv::ObjectPositionSrv_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robp_interfaces::srv::ObjectPositionSrv_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robp_interfaces::srv::ObjectPositionSrv_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robp_interfaces::srv::ObjectPositionSrv_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robp_interfaces__srv__ObjectPositionSrv_Request
    std::shared_ptr<robp_interfaces::srv::ObjectPositionSrv_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robp_interfaces__srv__ObjectPositionSrv_Request
    std::shared_ptr<robp_interfaces::srv::ObjectPositionSrv_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectPositionSrv_Request_ & other) const
  {
    if (this->want_object_position != other.want_object_position) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectPositionSrv_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectPositionSrv_Request_

// alias to use template instance with default allocator
using ObjectPositionSrv_Request =
  robp_interfaces::srv::ObjectPositionSrv_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robp_interfaces


#ifndef _WIN32
# define DEPRECATED__robp_interfaces__srv__ObjectPositionSrv_Response __attribute__((deprecated))
#else
# define DEPRECATED__robp_interfaces__srv__ObjectPositionSrv_Response __declspec(deprecated)
#endif

namespace robp_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ObjectPositionSrv_Response_
{
  using Type = ObjectPositionSrv_Response_<ContainerAllocator>;

  explicit ObjectPositionSrv_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->object_type = "";
    }
  }

  explicit ObjectPositionSrv_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : object_type(_alloc)
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
    robp_interfaces::srv::ObjectPositionSrv_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robp_interfaces::srv::ObjectPositionSrv_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robp_interfaces::srv::ObjectPositionSrv_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robp_interfaces::srv::ObjectPositionSrv_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robp_interfaces::srv::ObjectPositionSrv_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robp_interfaces::srv::ObjectPositionSrv_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robp_interfaces::srv::ObjectPositionSrv_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robp_interfaces::srv::ObjectPositionSrv_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robp_interfaces::srv::ObjectPositionSrv_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robp_interfaces::srv::ObjectPositionSrv_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robp_interfaces__srv__ObjectPositionSrv_Response
    std::shared_ptr<robp_interfaces::srv::ObjectPositionSrv_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robp_interfaces__srv__ObjectPositionSrv_Response
    std::shared_ptr<robp_interfaces::srv::ObjectPositionSrv_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectPositionSrv_Response_ & other) const
  {
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
  bool operator!=(const ObjectPositionSrv_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectPositionSrv_Response_

// alias to use template instance with default allocator
using ObjectPositionSrv_Response =
  robp_interfaces::srv::ObjectPositionSrv_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robp_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robp_interfaces__srv__ObjectPositionSrv_Event __attribute__((deprecated))
#else
# define DEPRECATED__robp_interfaces__srv__ObjectPositionSrv_Event __declspec(deprecated)
#endif

namespace robp_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ObjectPositionSrv_Event_
{
  using Type = ObjectPositionSrv_Event_<ContainerAllocator>;

  explicit ObjectPositionSrv_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit ObjectPositionSrv_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<robp_interfaces::srv::ObjectPositionSrv_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robp_interfaces::srv::ObjectPositionSrv_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<robp_interfaces::srv::ObjectPositionSrv_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robp_interfaces::srv::ObjectPositionSrv_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<robp_interfaces::srv::ObjectPositionSrv_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robp_interfaces::srv::ObjectPositionSrv_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<robp_interfaces::srv::ObjectPositionSrv_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robp_interfaces::srv::ObjectPositionSrv_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robp_interfaces::srv::ObjectPositionSrv_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const robp_interfaces::srv::ObjectPositionSrv_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robp_interfaces::srv::ObjectPositionSrv_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robp_interfaces::srv::ObjectPositionSrv_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robp_interfaces::srv::ObjectPositionSrv_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robp_interfaces::srv::ObjectPositionSrv_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robp_interfaces::srv::ObjectPositionSrv_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robp_interfaces::srv::ObjectPositionSrv_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robp_interfaces::srv::ObjectPositionSrv_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robp_interfaces::srv::ObjectPositionSrv_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robp_interfaces__srv__ObjectPositionSrv_Event
    std::shared_ptr<robp_interfaces::srv::ObjectPositionSrv_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robp_interfaces__srv__ObjectPositionSrv_Event
    std::shared_ptr<robp_interfaces::srv::ObjectPositionSrv_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectPositionSrv_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectPositionSrv_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectPositionSrv_Event_

// alias to use template instance with default allocator
using ObjectPositionSrv_Event =
  robp_interfaces::srv::ObjectPositionSrv_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robp_interfaces

namespace robp_interfaces
{

namespace srv
{

struct ObjectPositionSrv
{
  using Request = robp_interfaces::srv::ObjectPositionSrv_Request;
  using Response = robp_interfaces::srv::ObjectPositionSrv_Response;
  using Event = robp_interfaces::srv::ObjectPositionSrv_Event;
};

}  // namespace srv

}  // namespace robp_interfaces

#endif  // ROBP_INTERFACES__SRV__DETAIL__OBJECT_POSITION_SRV__STRUCT_HPP_
