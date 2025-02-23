// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robp_interfaces:srv/BoxPositionSrv.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robp_interfaces/srv/box_position_srv.h"


#ifndef ROBP_INTERFACES__SRV__DETAIL__BOX_POSITION_SRV__STRUCT_H_
#define ROBP_INTERFACES__SRV__DETAIL__BOX_POSITION_SRV__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/BoxPositionSrv in the package robp_interfaces.
typedef struct robp_interfaces__srv__BoxPositionSrv_Request
{
  bool want_box_pose;
} robp_interfaces__srv__BoxPositionSrv_Request;

// Struct for a sequence of robp_interfaces__srv__BoxPositionSrv_Request.
typedef struct robp_interfaces__srv__BoxPositionSrv_Request__Sequence
{
  robp_interfaces__srv__BoxPositionSrv_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robp_interfaces__srv__BoxPositionSrv_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/BoxPositionSrv in the package robp_interfaces.
typedef struct robp_interfaces__srv__BoxPositionSrv_Response
{
  float x;
  float y;
  float orientation;
} robp_interfaces__srv__BoxPositionSrv_Response;

// Struct for a sequence of robp_interfaces__srv__BoxPositionSrv_Response.
typedef struct robp_interfaces__srv__BoxPositionSrv_Response__Sequence
{
  robp_interfaces__srv__BoxPositionSrv_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robp_interfaces__srv__BoxPositionSrv_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  robp_interfaces__srv__BoxPositionSrv_Event__request__MAX_SIZE = 1
};
// response
enum
{
  robp_interfaces__srv__BoxPositionSrv_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/BoxPositionSrv in the package robp_interfaces.
typedef struct robp_interfaces__srv__BoxPositionSrv_Event
{
  service_msgs__msg__ServiceEventInfo info;
  robp_interfaces__srv__BoxPositionSrv_Request__Sequence request;
  robp_interfaces__srv__BoxPositionSrv_Response__Sequence response;
} robp_interfaces__srv__BoxPositionSrv_Event;

// Struct for a sequence of robp_interfaces__srv__BoxPositionSrv_Event.
typedef struct robp_interfaces__srv__BoxPositionSrv_Event__Sequence
{
  robp_interfaces__srv__BoxPositionSrv_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robp_interfaces__srv__BoxPositionSrv_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBP_INTERFACES__SRV__DETAIL__BOX_POSITION_SRV__STRUCT_H_
