// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robp_interfaces:srv/ObjectPositionSrv.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robp_interfaces/srv/object_position_srv.h"


#ifndef ROBP_INTERFACES__SRV__DETAIL__OBJECT_POSITION_SRV__STRUCT_H_
#define ROBP_INTERFACES__SRV__DETAIL__OBJECT_POSITION_SRV__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/ObjectPositionSrv in the package robp_interfaces.
typedef struct robp_interfaces__srv__ObjectPositionSrv_Request
{
  bool want_object_position;
} robp_interfaces__srv__ObjectPositionSrv_Request;

// Struct for a sequence of robp_interfaces__srv__ObjectPositionSrv_Request.
typedef struct robp_interfaces__srv__ObjectPositionSrv_Request__Sequence
{
  robp_interfaces__srv__ObjectPositionSrv_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robp_interfaces__srv__ObjectPositionSrv_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'object_type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ObjectPositionSrv in the package robp_interfaces.
typedef struct robp_interfaces__srv__ObjectPositionSrv_Response
{
  float x;
  float y;
  rosidl_runtime_c__String object_type;
} robp_interfaces__srv__ObjectPositionSrv_Response;

// Struct for a sequence of robp_interfaces__srv__ObjectPositionSrv_Response.
typedef struct robp_interfaces__srv__ObjectPositionSrv_Response__Sequence
{
  robp_interfaces__srv__ObjectPositionSrv_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robp_interfaces__srv__ObjectPositionSrv_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  robp_interfaces__srv__ObjectPositionSrv_Event__request__MAX_SIZE = 1
};
// response
enum
{
  robp_interfaces__srv__ObjectPositionSrv_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/ObjectPositionSrv in the package robp_interfaces.
typedef struct robp_interfaces__srv__ObjectPositionSrv_Event
{
  service_msgs__msg__ServiceEventInfo info;
  robp_interfaces__srv__ObjectPositionSrv_Request__Sequence request;
  robp_interfaces__srv__ObjectPositionSrv_Response__Sequence response;
} robp_interfaces__srv__ObjectPositionSrv_Event;

// Struct for a sequence of robp_interfaces__srv__ObjectPositionSrv_Event.
typedef struct robp_interfaces__srv__ObjectPositionSrv_Event__Sequence
{
  robp_interfaces__srv__ObjectPositionSrv_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robp_interfaces__srv__ObjectPositionSrv_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBP_INTERFACES__SRV__DETAIL__OBJECT_POSITION_SRV__STRUCT_H_
