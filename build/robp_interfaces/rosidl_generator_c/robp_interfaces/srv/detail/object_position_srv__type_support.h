// generated from rosidl_generator_c/resource/idl__type_support.h.em
// with input from robp_interfaces:srv/ObjectPositionSrv.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robp_interfaces/srv/object_position_srv.h"


#ifndef ROBP_INTERFACES__SRV__DETAIL__OBJECT_POSITION_SRV__TYPE_SUPPORT_H_
#define ROBP_INTERFACES__SRV__DETAIL__OBJECT_POSITION_SRV__TYPE_SUPPORT_H_

#include "rosidl_typesupport_interface/macros.h"

#include "robp_interfaces/msg/rosidl_generator_c__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_robp_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  robp_interfaces,
  srv,
  ObjectPositionSrv_Request
)(void);

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_robp_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  robp_interfaces,
  srv,
  ObjectPositionSrv_Response
)(void);

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_robp_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  robp_interfaces,
  srv,
  ObjectPositionSrv_Event
)(void);

#include "rosidl_runtime_c/service_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_robp_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
  rosidl_typesupport_c,
  robp_interfaces,
  srv,
  ObjectPositionSrv
)(void);

// Forward declare the function to create a service event message for this type.
ROSIDL_GENERATOR_C_PUBLIC_robp_interfaces
void *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  robp_interfaces,
  srv,
  ObjectPositionSrv
)(
  const rosidl_service_introspection_info_t * info,
  rcutils_allocator_t * allocator,
  const void * request_message,
  const void * response_message);

// Forward declare the function to destroy a service event message for this type.
ROSIDL_GENERATOR_C_PUBLIC_robp_interfaces
bool
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  robp_interfaces,
  srv,
  ObjectPositionSrv
)(
  void * event_msg,
  rcutils_allocator_t * allocator);

#ifdef __cplusplus
}
#endif

#endif  // ROBP_INTERFACES__SRV__DETAIL__OBJECT_POSITION_SRV__TYPE_SUPPORT_H_
