// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robp_interfaces:srv/ObjectPositionSrv.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robp_interfaces/srv/detail/object_position_srv__rosidl_typesupport_introspection_c.h"
#include "robp_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robp_interfaces/srv/detail/object_position_srv__functions.h"
#include "robp_interfaces/srv/detail/object_position_srv__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void robp_interfaces__srv__ObjectPositionSrv_Request__rosidl_typesupport_introspection_c__ObjectPositionSrv_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robp_interfaces__srv__ObjectPositionSrv_Request__init(message_memory);
}

void robp_interfaces__srv__ObjectPositionSrv_Request__rosidl_typesupport_introspection_c__ObjectPositionSrv_Request_fini_function(void * message_memory)
{
  robp_interfaces__srv__ObjectPositionSrv_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robp_interfaces__srv__ObjectPositionSrv_Request__rosidl_typesupport_introspection_c__ObjectPositionSrv_Request_message_member_array[1] = {
  {
    "want_object_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robp_interfaces__srv__ObjectPositionSrv_Request, want_object_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robp_interfaces__srv__ObjectPositionSrv_Request__rosidl_typesupport_introspection_c__ObjectPositionSrv_Request_message_members = {
  "robp_interfaces__srv",  // message namespace
  "ObjectPositionSrv_Request",  // message name
  1,  // number of fields
  sizeof(robp_interfaces__srv__ObjectPositionSrv_Request),
  false,  // has_any_key_member_
  robp_interfaces__srv__ObjectPositionSrv_Request__rosidl_typesupport_introspection_c__ObjectPositionSrv_Request_message_member_array,  // message members
  robp_interfaces__srv__ObjectPositionSrv_Request__rosidl_typesupport_introspection_c__ObjectPositionSrv_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  robp_interfaces__srv__ObjectPositionSrv_Request__rosidl_typesupport_introspection_c__ObjectPositionSrv_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robp_interfaces__srv__ObjectPositionSrv_Request__rosidl_typesupport_introspection_c__ObjectPositionSrv_Request_message_type_support_handle = {
  0,
  &robp_interfaces__srv__ObjectPositionSrv_Request__rosidl_typesupport_introspection_c__ObjectPositionSrv_Request_message_members,
  get_message_typesupport_handle_function,
  &robp_interfaces__srv__ObjectPositionSrv_Request__get_type_hash,
  &robp_interfaces__srv__ObjectPositionSrv_Request__get_type_description,
  &robp_interfaces__srv__ObjectPositionSrv_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robp_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robp_interfaces, srv, ObjectPositionSrv_Request)() {
  if (!robp_interfaces__srv__ObjectPositionSrv_Request__rosidl_typesupport_introspection_c__ObjectPositionSrv_Request_message_type_support_handle.typesupport_identifier) {
    robp_interfaces__srv__ObjectPositionSrv_Request__rosidl_typesupport_introspection_c__ObjectPositionSrv_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robp_interfaces__srv__ObjectPositionSrv_Request__rosidl_typesupport_introspection_c__ObjectPositionSrv_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robp_interfaces/srv/detail/object_position_srv__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robp_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robp_interfaces/srv/detail/object_position_srv__functions.h"
// already included above
// #include "robp_interfaces/srv/detail/object_position_srv__struct.h"


// Include directives for member types
// Member `object_type`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robp_interfaces__srv__ObjectPositionSrv_Response__rosidl_typesupport_introspection_c__ObjectPositionSrv_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robp_interfaces__srv__ObjectPositionSrv_Response__init(message_memory);
}

void robp_interfaces__srv__ObjectPositionSrv_Response__rosidl_typesupport_introspection_c__ObjectPositionSrv_Response_fini_function(void * message_memory)
{
  robp_interfaces__srv__ObjectPositionSrv_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robp_interfaces__srv__ObjectPositionSrv_Response__rosidl_typesupport_introspection_c__ObjectPositionSrv_Response_message_member_array[3] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robp_interfaces__srv__ObjectPositionSrv_Response, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robp_interfaces__srv__ObjectPositionSrv_Response, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "object_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robp_interfaces__srv__ObjectPositionSrv_Response, object_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robp_interfaces__srv__ObjectPositionSrv_Response__rosidl_typesupport_introspection_c__ObjectPositionSrv_Response_message_members = {
  "robp_interfaces__srv",  // message namespace
  "ObjectPositionSrv_Response",  // message name
  3,  // number of fields
  sizeof(robp_interfaces__srv__ObjectPositionSrv_Response),
  false,  // has_any_key_member_
  robp_interfaces__srv__ObjectPositionSrv_Response__rosidl_typesupport_introspection_c__ObjectPositionSrv_Response_message_member_array,  // message members
  robp_interfaces__srv__ObjectPositionSrv_Response__rosidl_typesupport_introspection_c__ObjectPositionSrv_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  robp_interfaces__srv__ObjectPositionSrv_Response__rosidl_typesupport_introspection_c__ObjectPositionSrv_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robp_interfaces__srv__ObjectPositionSrv_Response__rosidl_typesupport_introspection_c__ObjectPositionSrv_Response_message_type_support_handle = {
  0,
  &robp_interfaces__srv__ObjectPositionSrv_Response__rosidl_typesupport_introspection_c__ObjectPositionSrv_Response_message_members,
  get_message_typesupport_handle_function,
  &robp_interfaces__srv__ObjectPositionSrv_Response__get_type_hash,
  &robp_interfaces__srv__ObjectPositionSrv_Response__get_type_description,
  &robp_interfaces__srv__ObjectPositionSrv_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robp_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robp_interfaces, srv, ObjectPositionSrv_Response)() {
  if (!robp_interfaces__srv__ObjectPositionSrv_Response__rosidl_typesupport_introspection_c__ObjectPositionSrv_Response_message_type_support_handle.typesupport_identifier) {
    robp_interfaces__srv__ObjectPositionSrv_Response__rosidl_typesupport_introspection_c__ObjectPositionSrv_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robp_interfaces__srv__ObjectPositionSrv_Response__rosidl_typesupport_introspection_c__ObjectPositionSrv_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robp_interfaces/srv/detail/object_position_srv__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robp_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robp_interfaces/srv/detail/object_position_srv__functions.h"
// already included above
// #include "robp_interfaces/srv/detail/object_position_srv__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "robp_interfaces/srv/object_position_srv.h"
// Member `request`
// Member `response`
// already included above
// #include "robp_interfaces/srv/detail/object_position_srv__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__ObjectPositionSrv_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robp_interfaces__srv__ObjectPositionSrv_Event__init(message_memory);
}

void robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__ObjectPositionSrv_Event_fini_function(void * message_memory)
{
  robp_interfaces__srv__ObjectPositionSrv_Event__fini(message_memory);
}

size_t robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__size_function__ObjectPositionSrv_Event__request(
  const void * untyped_member)
{
  const robp_interfaces__srv__ObjectPositionSrv_Request__Sequence * member =
    (const robp_interfaces__srv__ObjectPositionSrv_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__get_const_function__ObjectPositionSrv_Event__request(
  const void * untyped_member, size_t index)
{
  const robp_interfaces__srv__ObjectPositionSrv_Request__Sequence * member =
    (const robp_interfaces__srv__ObjectPositionSrv_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__get_function__ObjectPositionSrv_Event__request(
  void * untyped_member, size_t index)
{
  robp_interfaces__srv__ObjectPositionSrv_Request__Sequence * member =
    (robp_interfaces__srv__ObjectPositionSrv_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__fetch_function__ObjectPositionSrv_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const robp_interfaces__srv__ObjectPositionSrv_Request * item =
    ((const robp_interfaces__srv__ObjectPositionSrv_Request *)
    robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__get_const_function__ObjectPositionSrv_Event__request(untyped_member, index));
  robp_interfaces__srv__ObjectPositionSrv_Request * value =
    (robp_interfaces__srv__ObjectPositionSrv_Request *)(untyped_value);
  *value = *item;
}

void robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__assign_function__ObjectPositionSrv_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  robp_interfaces__srv__ObjectPositionSrv_Request * item =
    ((robp_interfaces__srv__ObjectPositionSrv_Request *)
    robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__get_function__ObjectPositionSrv_Event__request(untyped_member, index));
  const robp_interfaces__srv__ObjectPositionSrv_Request * value =
    (const robp_interfaces__srv__ObjectPositionSrv_Request *)(untyped_value);
  *item = *value;
}

bool robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__resize_function__ObjectPositionSrv_Event__request(
  void * untyped_member, size_t size)
{
  robp_interfaces__srv__ObjectPositionSrv_Request__Sequence * member =
    (robp_interfaces__srv__ObjectPositionSrv_Request__Sequence *)(untyped_member);
  robp_interfaces__srv__ObjectPositionSrv_Request__Sequence__fini(member);
  return robp_interfaces__srv__ObjectPositionSrv_Request__Sequence__init(member, size);
}

size_t robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__size_function__ObjectPositionSrv_Event__response(
  const void * untyped_member)
{
  const robp_interfaces__srv__ObjectPositionSrv_Response__Sequence * member =
    (const robp_interfaces__srv__ObjectPositionSrv_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__get_const_function__ObjectPositionSrv_Event__response(
  const void * untyped_member, size_t index)
{
  const robp_interfaces__srv__ObjectPositionSrv_Response__Sequence * member =
    (const robp_interfaces__srv__ObjectPositionSrv_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__get_function__ObjectPositionSrv_Event__response(
  void * untyped_member, size_t index)
{
  robp_interfaces__srv__ObjectPositionSrv_Response__Sequence * member =
    (robp_interfaces__srv__ObjectPositionSrv_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__fetch_function__ObjectPositionSrv_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const robp_interfaces__srv__ObjectPositionSrv_Response * item =
    ((const robp_interfaces__srv__ObjectPositionSrv_Response *)
    robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__get_const_function__ObjectPositionSrv_Event__response(untyped_member, index));
  robp_interfaces__srv__ObjectPositionSrv_Response * value =
    (robp_interfaces__srv__ObjectPositionSrv_Response *)(untyped_value);
  *value = *item;
}

void robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__assign_function__ObjectPositionSrv_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  robp_interfaces__srv__ObjectPositionSrv_Response * item =
    ((robp_interfaces__srv__ObjectPositionSrv_Response *)
    robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__get_function__ObjectPositionSrv_Event__response(untyped_member, index));
  const robp_interfaces__srv__ObjectPositionSrv_Response * value =
    (const robp_interfaces__srv__ObjectPositionSrv_Response *)(untyped_value);
  *item = *value;
}

bool robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__resize_function__ObjectPositionSrv_Event__response(
  void * untyped_member, size_t size)
{
  robp_interfaces__srv__ObjectPositionSrv_Response__Sequence * member =
    (robp_interfaces__srv__ObjectPositionSrv_Response__Sequence *)(untyped_member);
  robp_interfaces__srv__ObjectPositionSrv_Response__Sequence__fini(member);
  return robp_interfaces__srv__ObjectPositionSrv_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__ObjectPositionSrv_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robp_interfaces__srv__ObjectPositionSrv_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(robp_interfaces__srv__ObjectPositionSrv_Event, request),  // bytes offset in struct
    NULL,  // default value
    robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__size_function__ObjectPositionSrv_Event__request,  // size() function pointer
    robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__get_const_function__ObjectPositionSrv_Event__request,  // get_const(index) function pointer
    robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__get_function__ObjectPositionSrv_Event__request,  // get(index) function pointer
    robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__fetch_function__ObjectPositionSrv_Event__request,  // fetch(index, &value) function pointer
    robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__assign_function__ObjectPositionSrv_Event__request,  // assign(index, value) function pointer
    robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__resize_function__ObjectPositionSrv_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(robp_interfaces__srv__ObjectPositionSrv_Event, response),  // bytes offset in struct
    NULL,  // default value
    robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__size_function__ObjectPositionSrv_Event__response,  // size() function pointer
    robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__get_const_function__ObjectPositionSrv_Event__response,  // get_const(index) function pointer
    robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__get_function__ObjectPositionSrv_Event__response,  // get(index) function pointer
    robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__fetch_function__ObjectPositionSrv_Event__response,  // fetch(index, &value) function pointer
    robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__assign_function__ObjectPositionSrv_Event__response,  // assign(index, value) function pointer
    robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__resize_function__ObjectPositionSrv_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__ObjectPositionSrv_Event_message_members = {
  "robp_interfaces__srv",  // message namespace
  "ObjectPositionSrv_Event",  // message name
  3,  // number of fields
  sizeof(robp_interfaces__srv__ObjectPositionSrv_Event),
  false,  // has_any_key_member_
  robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__ObjectPositionSrv_Event_message_member_array,  // message members
  robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__ObjectPositionSrv_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__ObjectPositionSrv_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__ObjectPositionSrv_Event_message_type_support_handle = {
  0,
  &robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__ObjectPositionSrv_Event_message_members,
  get_message_typesupport_handle_function,
  &robp_interfaces__srv__ObjectPositionSrv_Event__get_type_hash,
  &robp_interfaces__srv__ObjectPositionSrv_Event__get_type_description,
  &robp_interfaces__srv__ObjectPositionSrv_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robp_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robp_interfaces, srv, ObjectPositionSrv_Event)() {
  robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__ObjectPositionSrv_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__ObjectPositionSrv_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robp_interfaces, srv, ObjectPositionSrv_Request)();
  robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__ObjectPositionSrv_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robp_interfaces, srv, ObjectPositionSrv_Response)();
  if (!robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__ObjectPositionSrv_Event_message_type_support_handle.typesupport_identifier) {
    robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__ObjectPositionSrv_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__ObjectPositionSrv_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robp_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "robp_interfaces/srv/detail/object_position_srv__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers robp_interfaces__srv__detail__object_position_srv__rosidl_typesupport_introspection_c__ObjectPositionSrv_service_members = {
  "robp_interfaces__srv",  // service namespace
  "ObjectPositionSrv",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // robp_interfaces__srv__detail__object_position_srv__rosidl_typesupport_introspection_c__ObjectPositionSrv_Request_message_type_support_handle,
  NULL,  // response message
  // robp_interfaces__srv__detail__object_position_srv__rosidl_typesupport_introspection_c__ObjectPositionSrv_Response_message_type_support_handle
  NULL  // event_message
  // robp_interfaces__srv__detail__object_position_srv__rosidl_typesupport_introspection_c__ObjectPositionSrv_Response_message_type_support_handle
};


static rosidl_service_type_support_t robp_interfaces__srv__detail__object_position_srv__rosidl_typesupport_introspection_c__ObjectPositionSrv_service_type_support_handle = {
  0,
  &robp_interfaces__srv__detail__object_position_srv__rosidl_typesupport_introspection_c__ObjectPositionSrv_service_members,
  get_service_typesupport_handle_function,
  &robp_interfaces__srv__ObjectPositionSrv_Request__rosidl_typesupport_introspection_c__ObjectPositionSrv_Request_message_type_support_handle,
  &robp_interfaces__srv__ObjectPositionSrv_Response__rosidl_typesupport_introspection_c__ObjectPositionSrv_Response_message_type_support_handle,
  &robp_interfaces__srv__ObjectPositionSrv_Event__rosidl_typesupport_introspection_c__ObjectPositionSrv_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    robp_interfaces,
    srv,
    ObjectPositionSrv
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    robp_interfaces,
    srv,
    ObjectPositionSrv
  ),
  &robp_interfaces__srv__ObjectPositionSrv__get_type_hash,
  &robp_interfaces__srv__ObjectPositionSrv__get_type_description,
  &robp_interfaces__srv__ObjectPositionSrv__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robp_interfaces, srv, ObjectPositionSrv_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robp_interfaces, srv, ObjectPositionSrv_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robp_interfaces, srv, ObjectPositionSrv_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robp_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robp_interfaces, srv, ObjectPositionSrv)(void) {
  if (!robp_interfaces__srv__detail__object_position_srv__rosidl_typesupport_introspection_c__ObjectPositionSrv_service_type_support_handle.typesupport_identifier) {
    robp_interfaces__srv__detail__object_position_srv__rosidl_typesupport_introspection_c__ObjectPositionSrv_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)robp_interfaces__srv__detail__object_position_srv__rosidl_typesupport_introspection_c__ObjectPositionSrv_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robp_interfaces, srv, ObjectPositionSrv_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robp_interfaces, srv, ObjectPositionSrv_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robp_interfaces, srv, ObjectPositionSrv_Event)()->data;
  }

  return &robp_interfaces__srv__detail__object_position_srv__rosidl_typesupport_introspection_c__ObjectPositionSrv_service_type_support_handle;
}
