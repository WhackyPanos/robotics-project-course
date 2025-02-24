// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robp_interfaces:msg/ObjectPosition.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robp_interfaces/msg/detail/object_position__rosidl_typesupport_introspection_c.h"
#include "robp_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robp_interfaces/msg/detail/object_position__functions.h"
#include "robp_interfaces/msg/detail/object_position__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `object_type`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robp_interfaces__msg__ObjectPosition__rosidl_typesupport_introspection_c__ObjectPosition_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robp_interfaces__msg__ObjectPosition__init(message_memory);
}

void robp_interfaces__msg__ObjectPosition__rosidl_typesupport_introspection_c__ObjectPosition_fini_function(void * message_memory)
{
  robp_interfaces__msg__ObjectPosition__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robp_interfaces__msg__ObjectPosition__rosidl_typesupport_introspection_c__ObjectPosition_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robp_interfaces__msg__ObjectPosition, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robp_interfaces__msg__ObjectPosition, x),  // bytes offset in struct
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
    offsetof(robp_interfaces__msg__ObjectPosition, y),  // bytes offset in struct
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
    offsetof(robp_interfaces__msg__ObjectPosition, object_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robp_interfaces__msg__ObjectPosition__rosidl_typesupport_introspection_c__ObjectPosition_message_members = {
  "robp_interfaces__msg",  // message namespace
  "ObjectPosition",  // message name
  4,  // number of fields
  sizeof(robp_interfaces__msg__ObjectPosition),
  false,  // has_any_key_member_
  robp_interfaces__msg__ObjectPosition__rosidl_typesupport_introspection_c__ObjectPosition_message_member_array,  // message members
  robp_interfaces__msg__ObjectPosition__rosidl_typesupport_introspection_c__ObjectPosition_init_function,  // function to initialize message memory (memory has to be allocated)
  robp_interfaces__msg__ObjectPosition__rosidl_typesupport_introspection_c__ObjectPosition_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robp_interfaces__msg__ObjectPosition__rosidl_typesupport_introspection_c__ObjectPosition_message_type_support_handle = {
  0,
  &robp_interfaces__msg__ObjectPosition__rosidl_typesupport_introspection_c__ObjectPosition_message_members,
  get_message_typesupport_handle_function,
  &robp_interfaces__msg__ObjectPosition__get_type_hash,
  &robp_interfaces__msg__ObjectPosition__get_type_description,
  &robp_interfaces__msg__ObjectPosition__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robp_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robp_interfaces, msg, ObjectPosition)() {
  robp_interfaces__msg__ObjectPosition__rosidl_typesupport_introspection_c__ObjectPosition_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!robp_interfaces__msg__ObjectPosition__rosidl_typesupport_introspection_c__ObjectPosition_message_type_support_handle.typesupport_identifier) {
    robp_interfaces__msg__ObjectPosition__rosidl_typesupport_introspection_c__ObjectPosition_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robp_interfaces__msg__ObjectPosition__rosidl_typesupport_introspection_c__ObjectPosition_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
