// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robp_interfaces:msg/BoxPosition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robp_interfaces/msg/box_position.h"


#ifndef ROBP_INTERFACES__MSG__DETAIL__BOX_POSITION__STRUCT_H_
#define ROBP_INTERFACES__MSG__DETAIL__BOX_POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/BoxPosition in the package robp_interfaces.
typedef struct robp_interfaces__msg__BoxPosition
{
  std_msgs__msg__Header header;
  float x;
  float y;
  float orientation;
} robp_interfaces__msg__BoxPosition;

// Struct for a sequence of robp_interfaces__msg__BoxPosition.
typedef struct robp_interfaces__msg__BoxPosition__Sequence
{
  robp_interfaces__msg__BoxPosition * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robp_interfaces__msg__BoxPosition__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBP_INTERFACES__MSG__DETAIL__BOX_POSITION__STRUCT_H_
