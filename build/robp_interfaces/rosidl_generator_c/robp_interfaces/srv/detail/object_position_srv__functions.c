// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robp_interfaces:srv/ObjectPositionSrv.idl
// generated code does not contain a copyright notice
#include "robp_interfaces/srv/detail/object_position_srv__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
robp_interfaces__srv__ObjectPositionSrv_Request__init(robp_interfaces__srv__ObjectPositionSrv_Request * msg)
{
  if (!msg) {
    return false;
  }
  // want_object_position
  return true;
}

void
robp_interfaces__srv__ObjectPositionSrv_Request__fini(robp_interfaces__srv__ObjectPositionSrv_Request * msg)
{
  if (!msg) {
    return;
  }
  // want_object_position
}

bool
robp_interfaces__srv__ObjectPositionSrv_Request__are_equal(const robp_interfaces__srv__ObjectPositionSrv_Request * lhs, const robp_interfaces__srv__ObjectPositionSrv_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // want_object_position
  if (lhs->want_object_position != rhs->want_object_position) {
    return false;
  }
  return true;
}

bool
robp_interfaces__srv__ObjectPositionSrv_Request__copy(
  const robp_interfaces__srv__ObjectPositionSrv_Request * input,
  robp_interfaces__srv__ObjectPositionSrv_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // want_object_position
  output->want_object_position = input->want_object_position;
  return true;
}

robp_interfaces__srv__ObjectPositionSrv_Request *
robp_interfaces__srv__ObjectPositionSrv_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robp_interfaces__srv__ObjectPositionSrv_Request * msg = (robp_interfaces__srv__ObjectPositionSrv_Request *)allocator.allocate(sizeof(robp_interfaces__srv__ObjectPositionSrv_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robp_interfaces__srv__ObjectPositionSrv_Request));
  bool success = robp_interfaces__srv__ObjectPositionSrv_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robp_interfaces__srv__ObjectPositionSrv_Request__destroy(robp_interfaces__srv__ObjectPositionSrv_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robp_interfaces__srv__ObjectPositionSrv_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robp_interfaces__srv__ObjectPositionSrv_Request__Sequence__init(robp_interfaces__srv__ObjectPositionSrv_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robp_interfaces__srv__ObjectPositionSrv_Request * data = NULL;

  if (size) {
    data = (robp_interfaces__srv__ObjectPositionSrv_Request *)allocator.zero_allocate(size, sizeof(robp_interfaces__srv__ObjectPositionSrv_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robp_interfaces__srv__ObjectPositionSrv_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robp_interfaces__srv__ObjectPositionSrv_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robp_interfaces__srv__ObjectPositionSrv_Request__Sequence__fini(robp_interfaces__srv__ObjectPositionSrv_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robp_interfaces__srv__ObjectPositionSrv_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robp_interfaces__srv__ObjectPositionSrv_Request__Sequence *
robp_interfaces__srv__ObjectPositionSrv_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robp_interfaces__srv__ObjectPositionSrv_Request__Sequence * array = (robp_interfaces__srv__ObjectPositionSrv_Request__Sequence *)allocator.allocate(sizeof(robp_interfaces__srv__ObjectPositionSrv_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robp_interfaces__srv__ObjectPositionSrv_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robp_interfaces__srv__ObjectPositionSrv_Request__Sequence__destroy(robp_interfaces__srv__ObjectPositionSrv_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robp_interfaces__srv__ObjectPositionSrv_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robp_interfaces__srv__ObjectPositionSrv_Request__Sequence__are_equal(const robp_interfaces__srv__ObjectPositionSrv_Request__Sequence * lhs, const robp_interfaces__srv__ObjectPositionSrv_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robp_interfaces__srv__ObjectPositionSrv_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robp_interfaces__srv__ObjectPositionSrv_Request__Sequence__copy(
  const robp_interfaces__srv__ObjectPositionSrv_Request__Sequence * input,
  robp_interfaces__srv__ObjectPositionSrv_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robp_interfaces__srv__ObjectPositionSrv_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robp_interfaces__srv__ObjectPositionSrv_Request * data =
      (robp_interfaces__srv__ObjectPositionSrv_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robp_interfaces__srv__ObjectPositionSrv_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robp_interfaces__srv__ObjectPositionSrv_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robp_interfaces__srv__ObjectPositionSrv_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `object_type`
#include "rosidl_runtime_c/string_functions.h"

bool
robp_interfaces__srv__ObjectPositionSrv_Response__init(robp_interfaces__srv__ObjectPositionSrv_Response * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // object_type
  if (!rosidl_runtime_c__String__init(&msg->object_type)) {
    robp_interfaces__srv__ObjectPositionSrv_Response__fini(msg);
    return false;
  }
  return true;
}

void
robp_interfaces__srv__ObjectPositionSrv_Response__fini(robp_interfaces__srv__ObjectPositionSrv_Response * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // object_type
  rosidl_runtime_c__String__fini(&msg->object_type);
}

bool
robp_interfaces__srv__ObjectPositionSrv_Response__are_equal(const robp_interfaces__srv__ObjectPositionSrv_Response * lhs, const robp_interfaces__srv__ObjectPositionSrv_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // object_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->object_type), &(rhs->object_type)))
  {
    return false;
  }
  return true;
}

bool
robp_interfaces__srv__ObjectPositionSrv_Response__copy(
  const robp_interfaces__srv__ObjectPositionSrv_Response * input,
  robp_interfaces__srv__ObjectPositionSrv_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // object_type
  if (!rosidl_runtime_c__String__copy(
      &(input->object_type), &(output->object_type)))
  {
    return false;
  }
  return true;
}

robp_interfaces__srv__ObjectPositionSrv_Response *
robp_interfaces__srv__ObjectPositionSrv_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robp_interfaces__srv__ObjectPositionSrv_Response * msg = (robp_interfaces__srv__ObjectPositionSrv_Response *)allocator.allocate(sizeof(robp_interfaces__srv__ObjectPositionSrv_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robp_interfaces__srv__ObjectPositionSrv_Response));
  bool success = robp_interfaces__srv__ObjectPositionSrv_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robp_interfaces__srv__ObjectPositionSrv_Response__destroy(robp_interfaces__srv__ObjectPositionSrv_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robp_interfaces__srv__ObjectPositionSrv_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robp_interfaces__srv__ObjectPositionSrv_Response__Sequence__init(robp_interfaces__srv__ObjectPositionSrv_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robp_interfaces__srv__ObjectPositionSrv_Response * data = NULL;

  if (size) {
    data = (robp_interfaces__srv__ObjectPositionSrv_Response *)allocator.zero_allocate(size, sizeof(robp_interfaces__srv__ObjectPositionSrv_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robp_interfaces__srv__ObjectPositionSrv_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robp_interfaces__srv__ObjectPositionSrv_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robp_interfaces__srv__ObjectPositionSrv_Response__Sequence__fini(robp_interfaces__srv__ObjectPositionSrv_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robp_interfaces__srv__ObjectPositionSrv_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robp_interfaces__srv__ObjectPositionSrv_Response__Sequence *
robp_interfaces__srv__ObjectPositionSrv_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robp_interfaces__srv__ObjectPositionSrv_Response__Sequence * array = (robp_interfaces__srv__ObjectPositionSrv_Response__Sequence *)allocator.allocate(sizeof(robp_interfaces__srv__ObjectPositionSrv_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robp_interfaces__srv__ObjectPositionSrv_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robp_interfaces__srv__ObjectPositionSrv_Response__Sequence__destroy(robp_interfaces__srv__ObjectPositionSrv_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robp_interfaces__srv__ObjectPositionSrv_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robp_interfaces__srv__ObjectPositionSrv_Response__Sequence__are_equal(const robp_interfaces__srv__ObjectPositionSrv_Response__Sequence * lhs, const robp_interfaces__srv__ObjectPositionSrv_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robp_interfaces__srv__ObjectPositionSrv_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robp_interfaces__srv__ObjectPositionSrv_Response__Sequence__copy(
  const robp_interfaces__srv__ObjectPositionSrv_Response__Sequence * input,
  robp_interfaces__srv__ObjectPositionSrv_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robp_interfaces__srv__ObjectPositionSrv_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robp_interfaces__srv__ObjectPositionSrv_Response * data =
      (robp_interfaces__srv__ObjectPositionSrv_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robp_interfaces__srv__ObjectPositionSrv_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robp_interfaces__srv__ObjectPositionSrv_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robp_interfaces__srv__ObjectPositionSrv_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "robp_interfaces/srv/detail/object_position_srv__functions.h"

bool
robp_interfaces__srv__ObjectPositionSrv_Event__init(robp_interfaces__srv__ObjectPositionSrv_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    robp_interfaces__srv__ObjectPositionSrv_Event__fini(msg);
    return false;
  }
  // request
  if (!robp_interfaces__srv__ObjectPositionSrv_Request__Sequence__init(&msg->request, 0)) {
    robp_interfaces__srv__ObjectPositionSrv_Event__fini(msg);
    return false;
  }
  // response
  if (!robp_interfaces__srv__ObjectPositionSrv_Response__Sequence__init(&msg->response, 0)) {
    robp_interfaces__srv__ObjectPositionSrv_Event__fini(msg);
    return false;
  }
  return true;
}

void
robp_interfaces__srv__ObjectPositionSrv_Event__fini(robp_interfaces__srv__ObjectPositionSrv_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  robp_interfaces__srv__ObjectPositionSrv_Request__Sequence__fini(&msg->request);
  // response
  robp_interfaces__srv__ObjectPositionSrv_Response__Sequence__fini(&msg->response);
}

bool
robp_interfaces__srv__ObjectPositionSrv_Event__are_equal(const robp_interfaces__srv__ObjectPositionSrv_Event * lhs, const robp_interfaces__srv__ObjectPositionSrv_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!robp_interfaces__srv__ObjectPositionSrv_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!robp_interfaces__srv__ObjectPositionSrv_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
robp_interfaces__srv__ObjectPositionSrv_Event__copy(
  const robp_interfaces__srv__ObjectPositionSrv_Event * input,
  robp_interfaces__srv__ObjectPositionSrv_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!robp_interfaces__srv__ObjectPositionSrv_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!robp_interfaces__srv__ObjectPositionSrv_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

robp_interfaces__srv__ObjectPositionSrv_Event *
robp_interfaces__srv__ObjectPositionSrv_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robp_interfaces__srv__ObjectPositionSrv_Event * msg = (robp_interfaces__srv__ObjectPositionSrv_Event *)allocator.allocate(sizeof(robp_interfaces__srv__ObjectPositionSrv_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robp_interfaces__srv__ObjectPositionSrv_Event));
  bool success = robp_interfaces__srv__ObjectPositionSrv_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robp_interfaces__srv__ObjectPositionSrv_Event__destroy(robp_interfaces__srv__ObjectPositionSrv_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robp_interfaces__srv__ObjectPositionSrv_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robp_interfaces__srv__ObjectPositionSrv_Event__Sequence__init(robp_interfaces__srv__ObjectPositionSrv_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robp_interfaces__srv__ObjectPositionSrv_Event * data = NULL;

  if (size) {
    data = (robp_interfaces__srv__ObjectPositionSrv_Event *)allocator.zero_allocate(size, sizeof(robp_interfaces__srv__ObjectPositionSrv_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robp_interfaces__srv__ObjectPositionSrv_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robp_interfaces__srv__ObjectPositionSrv_Event__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robp_interfaces__srv__ObjectPositionSrv_Event__Sequence__fini(robp_interfaces__srv__ObjectPositionSrv_Event__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robp_interfaces__srv__ObjectPositionSrv_Event__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robp_interfaces__srv__ObjectPositionSrv_Event__Sequence *
robp_interfaces__srv__ObjectPositionSrv_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robp_interfaces__srv__ObjectPositionSrv_Event__Sequence * array = (robp_interfaces__srv__ObjectPositionSrv_Event__Sequence *)allocator.allocate(sizeof(robp_interfaces__srv__ObjectPositionSrv_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robp_interfaces__srv__ObjectPositionSrv_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robp_interfaces__srv__ObjectPositionSrv_Event__Sequence__destroy(robp_interfaces__srv__ObjectPositionSrv_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robp_interfaces__srv__ObjectPositionSrv_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robp_interfaces__srv__ObjectPositionSrv_Event__Sequence__are_equal(const robp_interfaces__srv__ObjectPositionSrv_Event__Sequence * lhs, const robp_interfaces__srv__ObjectPositionSrv_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robp_interfaces__srv__ObjectPositionSrv_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robp_interfaces__srv__ObjectPositionSrv_Event__Sequence__copy(
  const robp_interfaces__srv__ObjectPositionSrv_Event__Sequence * input,
  robp_interfaces__srv__ObjectPositionSrv_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robp_interfaces__srv__ObjectPositionSrv_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robp_interfaces__srv__ObjectPositionSrv_Event * data =
      (robp_interfaces__srv__ObjectPositionSrv_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robp_interfaces__srv__ObjectPositionSrv_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robp_interfaces__srv__ObjectPositionSrv_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robp_interfaces__srv__ObjectPositionSrv_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
