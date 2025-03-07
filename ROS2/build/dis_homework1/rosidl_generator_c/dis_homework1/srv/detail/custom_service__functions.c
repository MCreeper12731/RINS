// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dis_homework1:srv/CustomService.idl
// generated code does not contain a copyright notice
#include "dis_homework1/srv/detail/custom_service__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `field1`
#include "rosidl_runtime_c/string_functions.h"

bool
dis_homework1__srv__CustomService_Request__init(dis_homework1__srv__CustomService_Request * msg)
{
  if (!msg) {
    return false;
  }
  // field1
  if (!rosidl_runtime_c__String__init(&msg->field1)) {
    dis_homework1__srv__CustomService_Request__fini(msg);
    return false;
  }
  // field2
  return true;
}

void
dis_homework1__srv__CustomService_Request__fini(dis_homework1__srv__CustomService_Request * msg)
{
  if (!msg) {
    return;
  }
  // field1
  rosidl_runtime_c__String__fini(&msg->field1);
  // field2
}

bool
dis_homework1__srv__CustomService_Request__are_equal(const dis_homework1__srv__CustomService_Request * lhs, const dis_homework1__srv__CustomService_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // field1
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->field1), &(rhs->field1)))
  {
    return false;
  }
  // field2
  for (size_t i = 0; i < 10; ++i) {
    if (lhs->field2[i] != rhs->field2[i]) {
      return false;
    }
  }
  return true;
}

bool
dis_homework1__srv__CustomService_Request__copy(
  const dis_homework1__srv__CustomService_Request * input,
  dis_homework1__srv__CustomService_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // field1
  if (!rosidl_runtime_c__String__copy(
      &(input->field1), &(output->field1)))
  {
    return false;
  }
  // field2
  for (size_t i = 0; i < 10; ++i) {
    output->field2[i] = input->field2[i];
  }
  return true;
}

dis_homework1__srv__CustomService_Request *
dis_homework1__srv__CustomService_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dis_homework1__srv__CustomService_Request * msg = (dis_homework1__srv__CustomService_Request *)allocator.allocate(sizeof(dis_homework1__srv__CustomService_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dis_homework1__srv__CustomService_Request));
  bool success = dis_homework1__srv__CustomService_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dis_homework1__srv__CustomService_Request__destroy(dis_homework1__srv__CustomService_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dis_homework1__srv__CustomService_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dis_homework1__srv__CustomService_Request__Sequence__init(dis_homework1__srv__CustomService_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dis_homework1__srv__CustomService_Request * data = NULL;

  if (size) {
    data = (dis_homework1__srv__CustomService_Request *)allocator.zero_allocate(size, sizeof(dis_homework1__srv__CustomService_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dis_homework1__srv__CustomService_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dis_homework1__srv__CustomService_Request__fini(&data[i - 1]);
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
dis_homework1__srv__CustomService_Request__Sequence__fini(dis_homework1__srv__CustomService_Request__Sequence * array)
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
      dis_homework1__srv__CustomService_Request__fini(&array->data[i]);
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

dis_homework1__srv__CustomService_Request__Sequence *
dis_homework1__srv__CustomService_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dis_homework1__srv__CustomService_Request__Sequence * array = (dis_homework1__srv__CustomService_Request__Sequence *)allocator.allocate(sizeof(dis_homework1__srv__CustomService_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dis_homework1__srv__CustomService_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dis_homework1__srv__CustomService_Request__Sequence__destroy(dis_homework1__srv__CustomService_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dis_homework1__srv__CustomService_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dis_homework1__srv__CustomService_Request__Sequence__are_equal(const dis_homework1__srv__CustomService_Request__Sequence * lhs, const dis_homework1__srv__CustomService_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dis_homework1__srv__CustomService_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dis_homework1__srv__CustomService_Request__Sequence__copy(
  const dis_homework1__srv__CustomService_Request__Sequence * input,
  dis_homework1__srv__CustomService_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dis_homework1__srv__CustomService_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dis_homework1__srv__CustomService_Request * data =
      (dis_homework1__srv__CustomService_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dis_homework1__srv__CustomService_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dis_homework1__srv__CustomService_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dis_homework1__srv__CustomService_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `field3`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
dis_homework1__srv__CustomService_Response__init(dis_homework1__srv__CustomService_Response * msg)
{
  if (!msg) {
    return false;
  }
  // field3
  if (!rosidl_runtime_c__String__init(&msg->field3)) {
    dis_homework1__srv__CustomService_Response__fini(msg);
    return false;
  }
  // field4
  return true;
}

void
dis_homework1__srv__CustomService_Response__fini(dis_homework1__srv__CustomService_Response * msg)
{
  if (!msg) {
    return;
  }
  // field3
  rosidl_runtime_c__String__fini(&msg->field3);
  // field4
}

bool
dis_homework1__srv__CustomService_Response__are_equal(const dis_homework1__srv__CustomService_Response * lhs, const dis_homework1__srv__CustomService_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // field3
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->field3), &(rhs->field3)))
  {
    return false;
  }
  // field4
  if (lhs->field4 != rhs->field4) {
    return false;
  }
  return true;
}

bool
dis_homework1__srv__CustomService_Response__copy(
  const dis_homework1__srv__CustomService_Response * input,
  dis_homework1__srv__CustomService_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // field3
  if (!rosidl_runtime_c__String__copy(
      &(input->field3), &(output->field3)))
  {
    return false;
  }
  // field4
  output->field4 = input->field4;
  return true;
}

dis_homework1__srv__CustomService_Response *
dis_homework1__srv__CustomService_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dis_homework1__srv__CustomService_Response * msg = (dis_homework1__srv__CustomService_Response *)allocator.allocate(sizeof(dis_homework1__srv__CustomService_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dis_homework1__srv__CustomService_Response));
  bool success = dis_homework1__srv__CustomService_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dis_homework1__srv__CustomService_Response__destroy(dis_homework1__srv__CustomService_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dis_homework1__srv__CustomService_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dis_homework1__srv__CustomService_Response__Sequence__init(dis_homework1__srv__CustomService_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dis_homework1__srv__CustomService_Response * data = NULL;

  if (size) {
    data = (dis_homework1__srv__CustomService_Response *)allocator.zero_allocate(size, sizeof(dis_homework1__srv__CustomService_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dis_homework1__srv__CustomService_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dis_homework1__srv__CustomService_Response__fini(&data[i - 1]);
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
dis_homework1__srv__CustomService_Response__Sequence__fini(dis_homework1__srv__CustomService_Response__Sequence * array)
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
      dis_homework1__srv__CustomService_Response__fini(&array->data[i]);
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

dis_homework1__srv__CustomService_Response__Sequence *
dis_homework1__srv__CustomService_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dis_homework1__srv__CustomService_Response__Sequence * array = (dis_homework1__srv__CustomService_Response__Sequence *)allocator.allocate(sizeof(dis_homework1__srv__CustomService_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dis_homework1__srv__CustomService_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dis_homework1__srv__CustomService_Response__Sequence__destroy(dis_homework1__srv__CustomService_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dis_homework1__srv__CustomService_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dis_homework1__srv__CustomService_Response__Sequence__are_equal(const dis_homework1__srv__CustomService_Response__Sequence * lhs, const dis_homework1__srv__CustomService_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dis_homework1__srv__CustomService_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dis_homework1__srv__CustomService_Response__Sequence__copy(
  const dis_homework1__srv__CustomService_Response__Sequence * input,
  dis_homework1__srv__CustomService_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dis_homework1__srv__CustomService_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dis_homework1__srv__CustomService_Response * data =
      (dis_homework1__srv__CustomService_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dis_homework1__srv__CustomService_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dis_homework1__srv__CustomService_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dis_homework1__srv__CustomService_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
