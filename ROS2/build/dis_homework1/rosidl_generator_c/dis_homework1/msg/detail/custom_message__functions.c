// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dis_homework1:msg/CustomMessage.idl
// generated code does not contain a copyright notice
#include "dis_homework1/msg/detail/custom_message__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `field1`
#include "rosidl_runtime_c/string_functions.h"

bool
dis_homework1__msg__CustomMessage__init(dis_homework1__msg__CustomMessage * msg)
{
  if (!msg) {
    return false;
  }
  // field1
  if (!rosidl_runtime_c__String__init(&msg->field1)) {
    dis_homework1__msg__CustomMessage__fini(msg);
    return false;
  }
  // field2
  // field3
  return true;
}

void
dis_homework1__msg__CustomMessage__fini(dis_homework1__msg__CustomMessage * msg)
{
  if (!msg) {
    return;
  }
  // field1
  rosidl_runtime_c__String__fini(&msg->field1);
  // field2
  // field3
}

bool
dis_homework1__msg__CustomMessage__are_equal(const dis_homework1__msg__CustomMessage * lhs, const dis_homework1__msg__CustomMessage * rhs)
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
  if (lhs->field2 != rhs->field2) {
    return false;
  }
  // field3
  if (lhs->field3 != rhs->field3) {
    return false;
  }
  return true;
}

bool
dis_homework1__msg__CustomMessage__copy(
  const dis_homework1__msg__CustomMessage * input,
  dis_homework1__msg__CustomMessage * output)
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
  output->field2 = input->field2;
  // field3
  output->field3 = input->field3;
  return true;
}

dis_homework1__msg__CustomMessage *
dis_homework1__msg__CustomMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dis_homework1__msg__CustomMessage * msg = (dis_homework1__msg__CustomMessage *)allocator.allocate(sizeof(dis_homework1__msg__CustomMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dis_homework1__msg__CustomMessage));
  bool success = dis_homework1__msg__CustomMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dis_homework1__msg__CustomMessage__destroy(dis_homework1__msg__CustomMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dis_homework1__msg__CustomMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dis_homework1__msg__CustomMessage__Sequence__init(dis_homework1__msg__CustomMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dis_homework1__msg__CustomMessage * data = NULL;

  if (size) {
    data = (dis_homework1__msg__CustomMessage *)allocator.zero_allocate(size, sizeof(dis_homework1__msg__CustomMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dis_homework1__msg__CustomMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dis_homework1__msg__CustomMessage__fini(&data[i - 1]);
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
dis_homework1__msg__CustomMessage__Sequence__fini(dis_homework1__msg__CustomMessage__Sequence * array)
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
      dis_homework1__msg__CustomMessage__fini(&array->data[i]);
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

dis_homework1__msg__CustomMessage__Sequence *
dis_homework1__msg__CustomMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dis_homework1__msg__CustomMessage__Sequence * array = (dis_homework1__msg__CustomMessage__Sequence *)allocator.allocate(sizeof(dis_homework1__msg__CustomMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dis_homework1__msg__CustomMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dis_homework1__msg__CustomMessage__Sequence__destroy(dis_homework1__msg__CustomMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dis_homework1__msg__CustomMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dis_homework1__msg__CustomMessage__Sequence__are_equal(const dis_homework1__msg__CustomMessage__Sequence * lhs, const dis_homework1__msg__CustomMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dis_homework1__msg__CustomMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dis_homework1__msg__CustomMessage__Sequence__copy(
  const dis_homework1__msg__CustomMessage__Sequence * input,
  dis_homework1__msg__CustomMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dis_homework1__msg__CustomMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dis_homework1__msg__CustomMessage * data =
      (dis_homework1__msg__CustomMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dis_homework1__msg__CustomMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dis_homework1__msg__CustomMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dis_homework1__msg__CustomMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
