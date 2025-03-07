// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dis_homework1:msg/CustomMessage.idl
// generated code does not contain a copyright notice

#ifndef DIS_HOMEWORK1__MSG__DETAIL__CUSTOM_MESSAGE__STRUCT_H_
#define DIS_HOMEWORK1__MSG__DETAIL__CUSTOM_MESSAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'field1'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/CustomMessage in the package dis_homework1.
typedef struct dis_homework1__msg__CustomMessage
{
  rosidl_runtime_c__String field1;
  uint8_t field2;
  bool field3;
} dis_homework1__msg__CustomMessage;

// Struct for a sequence of dis_homework1__msg__CustomMessage.
typedef struct dis_homework1__msg__CustomMessage__Sequence
{
  dis_homework1__msg__CustomMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dis_homework1__msg__CustomMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DIS_HOMEWORK1__MSG__DETAIL__CUSTOM_MESSAGE__STRUCT_H_
