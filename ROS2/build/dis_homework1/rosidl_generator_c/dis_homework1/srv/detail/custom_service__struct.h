// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dis_homework1:srv/CustomService.idl
// generated code does not contain a copyright notice

#ifndef DIS_HOMEWORK1__SRV__DETAIL__CUSTOM_SERVICE__STRUCT_H_
#define DIS_HOMEWORK1__SRV__DETAIL__CUSTOM_SERVICE__STRUCT_H_

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

/// Struct defined in srv/CustomService in the package dis_homework1.
typedef struct dis_homework1__srv__CustomService_Request
{
  rosidl_runtime_c__String field1;
  int64_t field2[10];
} dis_homework1__srv__CustomService_Request;

// Struct for a sequence of dis_homework1__srv__CustomService_Request.
typedef struct dis_homework1__srv__CustomService_Request__Sequence
{
  dis_homework1__srv__CustomService_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dis_homework1__srv__CustomService_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'field3'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/CustomService in the package dis_homework1.
typedef struct dis_homework1__srv__CustomService_Response
{
  rosidl_runtime_c__String field3;
  int64_t field4;
} dis_homework1__srv__CustomService_Response;

// Struct for a sequence of dis_homework1__srv__CustomService_Response.
typedef struct dis_homework1__srv__CustomService_Response__Sequence
{
  dis_homework1__srv__CustomService_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dis_homework1__srv__CustomService_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DIS_HOMEWORK1__SRV__DETAIL__CUSTOM_SERVICE__STRUCT_H_
