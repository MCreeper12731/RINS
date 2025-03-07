// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dis_homework1:msg/CustomMessage.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dis_homework1/msg/detail/custom_message__rosidl_typesupport_introspection_c.h"
#include "dis_homework1/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dis_homework1/msg/detail/custom_message__functions.h"
#include "dis_homework1/msg/detail/custom_message__struct.h"


// Include directives for member types
// Member `field1`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dis_homework1__msg__CustomMessage__rosidl_typesupport_introspection_c__CustomMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dis_homework1__msg__CustomMessage__init(message_memory);
}

void dis_homework1__msg__CustomMessage__rosidl_typesupport_introspection_c__CustomMessage_fini_function(void * message_memory)
{
  dis_homework1__msg__CustomMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember dis_homework1__msg__CustomMessage__rosidl_typesupport_introspection_c__CustomMessage_message_member_array[3] = {
  {
    "field1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dis_homework1__msg__CustomMessage, field1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "field2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dis_homework1__msg__CustomMessage, field2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "field3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dis_homework1__msg__CustomMessage, field3),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dis_homework1__msg__CustomMessage__rosidl_typesupport_introspection_c__CustomMessage_message_members = {
  "dis_homework1__msg",  // message namespace
  "CustomMessage",  // message name
  3,  // number of fields
  sizeof(dis_homework1__msg__CustomMessage),
  dis_homework1__msg__CustomMessage__rosidl_typesupport_introspection_c__CustomMessage_message_member_array,  // message members
  dis_homework1__msg__CustomMessage__rosidl_typesupport_introspection_c__CustomMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  dis_homework1__msg__CustomMessage__rosidl_typesupport_introspection_c__CustomMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dis_homework1__msg__CustomMessage__rosidl_typesupport_introspection_c__CustomMessage_message_type_support_handle = {
  0,
  &dis_homework1__msg__CustomMessage__rosidl_typesupport_introspection_c__CustomMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dis_homework1
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dis_homework1, msg, CustomMessage)() {
  if (!dis_homework1__msg__CustomMessage__rosidl_typesupport_introspection_c__CustomMessage_message_type_support_handle.typesupport_identifier) {
    dis_homework1__msg__CustomMessage__rosidl_typesupport_introspection_c__CustomMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dis_homework1__msg__CustomMessage__rosidl_typesupport_introspection_c__CustomMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
