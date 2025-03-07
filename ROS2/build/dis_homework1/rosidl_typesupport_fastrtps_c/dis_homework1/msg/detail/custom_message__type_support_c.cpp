// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from dis_homework1:msg/CustomMessage.idl
// generated code does not contain a copyright notice
#include "dis_homework1/msg/detail/custom_message__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "dis_homework1/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "dis_homework1/msg/detail/custom_message__struct.h"
#include "dis_homework1/msg/detail/custom_message__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // field1
#include "rosidl_runtime_c/string_functions.h"  // field1

// forward declare type support functions


using _CustomMessage__ros_msg_type = dis_homework1__msg__CustomMessage;

static bool _CustomMessage__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _CustomMessage__ros_msg_type * ros_message = static_cast<const _CustomMessage__ros_msg_type *>(untyped_ros_message);
  // Field name: field1
  {
    const rosidl_runtime_c__String * str = &ros_message->field1;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: field2
  {
    cdr << ros_message->field2;
  }

  // Field name: field3
  {
    cdr << (ros_message->field3 ? true : false);
  }

  return true;
}

static bool _CustomMessage__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _CustomMessage__ros_msg_type * ros_message = static_cast<_CustomMessage__ros_msg_type *>(untyped_ros_message);
  // Field name: field1
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->field1.data) {
      rosidl_runtime_c__String__init(&ros_message->field1);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->field1,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'field1'\n");
      return false;
    }
  }

  // Field name: field2
  {
    cdr >> ros_message->field2;
  }

  // Field name: field3
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->field3 = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dis_homework1
size_t get_serialized_size_dis_homework1__msg__CustomMessage(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CustomMessage__ros_msg_type * ros_message = static_cast<const _CustomMessage__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name field1
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->field1.size + 1);
  // field.name field2
  {
    size_t item_size = sizeof(ros_message->field2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name field3
  {
    size_t item_size = sizeof(ros_message->field3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _CustomMessage__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_dis_homework1__msg__CustomMessage(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dis_homework1
size_t max_serialized_size_dis_homework1__msg__CustomMessage(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: field1
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: field2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: field3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dis_homework1__msg__CustomMessage;
    is_plain =
      (
      offsetof(DataType, field3) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _CustomMessage__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_dis_homework1__msg__CustomMessage(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_CustomMessage = {
  "dis_homework1::msg",
  "CustomMessage",
  _CustomMessage__cdr_serialize,
  _CustomMessage__cdr_deserialize,
  _CustomMessage__get_serialized_size,
  _CustomMessage__max_serialized_size
};

static rosidl_message_type_support_t _CustomMessage__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_CustomMessage,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dis_homework1, msg, CustomMessage)() {
  return &_CustomMessage__type_support;
}

#if defined(__cplusplus)
}
#endif
