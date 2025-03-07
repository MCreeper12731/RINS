// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from dis_homework1:msg/CustomMessage.idl
// generated code does not contain a copyright notice

#ifndef DIS_HOMEWORK1__MSG__DETAIL__CUSTOM_MESSAGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define DIS_HOMEWORK1__MSG__DETAIL__CUSTOM_MESSAGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "dis_homework1/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "dis_homework1/msg/detail/custom_message__struct.hpp"

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

#include "fastcdr/Cdr.h"

namespace dis_homework1
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dis_homework1
cdr_serialize(
  const dis_homework1::msg::CustomMessage & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dis_homework1
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  dis_homework1::msg::CustomMessage & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dis_homework1
get_serialized_size(
  const dis_homework1::msg::CustomMessage & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dis_homework1
max_serialized_size_CustomMessage(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace dis_homework1

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dis_homework1
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dis_homework1, msg, CustomMessage)();

#ifdef __cplusplus
}
#endif

#endif  // DIS_HOMEWORK1__MSG__DETAIL__CUSTOM_MESSAGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
