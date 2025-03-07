// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dis_homework1:msg/CustomMessage.idl
// generated code does not contain a copyright notice

#ifndef DIS_HOMEWORK1__MSG__DETAIL__CUSTOM_MESSAGE__TRAITS_HPP_
#define DIS_HOMEWORK1__MSG__DETAIL__CUSTOM_MESSAGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dis_homework1/msg/detail/custom_message__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dis_homework1
{

namespace msg
{

inline void to_flow_style_yaml(
  const CustomMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: field1
  {
    out << "field1: ";
    rosidl_generator_traits::value_to_yaml(msg.field1, out);
    out << ", ";
  }

  // member: field2
  {
    out << "field2: ";
    rosidl_generator_traits::value_to_yaml(msg.field2, out);
    out << ", ";
  }

  // member: field3
  {
    out << "field3: ";
    rosidl_generator_traits::value_to_yaml(msg.field3, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CustomMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: field1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "field1: ";
    rosidl_generator_traits::value_to_yaml(msg.field1, out);
    out << "\n";
  }

  // member: field2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "field2: ";
    rosidl_generator_traits::value_to_yaml(msg.field2, out);
    out << "\n";
  }

  // member: field3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "field3: ";
    rosidl_generator_traits::value_to_yaml(msg.field3, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CustomMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace dis_homework1

namespace rosidl_generator_traits
{

[[deprecated("use dis_homework1::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dis_homework1::msg::CustomMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  dis_homework1::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dis_homework1::msg::to_yaml() instead")]]
inline std::string to_yaml(const dis_homework1::msg::CustomMessage & msg)
{
  return dis_homework1::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dis_homework1::msg::CustomMessage>()
{
  return "dis_homework1::msg::CustomMessage";
}

template<>
inline const char * name<dis_homework1::msg::CustomMessage>()
{
  return "dis_homework1/msg/CustomMessage";
}

template<>
struct has_fixed_size<dis_homework1::msg::CustomMessage>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dis_homework1::msg::CustomMessage>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dis_homework1::msg::CustomMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DIS_HOMEWORK1__MSG__DETAIL__CUSTOM_MESSAGE__TRAITS_HPP_
