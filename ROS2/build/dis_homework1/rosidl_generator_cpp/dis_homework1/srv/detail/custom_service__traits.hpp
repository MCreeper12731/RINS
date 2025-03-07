// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dis_homework1:srv/CustomService.idl
// generated code does not contain a copyright notice

#ifndef DIS_HOMEWORK1__SRV__DETAIL__CUSTOM_SERVICE__TRAITS_HPP_
#define DIS_HOMEWORK1__SRV__DETAIL__CUSTOM_SERVICE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dis_homework1/srv/detail/custom_service__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dis_homework1
{

namespace srv
{

inline void to_flow_style_yaml(
  const CustomService_Request & msg,
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
    if (msg.field2.size() == 0) {
      out << "field2: []";
    } else {
      out << "field2: [";
      size_t pending_items = msg.field2.size();
      for (auto item : msg.field2) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CustomService_Request & msg,
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
    if (msg.field2.size() == 0) {
      out << "field2: []\n";
    } else {
      out << "field2:\n";
      for (auto item : msg.field2) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CustomService_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace dis_homework1

namespace rosidl_generator_traits
{

[[deprecated("use dis_homework1::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dis_homework1::srv::CustomService_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  dis_homework1::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dis_homework1::srv::to_yaml() instead")]]
inline std::string to_yaml(const dis_homework1::srv::CustomService_Request & msg)
{
  return dis_homework1::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dis_homework1::srv::CustomService_Request>()
{
  return "dis_homework1::srv::CustomService_Request";
}

template<>
inline const char * name<dis_homework1::srv::CustomService_Request>()
{
  return "dis_homework1/srv/CustomService_Request";
}

template<>
struct has_fixed_size<dis_homework1::srv::CustomService_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dis_homework1::srv::CustomService_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dis_homework1::srv::CustomService_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace dis_homework1
{

namespace srv
{

inline void to_flow_style_yaml(
  const CustomService_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: field3
  {
    out << "field3: ";
    rosidl_generator_traits::value_to_yaml(msg.field3, out);
    out << ", ";
  }

  // member: field4
  {
    out << "field4: ";
    rosidl_generator_traits::value_to_yaml(msg.field4, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CustomService_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: field3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "field3: ";
    rosidl_generator_traits::value_to_yaml(msg.field3, out);
    out << "\n";
  }

  // member: field4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "field4: ";
    rosidl_generator_traits::value_to_yaml(msg.field4, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CustomService_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace dis_homework1

namespace rosidl_generator_traits
{

[[deprecated("use dis_homework1::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dis_homework1::srv::CustomService_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  dis_homework1::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dis_homework1::srv::to_yaml() instead")]]
inline std::string to_yaml(const dis_homework1::srv::CustomService_Response & msg)
{
  return dis_homework1::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dis_homework1::srv::CustomService_Response>()
{
  return "dis_homework1::srv::CustomService_Response";
}

template<>
inline const char * name<dis_homework1::srv::CustomService_Response>()
{
  return "dis_homework1/srv/CustomService_Response";
}

template<>
struct has_fixed_size<dis_homework1::srv::CustomService_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dis_homework1::srv::CustomService_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dis_homework1::srv::CustomService_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dis_homework1::srv::CustomService>()
{
  return "dis_homework1::srv::CustomService";
}

template<>
inline const char * name<dis_homework1::srv::CustomService>()
{
  return "dis_homework1/srv/CustomService";
}

template<>
struct has_fixed_size<dis_homework1::srv::CustomService>
  : std::integral_constant<
    bool,
    has_fixed_size<dis_homework1::srv::CustomService_Request>::value &&
    has_fixed_size<dis_homework1::srv::CustomService_Response>::value
  >
{
};

template<>
struct has_bounded_size<dis_homework1::srv::CustomService>
  : std::integral_constant<
    bool,
    has_bounded_size<dis_homework1::srv::CustomService_Request>::value &&
    has_bounded_size<dis_homework1::srv::CustomService_Response>::value
  >
{
};

template<>
struct is_service<dis_homework1::srv::CustomService>
  : std::true_type
{
};

template<>
struct is_service_request<dis_homework1::srv::CustomService_Request>
  : std::true_type
{
};

template<>
struct is_service_response<dis_homework1::srv::CustomService_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DIS_HOMEWORK1__SRV__DETAIL__CUSTOM_SERVICE__TRAITS_HPP_
