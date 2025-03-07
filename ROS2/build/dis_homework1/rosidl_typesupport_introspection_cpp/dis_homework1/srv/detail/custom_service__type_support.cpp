// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dis_homework1:srv/CustomService.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dis_homework1/srv/detail/custom_service__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace dis_homework1
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void CustomService_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dis_homework1::srv::CustomService_Request(_init);
}

void CustomService_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dis_homework1::srv::CustomService_Request *>(message_memory);
  typed_message->~CustomService_Request();
}

size_t size_function__CustomService_Request__field2(const void * untyped_member)
{
  (void)untyped_member;
  return 10;
}

const void * get_const_function__CustomService_Request__field2(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int64_t, 10> *>(untyped_member);
  return &member[index];
}

void * get_function__CustomService_Request__field2(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int64_t, 10> *>(untyped_member);
  return &member[index];
}

void fetch_function__CustomService_Request__field2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int64_t *>(
    get_const_function__CustomService_Request__field2(untyped_member, index));
  auto & value = *reinterpret_cast<int64_t *>(untyped_value);
  value = item;
}

void assign_function__CustomService_Request__field2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int64_t *>(
    get_function__CustomService_Request__field2(untyped_member, index));
  const auto & value = *reinterpret_cast<const int64_t *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CustomService_Request_message_member_array[2] = {
  {
    "field1",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dis_homework1::srv::CustomService_Request, field1),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "field2",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    10,  // array size
    false,  // is upper bound
    offsetof(dis_homework1::srv::CustomService_Request, field2),  // bytes offset in struct
    nullptr,  // default value
    size_function__CustomService_Request__field2,  // size() function pointer
    get_const_function__CustomService_Request__field2,  // get_const(index) function pointer
    get_function__CustomService_Request__field2,  // get(index) function pointer
    fetch_function__CustomService_Request__field2,  // fetch(index, &value) function pointer
    assign_function__CustomService_Request__field2,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CustomService_Request_message_members = {
  "dis_homework1::srv",  // message namespace
  "CustomService_Request",  // message name
  2,  // number of fields
  sizeof(dis_homework1::srv::CustomService_Request),
  CustomService_Request_message_member_array,  // message members
  CustomService_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  CustomService_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CustomService_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CustomService_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace dis_homework1


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dis_homework1::srv::CustomService_Request>()
{
  return &::dis_homework1::srv::rosidl_typesupport_introspection_cpp::CustomService_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dis_homework1, srv, CustomService_Request)() {
  return &::dis_homework1::srv::rosidl_typesupport_introspection_cpp::CustomService_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "dis_homework1/srv/detail/custom_service__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace dis_homework1
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void CustomService_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dis_homework1::srv::CustomService_Response(_init);
}

void CustomService_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dis_homework1::srv::CustomService_Response *>(message_memory);
  typed_message->~CustomService_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CustomService_Response_message_member_array[2] = {
  {
    "field3",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dis_homework1::srv::CustomService_Response, field3),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "field4",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dis_homework1::srv::CustomService_Response, field4),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CustomService_Response_message_members = {
  "dis_homework1::srv",  // message namespace
  "CustomService_Response",  // message name
  2,  // number of fields
  sizeof(dis_homework1::srv::CustomService_Response),
  CustomService_Response_message_member_array,  // message members
  CustomService_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  CustomService_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CustomService_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CustomService_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace dis_homework1


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dis_homework1::srv::CustomService_Response>()
{
  return &::dis_homework1::srv::rosidl_typesupport_introspection_cpp::CustomService_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dis_homework1, srv, CustomService_Response)() {
  return &::dis_homework1::srv::rosidl_typesupport_introspection_cpp::CustomService_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "dis_homework1/srv/detail/custom_service__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace dis_homework1
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers CustomService_service_members = {
  "dis_homework1::srv",  // service namespace
  "CustomService",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<dis_homework1::srv::CustomService>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t CustomService_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CustomService_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace dis_homework1


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<dis_homework1::srv::CustomService>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::dis_homework1::srv::rosidl_typesupport_introspection_cpp::CustomService_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::dis_homework1::srv::CustomService_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::dis_homework1::srv::CustomService_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dis_homework1, srv, CustomService)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<dis_homework1::srv::CustomService>();
}

#ifdef __cplusplus
}
#endif
