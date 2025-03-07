// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dis_homework1:srv/CustomService.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dis_homework1/srv/detail/custom_service__rosidl_typesupport_introspection_c.h"
#include "dis_homework1/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dis_homework1/srv/detail/custom_service__functions.h"
#include "dis_homework1/srv/detail/custom_service__struct.h"


// Include directives for member types
// Member `field1`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__CustomService_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dis_homework1__srv__CustomService_Request__init(message_memory);
}

void dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__CustomService_Request_fini_function(void * message_memory)
{
  dis_homework1__srv__CustomService_Request__fini(message_memory);
}

size_t dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__size_function__CustomService_Request__field2(
  const void * untyped_member)
{
  (void)untyped_member;
  return 10;
}

const void * dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__get_const_function__CustomService_Request__field2(
  const void * untyped_member, size_t index)
{
  const int64_t * member =
    (const int64_t *)(untyped_member);
  return &member[index];
}

void * dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__get_function__CustomService_Request__field2(
  void * untyped_member, size_t index)
{
  int64_t * member =
    (int64_t *)(untyped_member);
  return &member[index];
}

void dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__fetch_function__CustomService_Request__field2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int64_t * item =
    ((const int64_t *)
    dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__get_const_function__CustomService_Request__field2(untyped_member, index));
  int64_t * value =
    (int64_t *)(untyped_value);
  *value = *item;
}

void dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__assign_function__CustomService_Request__field2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int64_t * item =
    ((int64_t *)
    dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__get_function__CustomService_Request__field2(untyped_member, index));
  const int64_t * value =
    (const int64_t *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__CustomService_Request_message_member_array[2] = {
  {
    "field1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dis_homework1__srv__CustomService_Request, field1),  // bytes offset in struct
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
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    10,  // array size
    false,  // is upper bound
    offsetof(dis_homework1__srv__CustomService_Request, field2),  // bytes offset in struct
    NULL,  // default value
    dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__size_function__CustomService_Request__field2,  // size() function pointer
    dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__get_const_function__CustomService_Request__field2,  // get_const(index) function pointer
    dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__get_function__CustomService_Request__field2,  // get(index) function pointer
    dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__fetch_function__CustomService_Request__field2,  // fetch(index, &value) function pointer
    dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__assign_function__CustomService_Request__field2,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__CustomService_Request_message_members = {
  "dis_homework1__srv",  // message namespace
  "CustomService_Request",  // message name
  2,  // number of fields
  sizeof(dis_homework1__srv__CustomService_Request),
  dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__CustomService_Request_message_member_array,  // message members
  dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__CustomService_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__CustomService_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__CustomService_Request_message_type_support_handle = {
  0,
  &dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__CustomService_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dis_homework1
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dis_homework1, srv, CustomService_Request)() {
  if (!dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__CustomService_Request_message_type_support_handle.typesupport_identifier) {
    dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__CustomService_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dis_homework1__srv__CustomService_Request__rosidl_typesupport_introspection_c__CustomService_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "dis_homework1/srv/detail/custom_service__rosidl_typesupport_introspection_c.h"
// already included above
// #include "dis_homework1/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "dis_homework1/srv/detail/custom_service__functions.h"
// already included above
// #include "dis_homework1/srv/detail/custom_service__struct.h"


// Include directives for member types
// Member `field3`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dis_homework1__srv__CustomService_Response__rosidl_typesupport_introspection_c__CustomService_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dis_homework1__srv__CustomService_Response__init(message_memory);
}

void dis_homework1__srv__CustomService_Response__rosidl_typesupport_introspection_c__CustomService_Response_fini_function(void * message_memory)
{
  dis_homework1__srv__CustomService_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember dis_homework1__srv__CustomService_Response__rosidl_typesupport_introspection_c__CustomService_Response_message_member_array[2] = {
  {
    "field3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dis_homework1__srv__CustomService_Response, field3),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "field4",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dis_homework1__srv__CustomService_Response, field4),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dis_homework1__srv__CustomService_Response__rosidl_typesupport_introspection_c__CustomService_Response_message_members = {
  "dis_homework1__srv",  // message namespace
  "CustomService_Response",  // message name
  2,  // number of fields
  sizeof(dis_homework1__srv__CustomService_Response),
  dis_homework1__srv__CustomService_Response__rosidl_typesupport_introspection_c__CustomService_Response_message_member_array,  // message members
  dis_homework1__srv__CustomService_Response__rosidl_typesupport_introspection_c__CustomService_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  dis_homework1__srv__CustomService_Response__rosidl_typesupport_introspection_c__CustomService_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dis_homework1__srv__CustomService_Response__rosidl_typesupport_introspection_c__CustomService_Response_message_type_support_handle = {
  0,
  &dis_homework1__srv__CustomService_Response__rosidl_typesupport_introspection_c__CustomService_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dis_homework1
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dis_homework1, srv, CustomService_Response)() {
  if (!dis_homework1__srv__CustomService_Response__rosidl_typesupport_introspection_c__CustomService_Response_message_type_support_handle.typesupport_identifier) {
    dis_homework1__srv__CustomService_Response__rosidl_typesupport_introspection_c__CustomService_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dis_homework1__srv__CustomService_Response__rosidl_typesupport_introspection_c__CustomService_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "dis_homework1/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "dis_homework1/srv/detail/custom_service__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers dis_homework1__srv__detail__custom_service__rosidl_typesupport_introspection_c__CustomService_service_members = {
  "dis_homework1__srv",  // service namespace
  "CustomService",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // dis_homework1__srv__detail__custom_service__rosidl_typesupport_introspection_c__CustomService_Request_message_type_support_handle,
  NULL  // response message
  // dis_homework1__srv__detail__custom_service__rosidl_typesupport_introspection_c__CustomService_Response_message_type_support_handle
};

static rosidl_service_type_support_t dis_homework1__srv__detail__custom_service__rosidl_typesupport_introspection_c__CustomService_service_type_support_handle = {
  0,
  &dis_homework1__srv__detail__custom_service__rosidl_typesupport_introspection_c__CustomService_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dis_homework1, srv, CustomService_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dis_homework1, srv, CustomService_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dis_homework1
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dis_homework1, srv, CustomService)() {
  if (!dis_homework1__srv__detail__custom_service__rosidl_typesupport_introspection_c__CustomService_service_type_support_handle.typesupport_identifier) {
    dis_homework1__srv__detail__custom_service__rosidl_typesupport_introspection_c__CustomService_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)dis_homework1__srv__detail__custom_service__rosidl_typesupport_introspection_c__CustomService_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dis_homework1, srv, CustomService_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dis_homework1, srv, CustomService_Response)()->data;
  }

  return &dis_homework1__srv__detail__custom_service__rosidl_typesupport_introspection_c__CustomService_service_type_support_handle;
}
