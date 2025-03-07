// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dis_homework1:srv/CustomService.idl
// generated code does not contain a copyright notice

#ifndef DIS_HOMEWORK1__SRV__DETAIL__CUSTOM_SERVICE__BUILDER_HPP_
#define DIS_HOMEWORK1__SRV__DETAIL__CUSTOM_SERVICE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dis_homework1/srv/detail/custom_service__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dis_homework1
{

namespace srv
{

namespace builder
{

class Init_CustomService_Request_field2
{
public:
  explicit Init_CustomService_Request_field2(::dis_homework1::srv::CustomService_Request & msg)
  : msg_(msg)
  {}
  ::dis_homework1::srv::CustomService_Request field2(::dis_homework1::srv::CustomService_Request::_field2_type arg)
  {
    msg_.field2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dis_homework1::srv::CustomService_Request msg_;
};

class Init_CustomService_Request_field1
{
public:
  Init_CustomService_Request_field1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CustomService_Request_field2 field1(::dis_homework1::srv::CustomService_Request::_field1_type arg)
  {
    msg_.field1 = std::move(arg);
    return Init_CustomService_Request_field2(msg_);
  }

private:
  ::dis_homework1::srv::CustomService_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dis_homework1::srv::CustomService_Request>()
{
  return dis_homework1::srv::builder::Init_CustomService_Request_field1();
}

}  // namespace dis_homework1


namespace dis_homework1
{

namespace srv
{

namespace builder
{

class Init_CustomService_Response_field4
{
public:
  explicit Init_CustomService_Response_field4(::dis_homework1::srv::CustomService_Response & msg)
  : msg_(msg)
  {}
  ::dis_homework1::srv::CustomService_Response field4(::dis_homework1::srv::CustomService_Response::_field4_type arg)
  {
    msg_.field4 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dis_homework1::srv::CustomService_Response msg_;
};

class Init_CustomService_Response_field3
{
public:
  Init_CustomService_Response_field3()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CustomService_Response_field4 field3(::dis_homework1::srv::CustomService_Response::_field3_type arg)
  {
    msg_.field3 = std::move(arg);
    return Init_CustomService_Response_field4(msg_);
  }

private:
  ::dis_homework1::srv::CustomService_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dis_homework1::srv::CustomService_Response>()
{
  return dis_homework1::srv::builder::Init_CustomService_Response_field3();
}

}  // namespace dis_homework1

#endif  // DIS_HOMEWORK1__SRV__DETAIL__CUSTOM_SERVICE__BUILDER_HPP_
