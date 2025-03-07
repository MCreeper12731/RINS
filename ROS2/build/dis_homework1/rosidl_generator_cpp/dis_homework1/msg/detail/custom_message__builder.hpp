// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dis_homework1:msg/CustomMessage.idl
// generated code does not contain a copyright notice

#ifndef DIS_HOMEWORK1__MSG__DETAIL__CUSTOM_MESSAGE__BUILDER_HPP_
#define DIS_HOMEWORK1__MSG__DETAIL__CUSTOM_MESSAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dis_homework1/msg/detail/custom_message__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dis_homework1
{

namespace msg
{

namespace builder
{

class Init_CustomMessage_field3
{
public:
  explicit Init_CustomMessage_field3(::dis_homework1::msg::CustomMessage & msg)
  : msg_(msg)
  {}
  ::dis_homework1::msg::CustomMessage field3(::dis_homework1::msg::CustomMessage::_field3_type arg)
  {
    msg_.field3 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dis_homework1::msg::CustomMessage msg_;
};

class Init_CustomMessage_field2
{
public:
  explicit Init_CustomMessage_field2(::dis_homework1::msg::CustomMessage & msg)
  : msg_(msg)
  {}
  Init_CustomMessage_field3 field2(::dis_homework1::msg::CustomMessage::_field2_type arg)
  {
    msg_.field2 = std::move(arg);
    return Init_CustomMessage_field3(msg_);
  }

private:
  ::dis_homework1::msg::CustomMessage msg_;
};

class Init_CustomMessage_field1
{
public:
  Init_CustomMessage_field1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CustomMessage_field2 field1(::dis_homework1::msg::CustomMessage::_field1_type arg)
  {
    msg_.field1 = std::move(arg);
    return Init_CustomMessage_field2(msg_);
  }

private:
  ::dis_homework1::msg::CustomMessage msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dis_homework1::msg::CustomMessage>()
{
  return dis_homework1::msg::builder::Init_CustomMessage_field1();
}

}  // namespace dis_homework1

#endif  // DIS_HOMEWORK1__MSG__DETAIL__CUSTOM_MESSAGE__BUILDER_HPP_
