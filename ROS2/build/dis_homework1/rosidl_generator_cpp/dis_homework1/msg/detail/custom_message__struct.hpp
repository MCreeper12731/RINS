// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dis_homework1:msg/CustomMessage.idl
// generated code does not contain a copyright notice

#ifndef DIS_HOMEWORK1__MSG__DETAIL__CUSTOM_MESSAGE__STRUCT_HPP_
#define DIS_HOMEWORK1__MSG__DETAIL__CUSTOM_MESSAGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dis_homework1__msg__CustomMessage __attribute__((deprecated))
#else
# define DEPRECATED__dis_homework1__msg__CustomMessage __declspec(deprecated)
#endif

namespace dis_homework1
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CustomMessage_
{
  using Type = CustomMessage_<ContainerAllocator>;

  explicit CustomMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->field1 = "";
      this->field2 = 0;
      this->field3 = false;
    }
  }

  explicit CustomMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : field1(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->field1 = "";
      this->field2 = 0;
      this->field3 = false;
    }
  }

  // field types and members
  using _field1_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _field1_type field1;
  using _field2_type =
    uint8_t;
  _field2_type field2;
  using _field3_type =
    bool;
  _field3_type field3;

  // setters for named parameter idiom
  Type & set__field1(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->field1 = _arg;
    return *this;
  }
  Type & set__field2(
    const uint8_t & _arg)
  {
    this->field2 = _arg;
    return *this;
  }
  Type & set__field3(
    const bool & _arg)
  {
    this->field3 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dis_homework1::msg::CustomMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const dis_homework1::msg::CustomMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dis_homework1::msg::CustomMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dis_homework1::msg::CustomMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dis_homework1::msg::CustomMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dis_homework1::msg::CustomMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dis_homework1::msg::CustomMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dis_homework1::msg::CustomMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dis_homework1::msg::CustomMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dis_homework1::msg::CustomMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dis_homework1__msg__CustomMessage
    std::shared_ptr<dis_homework1::msg::CustomMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dis_homework1__msg__CustomMessage
    std::shared_ptr<dis_homework1::msg::CustomMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CustomMessage_ & other) const
  {
    if (this->field1 != other.field1) {
      return false;
    }
    if (this->field2 != other.field2) {
      return false;
    }
    if (this->field3 != other.field3) {
      return false;
    }
    return true;
  }
  bool operator!=(const CustomMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CustomMessage_

// alias to use template instance with default allocator
using CustomMessage =
  dis_homework1::msg::CustomMessage_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dis_homework1

#endif  // DIS_HOMEWORK1__MSG__DETAIL__CUSTOM_MESSAGE__STRUCT_HPP_
