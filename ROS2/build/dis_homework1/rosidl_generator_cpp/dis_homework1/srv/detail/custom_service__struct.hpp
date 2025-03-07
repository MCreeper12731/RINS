// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dis_homework1:srv/CustomService.idl
// generated code does not contain a copyright notice

#ifndef DIS_HOMEWORK1__SRV__DETAIL__CUSTOM_SERVICE__STRUCT_HPP_
#define DIS_HOMEWORK1__SRV__DETAIL__CUSTOM_SERVICE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dis_homework1__srv__CustomService_Request __attribute__((deprecated))
#else
# define DEPRECATED__dis_homework1__srv__CustomService_Request __declspec(deprecated)
#endif

namespace dis_homework1
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CustomService_Request_
{
  using Type = CustomService_Request_<ContainerAllocator>;

  explicit CustomService_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->field1 = "";
      std::fill<typename std::array<int64_t, 10>::iterator, int64_t>(this->field2.begin(), this->field2.end(), 0ll);
    }
  }

  explicit CustomService_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : field1(_alloc),
    field2(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->field1 = "";
      std::fill<typename std::array<int64_t, 10>::iterator, int64_t>(this->field2.begin(), this->field2.end(), 0ll);
    }
  }

  // field types and members
  using _field1_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _field1_type field1;
  using _field2_type =
    std::array<int64_t, 10>;
  _field2_type field2;

  // setters for named parameter idiom
  Type & set__field1(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->field1 = _arg;
    return *this;
  }
  Type & set__field2(
    const std::array<int64_t, 10> & _arg)
  {
    this->field2 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dis_homework1::srv::CustomService_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dis_homework1::srv::CustomService_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dis_homework1::srv::CustomService_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dis_homework1::srv::CustomService_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dis_homework1::srv::CustomService_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dis_homework1::srv::CustomService_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dis_homework1::srv::CustomService_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dis_homework1::srv::CustomService_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dis_homework1::srv::CustomService_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dis_homework1::srv::CustomService_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dis_homework1__srv__CustomService_Request
    std::shared_ptr<dis_homework1::srv::CustomService_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dis_homework1__srv__CustomService_Request
    std::shared_ptr<dis_homework1::srv::CustomService_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CustomService_Request_ & other) const
  {
    if (this->field1 != other.field1) {
      return false;
    }
    if (this->field2 != other.field2) {
      return false;
    }
    return true;
  }
  bool operator!=(const CustomService_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CustomService_Request_

// alias to use template instance with default allocator
using CustomService_Request =
  dis_homework1::srv::CustomService_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dis_homework1


#ifndef _WIN32
# define DEPRECATED__dis_homework1__srv__CustomService_Response __attribute__((deprecated))
#else
# define DEPRECATED__dis_homework1__srv__CustomService_Response __declspec(deprecated)
#endif

namespace dis_homework1
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CustomService_Response_
{
  using Type = CustomService_Response_<ContainerAllocator>;

  explicit CustomService_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->field3 = "";
      this->field4 = 0ll;
    }
  }

  explicit CustomService_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : field3(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->field3 = "";
      this->field4 = 0ll;
    }
  }

  // field types and members
  using _field3_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _field3_type field3;
  using _field4_type =
    int64_t;
  _field4_type field4;

  // setters for named parameter idiom
  Type & set__field3(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->field3 = _arg;
    return *this;
  }
  Type & set__field4(
    const int64_t & _arg)
  {
    this->field4 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dis_homework1::srv::CustomService_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dis_homework1::srv::CustomService_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dis_homework1::srv::CustomService_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dis_homework1::srv::CustomService_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dis_homework1::srv::CustomService_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dis_homework1::srv::CustomService_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dis_homework1::srv::CustomService_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dis_homework1::srv::CustomService_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dis_homework1::srv::CustomService_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dis_homework1::srv::CustomService_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dis_homework1__srv__CustomService_Response
    std::shared_ptr<dis_homework1::srv::CustomService_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dis_homework1__srv__CustomService_Response
    std::shared_ptr<dis_homework1::srv::CustomService_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CustomService_Response_ & other) const
  {
    if (this->field3 != other.field3) {
      return false;
    }
    if (this->field4 != other.field4) {
      return false;
    }
    return true;
  }
  bool operator!=(const CustomService_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CustomService_Response_

// alias to use template instance with default allocator
using CustomService_Response =
  dis_homework1::srv::CustomService_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dis_homework1

namespace dis_homework1
{

namespace srv
{

struct CustomService
{
  using Request = dis_homework1::srv::CustomService_Request;
  using Response = dis_homework1::srv::CustomService_Response;
};

}  // namespace srv

}  // namespace dis_homework1

#endif  // DIS_HOMEWORK1__SRV__DETAIL__CUSTOM_SERVICE__STRUCT_HPP_
