// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auria_msgs:msg/PointArray.idl
// generated code does not contain a copyright notice

#ifndef AURIA_MSGS__MSG__DETAIL__POINT_ARRAY__BUILDER_HPP_
#define AURIA_MSGS__MSG__DETAIL__POINT_ARRAY__BUILDER_HPP_

#include "auria_msgs/msg/detail/point_array__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace auria_msgs
{

namespace msg
{

namespace builder
{

class Init_PointArray_points
{
public:
  Init_PointArray_points()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::auria_msgs::msg::PointArray points(::auria_msgs::msg::PointArray::_points_type arg)
  {
    msg_.points = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auria_msgs::msg::PointArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auria_msgs::msg::PointArray>()
{
  return auria_msgs::msg::builder::Init_PointArray_points();
}

}  // namespace auria_msgs

#endif  // AURIA_MSGS__MSG__DETAIL__POINT_ARRAY__BUILDER_HPP_
