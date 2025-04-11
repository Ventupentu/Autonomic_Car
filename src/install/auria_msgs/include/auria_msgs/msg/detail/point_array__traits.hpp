// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auria_msgs:msg/PointArray.idl
// generated code does not contain a copyright notice

#ifndef AURIA_MSGS__MSG__DETAIL__POINT_ARRAY__TRAITS_HPP_
#define AURIA_MSGS__MSG__DETAIL__POINT_ARRAY__TRAITS_HPP_

#include "auria_msgs/msg/detail/point_array__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'points'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const auria_msgs::msg::PointArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: points
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.points.size() == 0) {
      out << "points: []\n";
    } else {
      out << "points:\n";
      for (auto item : msg.points) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const auria_msgs::msg::PointArray & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<auria_msgs::msg::PointArray>()
{
  return "auria_msgs::msg::PointArray";
}

template<>
inline const char * name<auria_msgs::msg::PointArray>()
{
  return "auria_msgs/msg/PointArray";
}

template<>
struct has_fixed_size<auria_msgs::msg::PointArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<auria_msgs::msg::PointArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<auria_msgs::msg::PointArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AURIA_MSGS__MSG__DETAIL__POINT_ARRAY__TRAITS_HPP_
