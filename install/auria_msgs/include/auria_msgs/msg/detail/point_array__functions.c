// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from auria_msgs:msg/PointArray.idl
// generated code does not contain a copyright notice
#include "auria_msgs/msg/detail/point_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `points`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
auria_msgs__msg__PointArray__init(auria_msgs__msg__PointArray * msg)
{
  if (!msg) {
    return false;
  }
  // points
  if (!geometry_msgs__msg__Point__Sequence__init(&msg->points, 0)) {
    auria_msgs__msg__PointArray__fini(msg);
    return false;
  }
  return true;
}

void
auria_msgs__msg__PointArray__fini(auria_msgs__msg__PointArray * msg)
{
  if (!msg) {
    return;
  }
  // points
  geometry_msgs__msg__Point__Sequence__fini(&msg->points);
}

bool
auria_msgs__msg__PointArray__are_equal(const auria_msgs__msg__PointArray * lhs, const auria_msgs__msg__PointArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // points
  if (!geometry_msgs__msg__Point__Sequence__are_equal(
      &(lhs->points), &(rhs->points)))
  {
    return false;
  }
  return true;
}

bool
auria_msgs__msg__PointArray__copy(
  const auria_msgs__msg__PointArray * input,
  auria_msgs__msg__PointArray * output)
{
  if (!input || !output) {
    return false;
  }
  // points
  if (!geometry_msgs__msg__Point__Sequence__copy(
      &(input->points), &(output->points)))
  {
    return false;
  }
  return true;
}

auria_msgs__msg__PointArray *
auria_msgs__msg__PointArray__create()
{
  auria_msgs__msg__PointArray * msg = (auria_msgs__msg__PointArray *)malloc(sizeof(auria_msgs__msg__PointArray));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(auria_msgs__msg__PointArray));
  bool success = auria_msgs__msg__PointArray__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
auria_msgs__msg__PointArray__destroy(auria_msgs__msg__PointArray * msg)
{
  if (msg) {
    auria_msgs__msg__PointArray__fini(msg);
  }
  free(msg);
}


bool
auria_msgs__msg__PointArray__Sequence__init(auria_msgs__msg__PointArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  auria_msgs__msg__PointArray * data = NULL;
  if (size) {
    data = (auria_msgs__msg__PointArray *)calloc(size, sizeof(auria_msgs__msg__PointArray));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = auria_msgs__msg__PointArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        auria_msgs__msg__PointArray__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
auria_msgs__msg__PointArray__Sequence__fini(auria_msgs__msg__PointArray__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      auria_msgs__msg__PointArray__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

auria_msgs__msg__PointArray__Sequence *
auria_msgs__msg__PointArray__Sequence__create(size_t size)
{
  auria_msgs__msg__PointArray__Sequence * array = (auria_msgs__msg__PointArray__Sequence *)malloc(sizeof(auria_msgs__msg__PointArray__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = auria_msgs__msg__PointArray__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
auria_msgs__msg__PointArray__Sequence__destroy(auria_msgs__msg__PointArray__Sequence * array)
{
  if (array) {
    auria_msgs__msg__PointArray__Sequence__fini(array);
  }
  free(array);
}

bool
auria_msgs__msg__PointArray__Sequence__are_equal(const auria_msgs__msg__PointArray__Sequence * lhs, const auria_msgs__msg__PointArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!auria_msgs__msg__PointArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
auria_msgs__msg__PointArray__Sequence__copy(
  const auria_msgs__msg__PointArray__Sequence * input,
  auria_msgs__msg__PointArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(auria_msgs__msg__PointArray);
    auria_msgs__msg__PointArray * data =
      (auria_msgs__msg__PointArray *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!auria_msgs__msg__PointArray__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          auria_msgs__msg__PointArray__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!auria_msgs__msg__PointArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
