// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from choirbot_interfaces:msg/PositionTask.idl
// generated code does not contain a copyright notice
#include "choirbot_interfaces/msg/detail/position_task__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `coordinates`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
choirbot_interfaces__msg__PositionTask__init(choirbot_interfaces__msg__PositionTask * msg)
{
  if (!msg) {
    return false;
  }
  // coordinates
  if (!rosidl_runtime_c__double__Sequence__init(&msg->coordinates, 0)) {
    choirbot_interfaces__msg__PositionTask__fini(msg);
    return false;
  }
  // id
  // seq_num
  return true;
}

void
choirbot_interfaces__msg__PositionTask__fini(choirbot_interfaces__msg__PositionTask * msg)
{
  if (!msg) {
    return;
  }
  // coordinates
  rosidl_runtime_c__double__Sequence__fini(&msg->coordinates);
  // id
  // seq_num
}

bool
choirbot_interfaces__msg__PositionTask__are_equal(const choirbot_interfaces__msg__PositionTask * lhs, const choirbot_interfaces__msg__PositionTask * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // coordinates
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->coordinates), &(rhs->coordinates)))
  {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // seq_num
  if (lhs->seq_num != rhs->seq_num) {
    return false;
  }
  return true;
}

bool
choirbot_interfaces__msg__PositionTask__copy(
  const choirbot_interfaces__msg__PositionTask * input,
  choirbot_interfaces__msg__PositionTask * output)
{
  if (!input || !output) {
    return false;
  }
  // coordinates
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->coordinates), &(output->coordinates)))
  {
    return false;
  }
  // id
  output->id = input->id;
  // seq_num
  output->seq_num = input->seq_num;
  return true;
}

choirbot_interfaces__msg__PositionTask *
choirbot_interfaces__msg__PositionTask__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__msg__PositionTask * msg = (choirbot_interfaces__msg__PositionTask *)allocator.allocate(sizeof(choirbot_interfaces__msg__PositionTask), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(choirbot_interfaces__msg__PositionTask));
  bool success = choirbot_interfaces__msg__PositionTask__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
choirbot_interfaces__msg__PositionTask__destroy(choirbot_interfaces__msg__PositionTask * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    choirbot_interfaces__msg__PositionTask__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
choirbot_interfaces__msg__PositionTask__Sequence__init(choirbot_interfaces__msg__PositionTask__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__msg__PositionTask * data = NULL;

  if (size) {
    data = (choirbot_interfaces__msg__PositionTask *)allocator.zero_allocate(size, sizeof(choirbot_interfaces__msg__PositionTask), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = choirbot_interfaces__msg__PositionTask__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        choirbot_interfaces__msg__PositionTask__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
choirbot_interfaces__msg__PositionTask__Sequence__fini(choirbot_interfaces__msg__PositionTask__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      choirbot_interfaces__msg__PositionTask__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

choirbot_interfaces__msg__PositionTask__Sequence *
choirbot_interfaces__msg__PositionTask__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__msg__PositionTask__Sequence * array = (choirbot_interfaces__msg__PositionTask__Sequence *)allocator.allocate(sizeof(choirbot_interfaces__msg__PositionTask__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = choirbot_interfaces__msg__PositionTask__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
choirbot_interfaces__msg__PositionTask__Sequence__destroy(choirbot_interfaces__msg__PositionTask__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    choirbot_interfaces__msg__PositionTask__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
choirbot_interfaces__msg__PositionTask__Sequence__are_equal(const choirbot_interfaces__msg__PositionTask__Sequence * lhs, const choirbot_interfaces__msg__PositionTask__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!choirbot_interfaces__msg__PositionTask__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
choirbot_interfaces__msg__PositionTask__Sequence__copy(
  const choirbot_interfaces__msg__PositionTask__Sequence * input,
  choirbot_interfaces__msg__PositionTask__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(choirbot_interfaces__msg__PositionTask);
    choirbot_interfaces__msg__PositionTask * data =
      (choirbot_interfaces__msg__PositionTask *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!choirbot_interfaces__msg__PositionTask__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          choirbot_interfaces__msg__PositionTask__fini(&data[i]);
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
    if (!choirbot_interfaces__msg__PositionTask__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
