// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from choirbot_interfaces:msg/PositionTaskArray.idl
// generated code does not contain a copyright notice
#include "choirbot_interfaces/msg/detail/position_task_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `tasks`
#include "choirbot_interfaces/msg/detail/position_task__functions.h"

bool
choirbot_interfaces__msg__PositionTaskArray__init(choirbot_interfaces__msg__PositionTaskArray * msg)
{
  if (!msg) {
    return false;
  }
  // tasks
  if (!choirbot_interfaces__msg__PositionTask__Sequence__init(&msg->tasks, 0)) {
    choirbot_interfaces__msg__PositionTaskArray__fini(msg);
    return false;
  }
  // all_tasks_count
  // label
  return true;
}

void
choirbot_interfaces__msg__PositionTaskArray__fini(choirbot_interfaces__msg__PositionTaskArray * msg)
{
  if (!msg) {
    return;
  }
  // tasks
  choirbot_interfaces__msg__PositionTask__Sequence__fini(&msg->tasks);
  // all_tasks_count
  // label
}

bool
choirbot_interfaces__msg__PositionTaskArray__are_equal(const choirbot_interfaces__msg__PositionTaskArray * lhs, const choirbot_interfaces__msg__PositionTaskArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // tasks
  if (!choirbot_interfaces__msg__PositionTask__Sequence__are_equal(
      &(lhs->tasks), &(rhs->tasks)))
  {
    return false;
  }
  // all_tasks_count
  if (lhs->all_tasks_count != rhs->all_tasks_count) {
    return false;
  }
  // label
  if (lhs->label != rhs->label) {
    return false;
  }
  return true;
}

bool
choirbot_interfaces__msg__PositionTaskArray__copy(
  const choirbot_interfaces__msg__PositionTaskArray * input,
  choirbot_interfaces__msg__PositionTaskArray * output)
{
  if (!input || !output) {
    return false;
  }
  // tasks
  if (!choirbot_interfaces__msg__PositionTask__Sequence__copy(
      &(input->tasks), &(output->tasks)))
  {
    return false;
  }
  // all_tasks_count
  output->all_tasks_count = input->all_tasks_count;
  // label
  output->label = input->label;
  return true;
}

choirbot_interfaces__msg__PositionTaskArray *
choirbot_interfaces__msg__PositionTaskArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__msg__PositionTaskArray * msg = (choirbot_interfaces__msg__PositionTaskArray *)allocator.allocate(sizeof(choirbot_interfaces__msg__PositionTaskArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(choirbot_interfaces__msg__PositionTaskArray));
  bool success = choirbot_interfaces__msg__PositionTaskArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
choirbot_interfaces__msg__PositionTaskArray__destroy(choirbot_interfaces__msg__PositionTaskArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    choirbot_interfaces__msg__PositionTaskArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
choirbot_interfaces__msg__PositionTaskArray__Sequence__init(choirbot_interfaces__msg__PositionTaskArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__msg__PositionTaskArray * data = NULL;

  if (size) {
    data = (choirbot_interfaces__msg__PositionTaskArray *)allocator.zero_allocate(size, sizeof(choirbot_interfaces__msg__PositionTaskArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = choirbot_interfaces__msg__PositionTaskArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        choirbot_interfaces__msg__PositionTaskArray__fini(&data[i - 1]);
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
choirbot_interfaces__msg__PositionTaskArray__Sequence__fini(choirbot_interfaces__msg__PositionTaskArray__Sequence * array)
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
      choirbot_interfaces__msg__PositionTaskArray__fini(&array->data[i]);
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

choirbot_interfaces__msg__PositionTaskArray__Sequence *
choirbot_interfaces__msg__PositionTaskArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__msg__PositionTaskArray__Sequence * array = (choirbot_interfaces__msg__PositionTaskArray__Sequence *)allocator.allocate(sizeof(choirbot_interfaces__msg__PositionTaskArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = choirbot_interfaces__msg__PositionTaskArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
choirbot_interfaces__msg__PositionTaskArray__Sequence__destroy(choirbot_interfaces__msg__PositionTaskArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    choirbot_interfaces__msg__PositionTaskArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
choirbot_interfaces__msg__PositionTaskArray__Sequence__are_equal(const choirbot_interfaces__msg__PositionTaskArray__Sequence * lhs, const choirbot_interfaces__msg__PositionTaskArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!choirbot_interfaces__msg__PositionTaskArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
choirbot_interfaces__msg__PositionTaskArray__Sequence__copy(
  const choirbot_interfaces__msg__PositionTaskArray__Sequence * input,
  choirbot_interfaces__msg__PositionTaskArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(choirbot_interfaces__msg__PositionTaskArray);
    choirbot_interfaces__msg__PositionTaskArray * data =
      (choirbot_interfaces__msg__PositionTaskArray *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!choirbot_interfaces__msg__PositionTaskArray__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          choirbot_interfaces__msg__PositionTaskArray__fini(&data[i]);
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
    if (!choirbot_interfaces__msg__PositionTaskArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
