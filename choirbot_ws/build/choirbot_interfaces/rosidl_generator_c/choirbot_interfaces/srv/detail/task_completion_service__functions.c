// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from choirbot_interfaces:srv/TaskCompletionService.idl
// generated code does not contain a copyright notice
#include "choirbot_interfaces/srv/detail/task_completion_service__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
choirbot_interfaces__srv__TaskCompletionService_Request__init(choirbot_interfaces__srv__TaskCompletionService_Request * msg)
{
  if (!msg) {
    return false;
  }
  // agent_id
  // task_seq_num
  return true;
}

void
choirbot_interfaces__srv__TaskCompletionService_Request__fini(choirbot_interfaces__srv__TaskCompletionService_Request * msg)
{
  if (!msg) {
    return;
  }
  // agent_id
  // task_seq_num
}

bool
choirbot_interfaces__srv__TaskCompletionService_Request__are_equal(const choirbot_interfaces__srv__TaskCompletionService_Request * lhs, const choirbot_interfaces__srv__TaskCompletionService_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // agent_id
  if (lhs->agent_id != rhs->agent_id) {
    return false;
  }
  // task_seq_num
  if (lhs->task_seq_num != rhs->task_seq_num) {
    return false;
  }
  return true;
}

bool
choirbot_interfaces__srv__TaskCompletionService_Request__copy(
  const choirbot_interfaces__srv__TaskCompletionService_Request * input,
  choirbot_interfaces__srv__TaskCompletionService_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // agent_id
  output->agent_id = input->agent_id;
  // task_seq_num
  output->task_seq_num = input->task_seq_num;
  return true;
}

choirbot_interfaces__srv__TaskCompletionService_Request *
choirbot_interfaces__srv__TaskCompletionService_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__srv__TaskCompletionService_Request * msg = (choirbot_interfaces__srv__TaskCompletionService_Request *)allocator.allocate(sizeof(choirbot_interfaces__srv__TaskCompletionService_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(choirbot_interfaces__srv__TaskCompletionService_Request));
  bool success = choirbot_interfaces__srv__TaskCompletionService_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
choirbot_interfaces__srv__TaskCompletionService_Request__destroy(choirbot_interfaces__srv__TaskCompletionService_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    choirbot_interfaces__srv__TaskCompletionService_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
choirbot_interfaces__srv__TaskCompletionService_Request__Sequence__init(choirbot_interfaces__srv__TaskCompletionService_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__srv__TaskCompletionService_Request * data = NULL;

  if (size) {
    data = (choirbot_interfaces__srv__TaskCompletionService_Request *)allocator.zero_allocate(size, sizeof(choirbot_interfaces__srv__TaskCompletionService_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = choirbot_interfaces__srv__TaskCompletionService_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        choirbot_interfaces__srv__TaskCompletionService_Request__fini(&data[i - 1]);
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
choirbot_interfaces__srv__TaskCompletionService_Request__Sequence__fini(choirbot_interfaces__srv__TaskCompletionService_Request__Sequence * array)
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
      choirbot_interfaces__srv__TaskCompletionService_Request__fini(&array->data[i]);
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

choirbot_interfaces__srv__TaskCompletionService_Request__Sequence *
choirbot_interfaces__srv__TaskCompletionService_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__srv__TaskCompletionService_Request__Sequence * array = (choirbot_interfaces__srv__TaskCompletionService_Request__Sequence *)allocator.allocate(sizeof(choirbot_interfaces__srv__TaskCompletionService_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = choirbot_interfaces__srv__TaskCompletionService_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
choirbot_interfaces__srv__TaskCompletionService_Request__Sequence__destroy(choirbot_interfaces__srv__TaskCompletionService_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    choirbot_interfaces__srv__TaskCompletionService_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
choirbot_interfaces__srv__TaskCompletionService_Request__Sequence__are_equal(const choirbot_interfaces__srv__TaskCompletionService_Request__Sequence * lhs, const choirbot_interfaces__srv__TaskCompletionService_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!choirbot_interfaces__srv__TaskCompletionService_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
choirbot_interfaces__srv__TaskCompletionService_Request__Sequence__copy(
  const choirbot_interfaces__srv__TaskCompletionService_Request__Sequence * input,
  choirbot_interfaces__srv__TaskCompletionService_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(choirbot_interfaces__srv__TaskCompletionService_Request);
    choirbot_interfaces__srv__TaskCompletionService_Request * data =
      (choirbot_interfaces__srv__TaskCompletionService_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!choirbot_interfaces__srv__TaskCompletionService_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          choirbot_interfaces__srv__TaskCompletionService_Request__fini(&data[i]);
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
    if (!choirbot_interfaces__srv__TaskCompletionService_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
choirbot_interfaces__srv__TaskCompletionService_Response__init(choirbot_interfaces__srv__TaskCompletionService_Response * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
choirbot_interfaces__srv__TaskCompletionService_Response__fini(choirbot_interfaces__srv__TaskCompletionService_Response * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
choirbot_interfaces__srv__TaskCompletionService_Response__are_equal(const choirbot_interfaces__srv__TaskCompletionService_Response * lhs, const choirbot_interfaces__srv__TaskCompletionService_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
choirbot_interfaces__srv__TaskCompletionService_Response__copy(
  const choirbot_interfaces__srv__TaskCompletionService_Response * input,
  choirbot_interfaces__srv__TaskCompletionService_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

choirbot_interfaces__srv__TaskCompletionService_Response *
choirbot_interfaces__srv__TaskCompletionService_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__srv__TaskCompletionService_Response * msg = (choirbot_interfaces__srv__TaskCompletionService_Response *)allocator.allocate(sizeof(choirbot_interfaces__srv__TaskCompletionService_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(choirbot_interfaces__srv__TaskCompletionService_Response));
  bool success = choirbot_interfaces__srv__TaskCompletionService_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
choirbot_interfaces__srv__TaskCompletionService_Response__destroy(choirbot_interfaces__srv__TaskCompletionService_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    choirbot_interfaces__srv__TaskCompletionService_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
choirbot_interfaces__srv__TaskCompletionService_Response__Sequence__init(choirbot_interfaces__srv__TaskCompletionService_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__srv__TaskCompletionService_Response * data = NULL;

  if (size) {
    data = (choirbot_interfaces__srv__TaskCompletionService_Response *)allocator.zero_allocate(size, sizeof(choirbot_interfaces__srv__TaskCompletionService_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = choirbot_interfaces__srv__TaskCompletionService_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        choirbot_interfaces__srv__TaskCompletionService_Response__fini(&data[i - 1]);
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
choirbot_interfaces__srv__TaskCompletionService_Response__Sequence__fini(choirbot_interfaces__srv__TaskCompletionService_Response__Sequence * array)
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
      choirbot_interfaces__srv__TaskCompletionService_Response__fini(&array->data[i]);
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

choirbot_interfaces__srv__TaskCompletionService_Response__Sequence *
choirbot_interfaces__srv__TaskCompletionService_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__srv__TaskCompletionService_Response__Sequence * array = (choirbot_interfaces__srv__TaskCompletionService_Response__Sequence *)allocator.allocate(sizeof(choirbot_interfaces__srv__TaskCompletionService_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = choirbot_interfaces__srv__TaskCompletionService_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
choirbot_interfaces__srv__TaskCompletionService_Response__Sequence__destroy(choirbot_interfaces__srv__TaskCompletionService_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    choirbot_interfaces__srv__TaskCompletionService_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
choirbot_interfaces__srv__TaskCompletionService_Response__Sequence__are_equal(const choirbot_interfaces__srv__TaskCompletionService_Response__Sequence * lhs, const choirbot_interfaces__srv__TaskCompletionService_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!choirbot_interfaces__srv__TaskCompletionService_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
choirbot_interfaces__srv__TaskCompletionService_Response__Sequence__copy(
  const choirbot_interfaces__srv__TaskCompletionService_Response__Sequence * input,
  choirbot_interfaces__srv__TaskCompletionService_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(choirbot_interfaces__srv__TaskCompletionService_Response);
    choirbot_interfaces__srv__TaskCompletionService_Response * data =
      (choirbot_interfaces__srv__TaskCompletionService_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!choirbot_interfaces__srv__TaskCompletionService_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          choirbot_interfaces__srv__TaskCompletionService_Response__fini(&data[i]);
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
    if (!choirbot_interfaces__srv__TaskCompletionService_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
