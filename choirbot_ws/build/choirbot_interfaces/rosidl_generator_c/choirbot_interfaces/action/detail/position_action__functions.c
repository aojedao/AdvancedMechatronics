// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from choirbot_interfaces:action/PositionAction.idl
// generated code does not contain a copyright notice
#include "choirbot_interfaces/action/detail/position_action__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `goal_position`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
choirbot_interfaces__action__PositionAction_Goal__init(choirbot_interfaces__action__PositionAction_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // goal_position
  if (!rosidl_runtime_c__double__Sequence__init(&msg->goal_position, 0)) {
    choirbot_interfaces__action__PositionAction_Goal__fini(msg);
    return false;
  }
  return true;
}

void
choirbot_interfaces__action__PositionAction_Goal__fini(choirbot_interfaces__action__PositionAction_Goal * msg)
{
  if (!msg) {
    return;
  }
  // goal_position
  rosidl_runtime_c__double__Sequence__fini(&msg->goal_position);
}

bool
choirbot_interfaces__action__PositionAction_Goal__are_equal(const choirbot_interfaces__action__PositionAction_Goal * lhs, const choirbot_interfaces__action__PositionAction_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_position
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->goal_position), &(rhs->goal_position)))
  {
    return false;
  }
  return true;
}

bool
choirbot_interfaces__action__PositionAction_Goal__copy(
  const choirbot_interfaces__action__PositionAction_Goal * input,
  choirbot_interfaces__action__PositionAction_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_position
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->goal_position), &(output->goal_position)))
  {
    return false;
  }
  return true;
}

choirbot_interfaces__action__PositionAction_Goal *
choirbot_interfaces__action__PositionAction_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_Goal * msg = (choirbot_interfaces__action__PositionAction_Goal *)allocator.allocate(sizeof(choirbot_interfaces__action__PositionAction_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(choirbot_interfaces__action__PositionAction_Goal));
  bool success = choirbot_interfaces__action__PositionAction_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
choirbot_interfaces__action__PositionAction_Goal__destroy(choirbot_interfaces__action__PositionAction_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    choirbot_interfaces__action__PositionAction_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
choirbot_interfaces__action__PositionAction_Goal__Sequence__init(choirbot_interfaces__action__PositionAction_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_Goal * data = NULL;

  if (size) {
    data = (choirbot_interfaces__action__PositionAction_Goal *)allocator.zero_allocate(size, sizeof(choirbot_interfaces__action__PositionAction_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = choirbot_interfaces__action__PositionAction_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        choirbot_interfaces__action__PositionAction_Goal__fini(&data[i - 1]);
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
choirbot_interfaces__action__PositionAction_Goal__Sequence__fini(choirbot_interfaces__action__PositionAction_Goal__Sequence * array)
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
      choirbot_interfaces__action__PositionAction_Goal__fini(&array->data[i]);
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

choirbot_interfaces__action__PositionAction_Goal__Sequence *
choirbot_interfaces__action__PositionAction_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_Goal__Sequence * array = (choirbot_interfaces__action__PositionAction_Goal__Sequence *)allocator.allocate(sizeof(choirbot_interfaces__action__PositionAction_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = choirbot_interfaces__action__PositionAction_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
choirbot_interfaces__action__PositionAction_Goal__Sequence__destroy(choirbot_interfaces__action__PositionAction_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    choirbot_interfaces__action__PositionAction_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
choirbot_interfaces__action__PositionAction_Goal__Sequence__are_equal(const choirbot_interfaces__action__PositionAction_Goal__Sequence * lhs, const choirbot_interfaces__action__PositionAction_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!choirbot_interfaces__action__PositionAction_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
choirbot_interfaces__action__PositionAction_Goal__Sequence__copy(
  const choirbot_interfaces__action__PositionAction_Goal__Sequence * input,
  choirbot_interfaces__action__PositionAction_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(choirbot_interfaces__action__PositionAction_Goal);
    choirbot_interfaces__action__PositionAction_Goal * data =
      (choirbot_interfaces__action__PositionAction_Goal *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!choirbot_interfaces__action__PositionAction_Goal__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          choirbot_interfaces__action__PositionAction_Goal__fini(&data[i]);
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
    if (!choirbot_interfaces__action__PositionAction_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `final_position`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
choirbot_interfaces__action__PositionAction_Result__init(choirbot_interfaces__action__PositionAction_Result * msg)
{
  if (!msg) {
    return false;
  }
  // final_position
  if (!rosidl_runtime_c__double__Sequence__init(&msg->final_position, 0)) {
    choirbot_interfaces__action__PositionAction_Result__fini(msg);
    return false;
  }
  return true;
}

void
choirbot_interfaces__action__PositionAction_Result__fini(choirbot_interfaces__action__PositionAction_Result * msg)
{
  if (!msg) {
    return;
  }
  // final_position
  rosidl_runtime_c__double__Sequence__fini(&msg->final_position);
}

bool
choirbot_interfaces__action__PositionAction_Result__are_equal(const choirbot_interfaces__action__PositionAction_Result * lhs, const choirbot_interfaces__action__PositionAction_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // final_position
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->final_position), &(rhs->final_position)))
  {
    return false;
  }
  return true;
}

bool
choirbot_interfaces__action__PositionAction_Result__copy(
  const choirbot_interfaces__action__PositionAction_Result * input,
  choirbot_interfaces__action__PositionAction_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // final_position
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->final_position), &(output->final_position)))
  {
    return false;
  }
  return true;
}

choirbot_interfaces__action__PositionAction_Result *
choirbot_interfaces__action__PositionAction_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_Result * msg = (choirbot_interfaces__action__PositionAction_Result *)allocator.allocate(sizeof(choirbot_interfaces__action__PositionAction_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(choirbot_interfaces__action__PositionAction_Result));
  bool success = choirbot_interfaces__action__PositionAction_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
choirbot_interfaces__action__PositionAction_Result__destroy(choirbot_interfaces__action__PositionAction_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    choirbot_interfaces__action__PositionAction_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
choirbot_interfaces__action__PositionAction_Result__Sequence__init(choirbot_interfaces__action__PositionAction_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_Result * data = NULL;

  if (size) {
    data = (choirbot_interfaces__action__PositionAction_Result *)allocator.zero_allocate(size, sizeof(choirbot_interfaces__action__PositionAction_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = choirbot_interfaces__action__PositionAction_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        choirbot_interfaces__action__PositionAction_Result__fini(&data[i - 1]);
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
choirbot_interfaces__action__PositionAction_Result__Sequence__fini(choirbot_interfaces__action__PositionAction_Result__Sequence * array)
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
      choirbot_interfaces__action__PositionAction_Result__fini(&array->data[i]);
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

choirbot_interfaces__action__PositionAction_Result__Sequence *
choirbot_interfaces__action__PositionAction_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_Result__Sequence * array = (choirbot_interfaces__action__PositionAction_Result__Sequence *)allocator.allocate(sizeof(choirbot_interfaces__action__PositionAction_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = choirbot_interfaces__action__PositionAction_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
choirbot_interfaces__action__PositionAction_Result__Sequence__destroy(choirbot_interfaces__action__PositionAction_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    choirbot_interfaces__action__PositionAction_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
choirbot_interfaces__action__PositionAction_Result__Sequence__are_equal(const choirbot_interfaces__action__PositionAction_Result__Sequence * lhs, const choirbot_interfaces__action__PositionAction_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!choirbot_interfaces__action__PositionAction_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
choirbot_interfaces__action__PositionAction_Result__Sequence__copy(
  const choirbot_interfaces__action__PositionAction_Result__Sequence * input,
  choirbot_interfaces__action__PositionAction_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(choirbot_interfaces__action__PositionAction_Result);
    choirbot_interfaces__action__PositionAction_Result * data =
      (choirbot_interfaces__action__PositionAction_Result *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!choirbot_interfaces__action__PositionAction_Result__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          choirbot_interfaces__action__PositionAction_Result__fini(&data[i]);
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
    if (!choirbot_interfaces__action__PositionAction_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `current_position`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
choirbot_interfaces__action__PositionAction_Feedback__init(choirbot_interfaces__action__PositionAction_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // current_position
  if (!rosidl_runtime_c__double__Sequence__init(&msg->current_position, 0)) {
    choirbot_interfaces__action__PositionAction_Feedback__fini(msg);
    return false;
  }
  // distance
  return true;
}

void
choirbot_interfaces__action__PositionAction_Feedback__fini(choirbot_interfaces__action__PositionAction_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // current_position
  rosidl_runtime_c__double__Sequence__fini(&msg->current_position);
  // distance
}

bool
choirbot_interfaces__action__PositionAction_Feedback__are_equal(const choirbot_interfaces__action__PositionAction_Feedback * lhs, const choirbot_interfaces__action__PositionAction_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // current_position
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->current_position), &(rhs->current_position)))
  {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  return true;
}

bool
choirbot_interfaces__action__PositionAction_Feedback__copy(
  const choirbot_interfaces__action__PositionAction_Feedback * input,
  choirbot_interfaces__action__PositionAction_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // current_position
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->current_position), &(output->current_position)))
  {
    return false;
  }
  // distance
  output->distance = input->distance;
  return true;
}

choirbot_interfaces__action__PositionAction_Feedback *
choirbot_interfaces__action__PositionAction_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_Feedback * msg = (choirbot_interfaces__action__PositionAction_Feedback *)allocator.allocate(sizeof(choirbot_interfaces__action__PositionAction_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(choirbot_interfaces__action__PositionAction_Feedback));
  bool success = choirbot_interfaces__action__PositionAction_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
choirbot_interfaces__action__PositionAction_Feedback__destroy(choirbot_interfaces__action__PositionAction_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    choirbot_interfaces__action__PositionAction_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
choirbot_interfaces__action__PositionAction_Feedback__Sequence__init(choirbot_interfaces__action__PositionAction_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_Feedback * data = NULL;

  if (size) {
    data = (choirbot_interfaces__action__PositionAction_Feedback *)allocator.zero_allocate(size, sizeof(choirbot_interfaces__action__PositionAction_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = choirbot_interfaces__action__PositionAction_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        choirbot_interfaces__action__PositionAction_Feedback__fini(&data[i - 1]);
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
choirbot_interfaces__action__PositionAction_Feedback__Sequence__fini(choirbot_interfaces__action__PositionAction_Feedback__Sequence * array)
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
      choirbot_interfaces__action__PositionAction_Feedback__fini(&array->data[i]);
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

choirbot_interfaces__action__PositionAction_Feedback__Sequence *
choirbot_interfaces__action__PositionAction_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_Feedback__Sequence * array = (choirbot_interfaces__action__PositionAction_Feedback__Sequence *)allocator.allocate(sizeof(choirbot_interfaces__action__PositionAction_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = choirbot_interfaces__action__PositionAction_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
choirbot_interfaces__action__PositionAction_Feedback__Sequence__destroy(choirbot_interfaces__action__PositionAction_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    choirbot_interfaces__action__PositionAction_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
choirbot_interfaces__action__PositionAction_Feedback__Sequence__are_equal(const choirbot_interfaces__action__PositionAction_Feedback__Sequence * lhs, const choirbot_interfaces__action__PositionAction_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!choirbot_interfaces__action__PositionAction_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
choirbot_interfaces__action__PositionAction_Feedback__Sequence__copy(
  const choirbot_interfaces__action__PositionAction_Feedback__Sequence * input,
  choirbot_interfaces__action__PositionAction_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(choirbot_interfaces__action__PositionAction_Feedback);
    choirbot_interfaces__action__PositionAction_Feedback * data =
      (choirbot_interfaces__action__PositionAction_Feedback *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!choirbot_interfaces__action__PositionAction_Feedback__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          choirbot_interfaces__action__PositionAction_Feedback__fini(&data[i]);
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
    if (!choirbot_interfaces__action__PositionAction_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "choirbot_interfaces/action/detail/position_action__functions.h"

bool
choirbot_interfaces__action__PositionAction_SendGoal_Request__init(choirbot_interfaces__action__PositionAction_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    choirbot_interfaces__action__PositionAction_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!choirbot_interfaces__action__PositionAction_Goal__init(&msg->goal)) {
    choirbot_interfaces__action__PositionAction_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
choirbot_interfaces__action__PositionAction_SendGoal_Request__fini(choirbot_interfaces__action__PositionAction_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  choirbot_interfaces__action__PositionAction_Goal__fini(&msg->goal);
}

bool
choirbot_interfaces__action__PositionAction_SendGoal_Request__are_equal(const choirbot_interfaces__action__PositionAction_SendGoal_Request * lhs, const choirbot_interfaces__action__PositionAction_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!choirbot_interfaces__action__PositionAction_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
choirbot_interfaces__action__PositionAction_SendGoal_Request__copy(
  const choirbot_interfaces__action__PositionAction_SendGoal_Request * input,
  choirbot_interfaces__action__PositionAction_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!choirbot_interfaces__action__PositionAction_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

choirbot_interfaces__action__PositionAction_SendGoal_Request *
choirbot_interfaces__action__PositionAction_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_SendGoal_Request * msg = (choirbot_interfaces__action__PositionAction_SendGoal_Request *)allocator.allocate(sizeof(choirbot_interfaces__action__PositionAction_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(choirbot_interfaces__action__PositionAction_SendGoal_Request));
  bool success = choirbot_interfaces__action__PositionAction_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
choirbot_interfaces__action__PositionAction_SendGoal_Request__destroy(choirbot_interfaces__action__PositionAction_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    choirbot_interfaces__action__PositionAction_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
choirbot_interfaces__action__PositionAction_SendGoal_Request__Sequence__init(choirbot_interfaces__action__PositionAction_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_SendGoal_Request * data = NULL;

  if (size) {
    data = (choirbot_interfaces__action__PositionAction_SendGoal_Request *)allocator.zero_allocate(size, sizeof(choirbot_interfaces__action__PositionAction_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = choirbot_interfaces__action__PositionAction_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        choirbot_interfaces__action__PositionAction_SendGoal_Request__fini(&data[i - 1]);
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
choirbot_interfaces__action__PositionAction_SendGoal_Request__Sequence__fini(choirbot_interfaces__action__PositionAction_SendGoal_Request__Sequence * array)
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
      choirbot_interfaces__action__PositionAction_SendGoal_Request__fini(&array->data[i]);
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

choirbot_interfaces__action__PositionAction_SendGoal_Request__Sequence *
choirbot_interfaces__action__PositionAction_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_SendGoal_Request__Sequence * array = (choirbot_interfaces__action__PositionAction_SendGoal_Request__Sequence *)allocator.allocate(sizeof(choirbot_interfaces__action__PositionAction_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = choirbot_interfaces__action__PositionAction_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
choirbot_interfaces__action__PositionAction_SendGoal_Request__Sequence__destroy(choirbot_interfaces__action__PositionAction_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    choirbot_interfaces__action__PositionAction_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
choirbot_interfaces__action__PositionAction_SendGoal_Request__Sequence__are_equal(const choirbot_interfaces__action__PositionAction_SendGoal_Request__Sequence * lhs, const choirbot_interfaces__action__PositionAction_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!choirbot_interfaces__action__PositionAction_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
choirbot_interfaces__action__PositionAction_SendGoal_Request__Sequence__copy(
  const choirbot_interfaces__action__PositionAction_SendGoal_Request__Sequence * input,
  choirbot_interfaces__action__PositionAction_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(choirbot_interfaces__action__PositionAction_SendGoal_Request);
    choirbot_interfaces__action__PositionAction_SendGoal_Request * data =
      (choirbot_interfaces__action__PositionAction_SendGoal_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!choirbot_interfaces__action__PositionAction_SendGoal_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          choirbot_interfaces__action__PositionAction_SendGoal_Request__fini(&data[i]);
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
    if (!choirbot_interfaces__action__PositionAction_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
choirbot_interfaces__action__PositionAction_SendGoal_Response__init(choirbot_interfaces__action__PositionAction_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    choirbot_interfaces__action__PositionAction_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
choirbot_interfaces__action__PositionAction_SendGoal_Response__fini(choirbot_interfaces__action__PositionAction_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
choirbot_interfaces__action__PositionAction_SendGoal_Response__are_equal(const choirbot_interfaces__action__PositionAction_SendGoal_Response * lhs, const choirbot_interfaces__action__PositionAction_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
choirbot_interfaces__action__PositionAction_SendGoal_Response__copy(
  const choirbot_interfaces__action__PositionAction_SendGoal_Response * input,
  choirbot_interfaces__action__PositionAction_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

choirbot_interfaces__action__PositionAction_SendGoal_Response *
choirbot_interfaces__action__PositionAction_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_SendGoal_Response * msg = (choirbot_interfaces__action__PositionAction_SendGoal_Response *)allocator.allocate(sizeof(choirbot_interfaces__action__PositionAction_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(choirbot_interfaces__action__PositionAction_SendGoal_Response));
  bool success = choirbot_interfaces__action__PositionAction_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
choirbot_interfaces__action__PositionAction_SendGoal_Response__destroy(choirbot_interfaces__action__PositionAction_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    choirbot_interfaces__action__PositionAction_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
choirbot_interfaces__action__PositionAction_SendGoal_Response__Sequence__init(choirbot_interfaces__action__PositionAction_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_SendGoal_Response * data = NULL;

  if (size) {
    data = (choirbot_interfaces__action__PositionAction_SendGoal_Response *)allocator.zero_allocate(size, sizeof(choirbot_interfaces__action__PositionAction_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = choirbot_interfaces__action__PositionAction_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        choirbot_interfaces__action__PositionAction_SendGoal_Response__fini(&data[i - 1]);
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
choirbot_interfaces__action__PositionAction_SendGoal_Response__Sequence__fini(choirbot_interfaces__action__PositionAction_SendGoal_Response__Sequence * array)
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
      choirbot_interfaces__action__PositionAction_SendGoal_Response__fini(&array->data[i]);
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

choirbot_interfaces__action__PositionAction_SendGoal_Response__Sequence *
choirbot_interfaces__action__PositionAction_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_SendGoal_Response__Sequence * array = (choirbot_interfaces__action__PositionAction_SendGoal_Response__Sequence *)allocator.allocate(sizeof(choirbot_interfaces__action__PositionAction_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = choirbot_interfaces__action__PositionAction_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
choirbot_interfaces__action__PositionAction_SendGoal_Response__Sequence__destroy(choirbot_interfaces__action__PositionAction_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    choirbot_interfaces__action__PositionAction_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
choirbot_interfaces__action__PositionAction_SendGoal_Response__Sequence__are_equal(const choirbot_interfaces__action__PositionAction_SendGoal_Response__Sequence * lhs, const choirbot_interfaces__action__PositionAction_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!choirbot_interfaces__action__PositionAction_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
choirbot_interfaces__action__PositionAction_SendGoal_Response__Sequence__copy(
  const choirbot_interfaces__action__PositionAction_SendGoal_Response__Sequence * input,
  choirbot_interfaces__action__PositionAction_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(choirbot_interfaces__action__PositionAction_SendGoal_Response);
    choirbot_interfaces__action__PositionAction_SendGoal_Response * data =
      (choirbot_interfaces__action__PositionAction_SendGoal_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!choirbot_interfaces__action__PositionAction_SendGoal_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          choirbot_interfaces__action__PositionAction_SendGoal_Response__fini(&data[i]);
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
    if (!choirbot_interfaces__action__PositionAction_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
choirbot_interfaces__action__PositionAction_GetResult_Request__init(choirbot_interfaces__action__PositionAction_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    choirbot_interfaces__action__PositionAction_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
choirbot_interfaces__action__PositionAction_GetResult_Request__fini(choirbot_interfaces__action__PositionAction_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
choirbot_interfaces__action__PositionAction_GetResult_Request__are_equal(const choirbot_interfaces__action__PositionAction_GetResult_Request * lhs, const choirbot_interfaces__action__PositionAction_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
choirbot_interfaces__action__PositionAction_GetResult_Request__copy(
  const choirbot_interfaces__action__PositionAction_GetResult_Request * input,
  choirbot_interfaces__action__PositionAction_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

choirbot_interfaces__action__PositionAction_GetResult_Request *
choirbot_interfaces__action__PositionAction_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_GetResult_Request * msg = (choirbot_interfaces__action__PositionAction_GetResult_Request *)allocator.allocate(sizeof(choirbot_interfaces__action__PositionAction_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(choirbot_interfaces__action__PositionAction_GetResult_Request));
  bool success = choirbot_interfaces__action__PositionAction_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
choirbot_interfaces__action__PositionAction_GetResult_Request__destroy(choirbot_interfaces__action__PositionAction_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    choirbot_interfaces__action__PositionAction_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
choirbot_interfaces__action__PositionAction_GetResult_Request__Sequence__init(choirbot_interfaces__action__PositionAction_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_GetResult_Request * data = NULL;

  if (size) {
    data = (choirbot_interfaces__action__PositionAction_GetResult_Request *)allocator.zero_allocate(size, sizeof(choirbot_interfaces__action__PositionAction_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = choirbot_interfaces__action__PositionAction_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        choirbot_interfaces__action__PositionAction_GetResult_Request__fini(&data[i - 1]);
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
choirbot_interfaces__action__PositionAction_GetResult_Request__Sequence__fini(choirbot_interfaces__action__PositionAction_GetResult_Request__Sequence * array)
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
      choirbot_interfaces__action__PositionAction_GetResult_Request__fini(&array->data[i]);
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

choirbot_interfaces__action__PositionAction_GetResult_Request__Sequence *
choirbot_interfaces__action__PositionAction_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_GetResult_Request__Sequence * array = (choirbot_interfaces__action__PositionAction_GetResult_Request__Sequence *)allocator.allocate(sizeof(choirbot_interfaces__action__PositionAction_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = choirbot_interfaces__action__PositionAction_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
choirbot_interfaces__action__PositionAction_GetResult_Request__Sequence__destroy(choirbot_interfaces__action__PositionAction_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    choirbot_interfaces__action__PositionAction_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
choirbot_interfaces__action__PositionAction_GetResult_Request__Sequence__are_equal(const choirbot_interfaces__action__PositionAction_GetResult_Request__Sequence * lhs, const choirbot_interfaces__action__PositionAction_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!choirbot_interfaces__action__PositionAction_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
choirbot_interfaces__action__PositionAction_GetResult_Request__Sequence__copy(
  const choirbot_interfaces__action__PositionAction_GetResult_Request__Sequence * input,
  choirbot_interfaces__action__PositionAction_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(choirbot_interfaces__action__PositionAction_GetResult_Request);
    choirbot_interfaces__action__PositionAction_GetResult_Request * data =
      (choirbot_interfaces__action__PositionAction_GetResult_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!choirbot_interfaces__action__PositionAction_GetResult_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          choirbot_interfaces__action__PositionAction_GetResult_Request__fini(&data[i]);
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
    if (!choirbot_interfaces__action__PositionAction_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "choirbot_interfaces/action/detail/position_action__functions.h"

bool
choirbot_interfaces__action__PositionAction_GetResult_Response__init(choirbot_interfaces__action__PositionAction_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!choirbot_interfaces__action__PositionAction_Result__init(&msg->result)) {
    choirbot_interfaces__action__PositionAction_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
choirbot_interfaces__action__PositionAction_GetResult_Response__fini(choirbot_interfaces__action__PositionAction_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  choirbot_interfaces__action__PositionAction_Result__fini(&msg->result);
}

bool
choirbot_interfaces__action__PositionAction_GetResult_Response__are_equal(const choirbot_interfaces__action__PositionAction_GetResult_Response * lhs, const choirbot_interfaces__action__PositionAction_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!choirbot_interfaces__action__PositionAction_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
choirbot_interfaces__action__PositionAction_GetResult_Response__copy(
  const choirbot_interfaces__action__PositionAction_GetResult_Response * input,
  choirbot_interfaces__action__PositionAction_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!choirbot_interfaces__action__PositionAction_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

choirbot_interfaces__action__PositionAction_GetResult_Response *
choirbot_interfaces__action__PositionAction_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_GetResult_Response * msg = (choirbot_interfaces__action__PositionAction_GetResult_Response *)allocator.allocate(sizeof(choirbot_interfaces__action__PositionAction_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(choirbot_interfaces__action__PositionAction_GetResult_Response));
  bool success = choirbot_interfaces__action__PositionAction_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
choirbot_interfaces__action__PositionAction_GetResult_Response__destroy(choirbot_interfaces__action__PositionAction_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    choirbot_interfaces__action__PositionAction_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
choirbot_interfaces__action__PositionAction_GetResult_Response__Sequence__init(choirbot_interfaces__action__PositionAction_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_GetResult_Response * data = NULL;

  if (size) {
    data = (choirbot_interfaces__action__PositionAction_GetResult_Response *)allocator.zero_allocate(size, sizeof(choirbot_interfaces__action__PositionAction_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = choirbot_interfaces__action__PositionAction_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        choirbot_interfaces__action__PositionAction_GetResult_Response__fini(&data[i - 1]);
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
choirbot_interfaces__action__PositionAction_GetResult_Response__Sequence__fini(choirbot_interfaces__action__PositionAction_GetResult_Response__Sequence * array)
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
      choirbot_interfaces__action__PositionAction_GetResult_Response__fini(&array->data[i]);
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

choirbot_interfaces__action__PositionAction_GetResult_Response__Sequence *
choirbot_interfaces__action__PositionAction_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_GetResult_Response__Sequence * array = (choirbot_interfaces__action__PositionAction_GetResult_Response__Sequence *)allocator.allocate(sizeof(choirbot_interfaces__action__PositionAction_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = choirbot_interfaces__action__PositionAction_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
choirbot_interfaces__action__PositionAction_GetResult_Response__Sequence__destroy(choirbot_interfaces__action__PositionAction_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    choirbot_interfaces__action__PositionAction_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
choirbot_interfaces__action__PositionAction_GetResult_Response__Sequence__are_equal(const choirbot_interfaces__action__PositionAction_GetResult_Response__Sequence * lhs, const choirbot_interfaces__action__PositionAction_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!choirbot_interfaces__action__PositionAction_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
choirbot_interfaces__action__PositionAction_GetResult_Response__Sequence__copy(
  const choirbot_interfaces__action__PositionAction_GetResult_Response__Sequence * input,
  choirbot_interfaces__action__PositionAction_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(choirbot_interfaces__action__PositionAction_GetResult_Response);
    choirbot_interfaces__action__PositionAction_GetResult_Response * data =
      (choirbot_interfaces__action__PositionAction_GetResult_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!choirbot_interfaces__action__PositionAction_GetResult_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          choirbot_interfaces__action__PositionAction_GetResult_Response__fini(&data[i]);
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
    if (!choirbot_interfaces__action__PositionAction_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "choirbot_interfaces/action/detail/position_action__functions.h"

bool
choirbot_interfaces__action__PositionAction_FeedbackMessage__init(choirbot_interfaces__action__PositionAction_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    choirbot_interfaces__action__PositionAction_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!choirbot_interfaces__action__PositionAction_Feedback__init(&msg->feedback)) {
    choirbot_interfaces__action__PositionAction_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
choirbot_interfaces__action__PositionAction_FeedbackMessage__fini(choirbot_interfaces__action__PositionAction_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  choirbot_interfaces__action__PositionAction_Feedback__fini(&msg->feedback);
}

bool
choirbot_interfaces__action__PositionAction_FeedbackMessage__are_equal(const choirbot_interfaces__action__PositionAction_FeedbackMessage * lhs, const choirbot_interfaces__action__PositionAction_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!choirbot_interfaces__action__PositionAction_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
choirbot_interfaces__action__PositionAction_FeedbackMessage__copy(
  const choirbot_interfaces__action__PositionAction_FeedbackMessage * input,
  choirbot_interfaces__action__PositionAction_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!choirbot_interfaces__action__PositionAction_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

choirbot_interfaces__action__PositionAction_FeedbackMessage *
choirbot_interfaces__action__PositionAction_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_FeedbackMessage * msg = (choirbot_interfaces__action__PositionAction_FeedbackMessage *)allocator.allocate(sizeof(choirbot_interfaces__action__PositionAction_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(choirbot_interfaces__action__PositionAction_FeedbackMessage));
  bool success = choirbot_interfaces__action__PositionAction_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
choirbot_interfaces__action__PositionAction_FeedbackMessage__destroy(choirbot_interfaces__action__PositionAction_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    choirbot_interfaces__action__PositionAction_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
choirbot_interfaces__action__PositionAction_FeedbackMessage__Sequence__init(choirbot_interfaces__action__PositionAction_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_FeedbackMessage * data = NULL;

  if (size) {
    data = (choirbot_interfaces__action__PositionAction_FeedbackMessage *)allocator.zero_allocate(size, sizeof(choirbot_interfaces__action__PositionAction_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = choirbot_interfaces__action__PositionAction_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        choirbot_interfaces__action__PositionAction_FeedbackMessage__fini(&data[i - 1]);
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
choirbot_interfaces__action__PositionAction_FeedbackMessage__Sequence__fini(choirbot_interfaces__action__PositionAction_FeedbackMessage__Sequence * array)
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
      choirbot_interfaces__action__PositionAction_FeedbackMessage__fini(&array->data[i]);
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

choirbot_interfaces__action__PositionAction_FeedbackMessage__Sequence *
choirbot_interfaces__action__PositionAction_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  choirbot_interfaces__action__PositionAction_FeedbackMessage__Sequence * array = (choirbot_interfaces__action__PositionAction_FeedbackMessage__Sequence *)allocator.allocate(sizeof(choirbot_interfaces__action__PositionAction_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = choirbot_interfaces__action__PositionAction_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
choirbot_interfaces__action__PositionAction_FeedbackMessage__Sequence__destroy(choirbot_interfaces__action__PositionAction_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    choirbot_interfaces__action__PositionAction_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
choirbot_interfaces__action__PositionAction_FeedbackMessage__Sequence__are_equal(const choirbot_interfaces__action__PositionAction_FeedbackMessage__Sequence * lhs, const choirbot_interfaces__action__PositionAction_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!choirbot_interfaces__action__PositionAction_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
choirbot_interfaces__action__PositionAction_FeedbackMessage__Sequence__copy(
  const choirbot_interfaces__action__PositionAction_FeedbackMessage__Sequence * input,
  choirbot_interfaces__action__PositionAction_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(choirbot_interfaces__action__PositionAction_FeedbackMessage);
    choirbot_interfaces__action__PositionAction_FeedbackMessage * data =
      (choirbot_interfaces__action__PositionAction_FeedbackMessage *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!choirbot_interfaces__action__PositionAction_FeedbackMessage__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          choirbot_interfaces__action__PositionAction_FeedbackMessage__fini(&data[i]);
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
    if (!choirbot_interfaces__action__PositionAction_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
