// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from choirbot_interfaces:msg/PositionTask.idl
// generated code does not contain a copyright notice

#ifndef CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK__FUNCTIONS_H_
#define CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "choirbot_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "choirbot_interfaces/msg/detail/position_task__struct.h"

/// Initialize msg/PositionTask message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * choirbot_interfaces__msg__PositionTask
 * )) before or use
 * choirbot_interfaces__msg__PositionTask__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_choirbot_interfaces
bool
choirbot_interfaces__msg__PositionTask__init(choirbot_interfaces__msg__PositionTask * msg);

/// Finalize msg/PositionTask message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_choirbot_interfaces
void
choirbot_interfaces__msg__PositionTask__fini(choirbot_interfaces__msg__PositionTask * msg);

/// Create msg/PositionTask message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * choirbot_interfaces__msg__PositionTask__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_choirbot_interfaces
choirbot_interfaces__msg__PositionTask *
choirbot_interfaces__msg__PositionTask__create();

/// Destroy msg/PositionTask message.
/**
 * It calls
 * choirbot_interfaces__msg__PositionTask__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_choirbot_interfaces
void
choirbot_interfaces__msg__PositionTask__destroy(choirbot_interfaces__msg__PositionTask * msg);

/// Check for msg/PositionTask message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_choirbot_interfaces
bool
choirbot_interfaces__msg__PositionTask__are_equal(const choirbot_interfaces__msg__PositionTask * lhs, const choirbot_interfaces__msg__PositionTask * rhs);

/// Copy a msg/PositionTask message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_choirbot_interfaces
bool
choirbot_interfaces__msg__PositionTask__copy(
  const choirbot_interfaces__msg__PositionTask * input,
  choirbot_interfaces__msg__PositionTask * output);

/// Initialize array of msg/PositionTask messages.
/**
 * It allocates the memory for the number of elements and calls
 * choirbot_interfaces__msg__PositionTask__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_choirbot_interfaces
bool
choirbot_interfaces__msg__PositionTask__Sequence__init(choirbot_interfaces__msg__PositionTask__Sequence * array, size_t size);

/// Finalize array of msg/PositionTask messages.
/**
 * It calls
 * choirbot_interfaces__msg__PositionTask__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_choirbot_interfaces
void
choirbot_interfaces__msg__PositionTask__Sequence__fini(choirbot_interfaces__msg__PositionTask__Sequence * array);

/// Create array of msg/PositionTask messages.
/**
 * It allocates the memory for the array and calls
 * choirbot_interfaces__msg__PositionTask__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_choirbot_interfaces
choirbot_interfaces__msg__PositionTask__Sequence *
choirbot_interfaces__msg__PositionTask__Sequence__create(size_t size);

/// Destroy array of msg/PositionTask messages.
/**
 * It calls
 * choirbot_interfaces__msg__PositionTask__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_choirbot_interfaces
void
choirbot_interfaces__msg__PositionTask__Sequence__destroy(choirbot_interfaces__msg__PositionTask__Sequence * array);

/// Check for msg/PositionTask message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_choirbot_interfaces
bool
choirbot_interfaces__msg__PositionTask__Sequence__are_equal(const choirbot_interfaces__msg__PositionTask__Sequence * lhs, const choirbot_interfaces__msg__PositionTask__Sequence * rhs);

/// Copy an array of msg/PositionTask messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_choirbot_interfaces
bool
choirbot_interfaces__msg__PositionTask__Sequence__copy(
  const choirbot_interfaces__msg__PositionTask__Sequence * input,
  choirbot_interfaces__msg__PositionTask__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CHOIRBOT_INTERFACES__MSG__DETAIL__POSITION_TASK__FUNCTIONS_H_
