// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from choirbot_interfaces:msg/PositionTaskArray.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "choirbot_interfaces/msg/detail/position_task_array__struct.h"
#include "choirbot_interfaces/msg/detail/position_task_array__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "choirbot_interfaces/msg/detail/position_task__functions.h"
// end nested array functions include
bool choirbot_interfaces__msg__position_task__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * choirbot_interfaces__msg__position_task__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool choirbot_interfaces__msg__position_task_array__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[63];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("choirbot_interfaces.msg._position_task_array.PositionTaskArray", full_classname_dest, 62) == 0);
  }
  choirbot_interfaces__msg__PositionTaskArray * ros_message = _ros_message;
  {  // tasks
    PyObject * field = PyObject_GetAttrString(_pymsg, "tasks");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'tasks'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!choirbot_interfaces__msg__PositionTask__Sequence__init(&(ros_message->tasks), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create choirbot_interfaces__msg__PositionTask__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    choirbot_interfaces__msg__PositionTask * dest = ros_message->tasks.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!choirbot_interfaces__msg__position_task__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // all_tasks_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "all_tasks_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->all_tasks_count = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // label
    PyObject * field = PyObject_GetAttrString(_pymsg, "label");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->label = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * choirbot_interfaces__msg__position_task_array__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PositionTaskArray */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("choirbot_interfaces.msg._position_task_array");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PositionTaskArray");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  choirbot_interfaces__msg__PositionTaskArray * ros_message = (choirbot_interfaces__msg__PositionTaskArray *)raw_ros_message;
  {  // tasks
    PyObject * field = NULL;
    size_t size = ros_message->tasks.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    choirbot_interfaces__msg__PositionTask * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->tasks.data[i]);
      PyObject * pyitem = choirbot_interfaces__msg__position_task__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "tasks", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // all_tasks_count
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->all_tasks_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "all_tasks_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // label
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->label);
    {
      int rc = PyObject_SetAttrString(_pymessage, "label", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
