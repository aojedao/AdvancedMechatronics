// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from choirbot_interfaces:srv/TaskCompletionService.idl
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
#include "choirbot_interfaces/srv/detail/task_completion_service__struct.h"
#include "choirbot_interfaces/srv/detail/task_completion_service__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool choirbot_interfaces__srv__task_completion_service__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[79];
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
    assert(strncmp("choirbot_interfaces.srv._task_completion_service.TaskCompletionService_Request", full_classname_dest, 78) == 0);
  }
  choirbot_interfaces__srv__TaskCompletionService_Request * ros_message = _ros_message;
  {  // agent_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "agent_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->agent_id = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // task_seq_num
    PyObject * field = PyObject_GetAttrString(_pymsg, "task_seq_num");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->task_seq_num = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * choirbot_interfaces__srv__task_completion_service__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TaskCompletionService_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("choirbot_interfaces.srv._task_completion_service");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TaskCompletionService_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  choirbot_interfaces__srv__TaskCompletionService_Request * ros_message = (choirbot_interfaces__srv__TaskCompletionService_Request *)raw_ros_message;
  {  // agent_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->agent_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "agent_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // task_seq_num
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->task_seq_num);
    {
      int rc = PyObject_SetAttrString(_pymessage, "task_seq_num", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "choirbot_interfaces/srv/detail/task_completion_service__struct.h"
// already included above
// #include "choirbot_interfaces/srv/detail/task_completion_service__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool choirbot_interfaces__srv__task_completion_service__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[80];
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
    assert(strncmp("choirbot_interfaces.srv._task_completion_service.TaskCompletionService_Response", full_classname_dest, 79) == 0);
  }
  choirbot_interfaces__srv__TaskCompletionService_Response * ros_message = _ros_message;
  ros_message->structure_needs_at_least_one_member = 0;

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * choirbot_interfaces__srv__task_completion_service__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TaskCompletionService_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("choirbot_interfaces.srv._task_completion_service");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TaskCompletionService_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  (void)raw_ros_message;

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
