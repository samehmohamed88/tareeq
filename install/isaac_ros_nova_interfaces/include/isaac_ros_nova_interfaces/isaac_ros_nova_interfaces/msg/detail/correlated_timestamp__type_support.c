// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from isaac_ros_nova_interfaces:msg/CorrelatedTimestamp.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "isaac_ros_nova_interfaces/msg/detail/correlated_timestamp__rosidl_typesupport_introspection_c.h"
#include "isaac_ros_nova_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "isaac_ros_nova_interfaces/msg/detail/correlated_timestamp__functions.h"
#include "isaac_ros_nova_interfaces/msg/detail/correlated_timestamp__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__rosidl_typesupport_introspection_c__CorrelatedTimestamp_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__init(message_memory);
}

void isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__rosidl_typesupport_introspection_c__CorrelatedTimestamp_fini_function(void * message_memory)
{
  isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__rosidl_typesupport_introspection_c__CorrelatedTimestamp_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(isaac_ros_nova_interfaces__msg__CorrelatedTimestamp, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "phc_val",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(isaac_ros_nova_interfaces__msg__CorrelatedTimestamp, phc_val),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tsc_val",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(isaac_ros_nova_interfaces__msg__CorrelatedTimestamp, tsc_val),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "phc2_val",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(isaac_ros_nova_interfaces__msg__CorrelatedTimestamp, phc2_val),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sys_val",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(isaac_ros_nova_interfaces__msg__CorrelatedTimestamp, sys_val),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "phc_latency",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(isaac_ros_nova_interfaces__msg__CorrelatedTimestamp, phc_latency),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__rosidl_typesupport_introspection_c__CorrelatedTimestamp_message_members = {
  "isaac_ros_nova_interfaces__msg",  // message namespace
  "CorrelatedTimestamp",  // message name
  6,  // number of fields
  sizeof(isaac_ros_nova_interfaces__msg__CorrelatedTimestamp),
  isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__rosidl_typesupport_introspection_c__CorrelatedTimestamp_message_member_array,  // message members
  isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__rosidl_typesupport_introspection_c__CorrelatedTimestamp_init_function,  // function to initialize message memory (memory has to be allocated)
  isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__rosidl_typesupport_introspection_c__CorrelatedTimestamp_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__rosidl_typesupport_introspection_c__CorrelatedTimestamp_message_type_support_handle = {
  0,
  &isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__rosidl_typesupport_introspection_c__CorrelatedTimestamp_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_isaac_ros_nova_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, isaac_ros_nova_interfaces, msg, CorrelatedTimestamp)() {
  isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__rosidl_typesupport_introspection_c__CorrelatedTimestamp_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__rosidl_typesupport_introspection_c__CorrelatedTimestamp_message_type_support_handle.typesupport_identifier) {
    isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__rosidl_typesupport_introspection_c__CorrelatedTimestamp_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &isaac_ros_nova_interfaces__msg__CorrelatedTimestamp__rosidl_typesupport_introspection_c__CorrelatedTimestamp_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
