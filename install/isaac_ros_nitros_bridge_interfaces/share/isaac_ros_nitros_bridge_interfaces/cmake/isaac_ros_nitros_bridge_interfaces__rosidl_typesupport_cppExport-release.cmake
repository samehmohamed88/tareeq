#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "isaac_ros_nitros_bridge_interfaces::isaac_ros_nitros_bridge_interfaces__rosidl_typesupport_cpp" for configuration "Release"
set_property(TARGET isaac_ros_nitros_bridge_interfaces::isaac_ros_nitros_bridge_interfaces__rosidl_typesupport_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(isaac_ros_nitros_bridge_interfaces::isaac_ros_nitros_bridge_interfaces__rosidl_typesupport_cpp PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "rosidl_runtime_c::rosidl_runtime_c;rosidl_typesupport_cpp::rosidl_typesupport_cpp;rosidl_typesupport_c::rosidl_typesupport_c"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libisaac_ros_nitros_bridge_interfaces__rosidl_typesupport_cpp.so"
  IMPORTED_SONAME_RELEASE "libisaac_ros_nitros_bridge_interfaces__rosidl_typesupport_cpp.so"
  )

list(APPEND _cmake_import_check_targets isaac_ros_nitros_bridge_interfaces::isaac_ros_nitros_bridge_interfaces__rosidl_typesupport_cpp )
list(APPEND _cmake_import_check_files_for_isaac_ros_nitros_bridge_interfaces::isaac_ros_nitros_bridge_interfaces__rosidl_typesupport_cpp "${_IMPORT_PREFIX}/lib/libisaac_ros_nitros_bridge_interfaces__rosidl_typesupport_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
