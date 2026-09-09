#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "auracle_hardware::auracle_hardware" for configuration ""
set_property(TARGET auracle_hardware::auracle_hardware APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(auracle_hardware::auracle_hardware PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libauracle_hardware.so"
  IMPORTED_SONAME_NOCONFIG "libauracle_hardware.so"
  )

list(APPEND _cmake_import_check_targets auracle_hardware::auracle_hardware )
list(APPEND _cmake_import_check_files_for_auracle_hardware::auracle_hardware "${_IMPORT_PREFIX}/lib/libauracle_hardware.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
