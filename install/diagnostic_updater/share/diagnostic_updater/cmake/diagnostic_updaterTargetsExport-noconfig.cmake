#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "diagnostic_updater::diagnostic_updater" for configuration ""
set_property(TARGET diagnostic_updater::diagnostic_updater APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(diagnostic_updater::diagnostic_updater PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdiagnostic_updater.so"
  IMPORTED_SONAME_NOCONFIG "libdiagnostic_updater.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS diagnostic_updater::diagnostic_updater )
list(APPEND _IMPORT_CHECK_FILES_FOR_diagnostic_updater::diagnostic_updater "${_IMPORT_PREFIX}/lib/libdiagnostic_updater.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
