#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "qdldl::qdldlstatic" for configuration "Debug"
set_property(TARGET qdldl::qdldlstatic APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(qdldl::qdldlstatic PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "C"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/qdldl.lib"
  )

list(APPEND _cmake_import_check_targets qdldl::qdldlstatic )
list(APPEND _cmake_import_check_files_for_qdldl::qdldlstatic "${_IMPORT_PREFIX}/lib/qdldl.lib" )

# Import target "qdldl::qdldl" for configuration "Debug"
set_property(TARGET qdldl::qdldl APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(qdldl::qdldl PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/qdldl.lib"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/qdldl.dll"
  )

list(APPEND _cmake_import_check_targets qdldl::qdldl )
list(APPEND _cmake_import_check_files_for_qdldl::qdldl "${_IMPORT_PREFIX}/lib/qdldl.lib" "${_IMPORT_PREFIX}/bin/qdldl.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
