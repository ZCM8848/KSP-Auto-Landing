#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "qdldl::qdldlstatic" for configuration "RelWithDebInfo"
set_property(TARGET qdldl::qdldlstatic APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(qdldl::qdldlstatic PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELWITHDEBINFO "C"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/qdldl.lib"
  )

list(APPEND _cmake_import_check_targets qdldl::qdldlstatic )
list(APPEND _cmake_import_check_files_for_qdldl::qdldlstatic "${_IMPORT_PREFIX}/lib/qdldl.lib" )

# Import target "qdldl::qdldl" for configuration "RelWithDebInfo"
set_property(TARGET qdldl::qdldl APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(qdldl::qdldl PROPERTIES
  IMPORTED_IMPLIB_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/qdldl.lib"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/bin/qdldl.dll"
  )

list(APPEND _cmake_import_check_targets qdldl::qdldl )
list(APPEND _cmake_import_check_files_for_qdldl::qdldl "${_IMPORT_PREFIX}/lib/qdldl.lib" "${_IMPORT_PREFIX}/bin/qdldl.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
