# Install script for directory: C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/solver_code

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files/cvxpygen")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/build/solver_code/lib/qdldl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/build/solver_code/lib/amd/cmake_install.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/build/out/Debug/qocostatic.lib")
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/build/out/Release/qocostatic.lib")
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/build/out/MinSizeRel/qocostatic.lib")
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/build/out/RelWithDebInfo/qocostatic.lib")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/build/out/Debug/qoco.lib")
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/build/out/Release/qoco.lib")
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/build/out/MinSizeRel/qoco.lib")
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/build/out/RelWithDebInfo/qoco.lib")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/build/out/Debug/qoco.dll")
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/build/out/Release/qoco.dll")
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/build/out/MinSizeRel/qoco.dll")
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/build/out/RelWithDebInfo/qoco.dll")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/qoco" TYPE FILE FILES
    "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/solver_code/include/qoco.h"
    "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/solver_code/include/qoco_api.h"
    "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/solver_code/include/input_validation.h"
    "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/solver_code/include/qoco_linalg.h"
    "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/solver_code/include/kkt.h"
    "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/solver_code/include/cone.h"
    "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/solver_code/include/qoco_status.h"
    "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/solver_code/include/equilibration.h"
    "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/solver_code/include/enums.h"
    "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/solver_code/include/definitions.h"
    "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/solver_code/include/structs.h"
    "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/solver_code/include/timer.h"
    "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/solver_code/include/qoco_utils.h"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "C:/Users/caseY/Documents/python/KSP-Auto-Landing/solver/GFOLD/compiled_solvers/normal_landing/c/build/solver_code/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
