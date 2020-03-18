# Install script for directory: /media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
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

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3" TYPE FILE FILES "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/signature_of_eigen3_matrix_library")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pkgconfig" TYPE FILE FILES "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/build_dir/eigen3.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake/Eigen3Targets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake/Eigen3Targets.cmake"
         "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/build_dir/CMakeFiles/Export/share/eigen3/cmake/Eigen3Targets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake/Eigen3Targets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake/Eigen3Targets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake" TYPE FILE FILES "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/build_dir/CMakeFiles/Export/share/eigen3/cmake/Eigen3Targets.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake" TYPE FILE FILES
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/cmake/UseEigen3.cmake"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/build_dir/Eigen3Config.cmake"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/build_dir/Eigen3ConfigVersion.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/build_dir/Eigen/cmake_install.cmake")
  include("/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/build_dir/unsupported/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/build_dir/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
