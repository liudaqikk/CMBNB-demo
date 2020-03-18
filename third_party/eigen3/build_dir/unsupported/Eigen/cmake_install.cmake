# Install script for directory: /media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE FILE FILES
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/AdolcForward"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/AlignedVector3"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/ArpackSupport"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/AutoDiff"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/BVH"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/EulerAngles"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/FFT"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/IterativeSolvers"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/KroneckerProduct"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/LevenbergMarquardt"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/MatrixFunctions"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/MoreVectorization"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/MPRealSupport"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/NonLinearOptimization"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/NumericalDiff"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/OpenGLSupport"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/Polynomials"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/Skyline"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/SparseExtra"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/SpecialFunctions"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/Splines"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE DIRECTORY FILES "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/unsupported/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/build_dir/unsupported/Eigen/CXX11/cmake_install.cmake")

endif()

