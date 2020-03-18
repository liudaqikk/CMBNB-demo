# Install script for directory: /media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE FILE FILES
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/Cholesky"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/CholmodSupport"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/Core"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/Dense"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/Eigen"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/Eigenvalues"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/Geometry"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/Householder"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/IterativeLinearSolvers"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/Jacobi"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/LU"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/MetisSupport"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/OrderingMethods"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/PaStiXSupport"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/PardisoSupport"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/QR"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/QtAlignedMalloc"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/SPQRSupport"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/SVD"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/Sparse"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/SparseCholesky"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/SparseCore"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/SparseLU"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/SparseQR"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/StdDeque"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/StdList"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/StdVector"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/SuperLUSupport"
    "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/UmfPackSupport"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE DIRECTORY FILES "/media/daqi/file/documen/project/SSA_reference/reading/code/point_process_optimisation-master/source/eigen3/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

