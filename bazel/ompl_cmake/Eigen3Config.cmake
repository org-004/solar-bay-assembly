# A stand-in for the Eigen3Config.cmake that an installed Eigen would provide.
# OMPL's CMake files ask for Eigen with find_package(Eigen3 REQUIRED NO_MODULE),
# which only accepts such a file, and the registry's @eigen is a plain Bazel
# library that ships none. rules_foreign_cc copies @eigen's headers into the
# include directory under CMAKE_PREFIX_PATH; this file finds them there, and
# only there, and names them Eigen3::Eigen the way Eigen's own file does.
find_path(OMPL_BAZEL_EIGEN3_INCLUDE_DIR
    NAMES Eigen/Core
    PATH_SUFFIXES include
    NO_CMAKE_ENVIRONMENT_PATH
    NO_SYSTEM_ENVIRONMENT_PATH
    NO_CMAKE_SYSTEM_PATH)
if(NOT OMPL_BAZEL_EIGEN3_INCLUDE_DIR)
    set(Eigen3_FOUND FALSE)
    set(Eigen3_NOT_FOUND_MESSAGE "Eigen/Core is not under CMAKE_PREFIX_PATH; is @eigen in the deps of the cmake rule?")
    return()
endif()
if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${OMPL_BAZEL_EIGEN3_INCLUDE_DIR}")
endif()
set(EIGEN3_INCLUDE_DIR "${OMPL_BAZEL_EIGEN3_INCLUDE_DIR}")
set(EIGEN3_INCLUDE_DIRS "${OMPL_BAZEL_EIGEN3_INCLUDE_DIR}")
set(Eigen3_FOUND TRUE)
