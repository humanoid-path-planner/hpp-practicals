# Copyright (c) 2017, 2020 CNRS Author: Florent Lamiraux, Guilhem Saurel
#
# this file is free software: you can redistribute it and/or modify it under the
# terms of the GNU Lesser General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option) any
# later version.
#
# this file is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# A PARTICULAR PURPOSE.  See the GNU General Lesser Public License for more
# details.  You should have received a copy of the GNU Lesser General Public
# License along with this file.  If not, see <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.22)

set(PROJECT_NAME hpp_practicals)
set(PROJECT_DESCRIPTION "Practicals for Humanoid Path Planner software.")

set(PROJECT_USE_CMAKE_EXPORT TRUE)

option(USE_CORBA "Enable dependency to hpp-manipulation-corba" OFF)

find_package(jrl-cmakemodules QUIET CONFIG)
if(jrl-cmakemodules_FOUND)
    get_property(
        JRL_CMAKE_MODULES
        TARGET jrl-cmakemodules::jrl-cmakemodules
        PROPERTY INTERFACE_INCLUDE_DIRECTORIES
    )
    message(STATUS "JRL cmakemodules found on system at ${JRL_CMAKE_MODULES}")
else()
    message(STATUS "JRL cmakemodules not found. Let's fetch it.")
    include(FetchContent)
    FetchContent_Declare(
        "jrl-cmakemodules"
        GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git"
    )
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
endif()

include("${JRL_CMAKE_MODULES}/hpp.cmake")
include("${JRL_CMAKE_MODULES}/python.cmake")

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

findpython()

if(USE_CORBA)
    find_package(hpp-manipulation-corba REQUIRED)
    find_package(hpp-gui REQUIRED)
endif()
find_package(hpp-gepetto-viewer REQUIRED)
find_package(hpp-plot REQUIRED)

set(URDF_UR_BENCHMARK_FILES
    urdf/ur_benchmark/obstacles.urdf
    urdf/ur_benchmark/table.urdf
    urdf/ur_benchmark/wall.urdf
    urdf/ur_benchmark/box.urdf
    urdf/ur_benchmark/ground.urdf
    urdf/ur_benchmark/ground_with_obstacles.urdf
    urdf/ur_benchmark/pokeball.urdf
)

set(URDF_UR_FILES urdf/ur5_gripper.urdf urdf/ur5_joint_limited_robot.urdf)

set(SRDF_UR_FILES srdf/ur5_gripper.srdf srdf/ur5_joint_limited_robot.srdf)

set(SRDF_UR_BENCHMARK_FILES
    srdf/ur_benchmark/box.srdf
    srdf/ur_benchmark/ground.srdf
    srdf/ur_benchmark/pokeball.srdf
)

# Install environment and robot model files
#
install(
    FILES ${URDF_UR_BENCHMARK_FILES}
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/urdf/ur_benchmark
)

install(
    FILES ${SRDF_UR_BENCHMARK_FILES}
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/srdf/ur_benchmark
)

install(
    FILES ${URDF_UR_FILES}
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/urdf
)

install(
    FILES ${SRDF_UR_FILES}
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/srdf
)

install(
    DIRECTORY meshes
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}
)

# Install python client files
#
if(USE_CORBA)
    install(
        FILES
            ${PROJECT_SOURCE_DIR}/src/hpp/corbaserver/practicals/ur5/robot.py
            ${PROJECT_SOURCE_DIR}/src/hpp/corbaserver/practicals/ur5/__init__.py
        DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/practicals/ur5
    )

    install(
        FILES ${PROJECT_SOURCE_DIR}/src/hpp/corbaserver/practicals/__init__.py
        DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/practicals
    )

    install(
        FILES
            ${PROJECT_SOURCE_DIR}/src/hpp/corbaserver/practicals/manipulation/__init__.py
        DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/practicals/manipulation
    )

    install(
        FILES
            ${PROJECT_SOURCE_DIR}/src/hpp/corbaserver/practicals/manipulation/ur5/robot.py
            ${PROJECT_SOURCE_DIR}/src/hpp/corbaserver/practicals/manipulation/ur5/__init__.py
        DESTINATION
            ${PYTHON_SITELIB}/hpp/corbaserver/practicals/manipulation/ur5
    )
endif()

add_library(${PROJECT_NAME} INTERFACE)
install(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
