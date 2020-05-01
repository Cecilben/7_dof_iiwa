cmake_minimum_required(VERSION 3.15)
project(7_dof_iiwa)

set(CMAKE_CXX_STANDARD 20)

add_executable(7_dof_iiwa main.cpp inverse_kinematics.cpp inverse_kinematics.h forward_kinematics.cpp forward_kinematics.h)