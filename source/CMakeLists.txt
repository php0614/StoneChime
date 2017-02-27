cmake_minimum_required(VERSION 3.2)

project (StoneChime)

include_directories(SC Source... /include/plugin_interface)
include_directories(SC Source... /include/common)
include_directories(SC Source... /external_libraries/libsndfile/)

set(CMAKE_SHARED_MODULE_PREFIX "")
set(CMAKE_SHARED_MODULE_SUFFIX ".scx")

add_library(StoneChime MODULE StoneChime.cpp StoneChime.h Membrane_shape.c Membrane_shape.h VarMembrane.cpp)
