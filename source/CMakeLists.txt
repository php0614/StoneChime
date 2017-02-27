cmake_minimum_required(VERSION 3.2)

project (StoneChime)

include_directories(/Works_S/ScSource/include/plugin_interface)
include_directories(/Works_S/ScSource/include/common)
include_directories(/Works_S/ScSource/external_libraries/libsndfile/)

set(CMAKE_SHARED_MODULE_PREFIX "")
set(CMAKE_SHARED_MODULE_SUFFIX ".scx")

add_library(StoneChime MODULE StoneChime.cpp StoneChime.h Membrane_shape.c Membrane_shape.h VarMembrane.cpp)