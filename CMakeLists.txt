cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

project(BrtListenerModel VERSION 0.2.0)

include(dependencies.cmake)

add_library(BrtListenerModel STATIC
  src/Model.cpp
  src/Model.hpp
  src/UI.hpp
  src/Processor.hpp
)

target_include_directories(BrtListenerModel 
    PUBLIC brt/include
 )

target_link_libraries(BrtListenerModel PUBLIC brt )

avnd_make_pd(
  TARGET BrtListenerModel
  MAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/Processor.hpp"
  MAIN_CLASS BrtListenerModel
  C_NAME brt_listener_model
)

avnd_make_vst3(
  TARGET BrtListenerModel
  MAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/Processor.hpp"
  MAIN_CLASS BrtListenerModel
  C_NAME brt_listener_model
)