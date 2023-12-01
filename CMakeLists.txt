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

option (BRT_CREATE_VST3 "Create the VST3 Plugin" OFF)
option (BRT_CREATE_PD "Crete the PD plugin" ON)

  if(BRT_CREATE_PD)

  avnd_make_pd(
    TARGET BrtListenerModel
    MAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/Processor.hpp"
    MAIN_CLASS BrtListenerModel
    C_NAME brt_listener_model
  )

  target_compile_definitions(
    BrtListenerModel
    PUBLIC
      -DAVND_PD=1
  )

endif(BRT_CREATE_PD)

if (BRT_CREATE_VST3)

  avnd_make_vst3(
    TARGET BrtListenerModel
    MAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/Processor.hpp"
    MAIN_CLASS BrtListenerModel
    C_NAME brt_listener_model
  )

  target_compile_definitions(
    BrtListenerModel
    PUBLIC
      -DAVND_VST3=1
  )

endif(BRT_CREATE_VST3)

# avnd_make_standalone(
#   TARGET BrtListenerModel
#   MAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/Processor.hpp"
#   MAIN_CLASS BrtListenerModel
#   C_NAME brt_listener_model
# )


# Create zip file for installations.  

#1) First install files to directory set by CMAKE_INSTALL_PATH
install(FILES scripts/readme.txt DESTINATION brt/)
if (WIN32)
   install(PROGRAMS scripts/setup_win.bat DESTINATION brt/scripts)
elseif(APPLE)
   install(PROGRAMS scripts/setup_mac.sh DESTINATION brt/scripts )
endif(WIN32)

install(DIRECTORY resources DESTINATION brt)

if (BRT_CREATE_VST3)
  if (WIN32)
    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/vst3/ DESTINATION brt/vst3plugins/)
  elseif(APPLE)
    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/VST3/Release/ DESTINATION brt/vst3plugins/)
  endif(WIN32)
endif(BRT_CREATE_VST3)

if (BRT_CREATE_PD)
  if (WIN32)
      install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/pd/Release/ DESTINATION brt/pd/)
  elseif(APPLE)
      set(CMAKE_GENERATOR_VALUE "${CMAKE_GENERATOR}" CACHE STRING "Need to read internal variable CMAKE_GENERATOR" FORCE)
      message(STATUS "${GMAKE_GENERATOR_VALUE}")
      if (MAKE_GENERATOR_VALUE STREQUAL Xcode)
        message(STATUS "---Detected Xcode")
        install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/pd/Release/ DESTINATION brt/pd/)
      else()
        message(STATUS "---Assuming Unix Makefile")
        install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/pd/ DESTINATION brt/pd/)
      endif()
  endif(WIN32)
endif(BRT_CREATE_PD)

# 2) Package files from installation directory into zip file
# https://stackoverflow.com/questions/7050997/zip-files-using-cmake
set(CPACK_GENERATOR "ZIP")
set(CPACK_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_VERBATIM_VARIABLES YES)
include(CPack)
