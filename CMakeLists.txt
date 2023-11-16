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

# avnd_make_pd(
#   TARGET BrtListenerModel
#   MAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/Processor.hpp"
#   MAIN_CLASS BrtListenerModel
#   C_NAME brt_listener_model
# )

avnd_make_vst3(
  TARGET BrtListenerModel
  MAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/Processor.hpp"
  MAIN_CLASS BrtListenerModel
  C_NAME brt_listener_model
)

option (BRT_CREATE_VST3_INSTALLER "Create or not an installer for the VST3 Plugin" ON)

# Create zip file for VST3 plugin installation.  
if (BRT_CREATE_VST3_INSTALLER)
#1) First install files to directory set by CMAKE_INSTALL_PATH
  install(FILES scripts/readme.txt DESTINATION brt/)
  if (WIN32)
    install(PROGRAMS scripts/setup_win.bat DESTINATION brt/scripts)
  elseif(APPLE)
    install(PROGRAMS scripts/setup_mac.sh DESTINATION brt/scripts )
  endif(WIN32)

  install(DIRECTORY resources DESTINATION brt)
  if (WIN32)
    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/vst3/ DESTINATION brt/vst3plugins/)
  elseif(APPLE)
    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/VST3/Release/ DESTINATION brt/vst3plugins/)
  endif(WIN32)

# 2) Package files from installation directory into zip file
# https://stackoverflow.com/questions/7050997/zip-files-using-cmake
  set(CPACK_GENERATOR "ZIP")
  set(CPACK_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}")
  set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
  set(CPACK_VERBATIM_VARIABLES YES)
  include(CPack)
endif(BRT_CREATE_VST3_INSTALLER)