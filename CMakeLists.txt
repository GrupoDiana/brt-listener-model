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

if (BRT_CREATE_VST3_INSTALLER)
  if (WIN32)
    install(FILES scripts/readme.txt DESTINATION brt/)
    install(FILES scripts/setup_brt_run_me_as_admin.bat DESTINATION brt/scripts)
  endif(WIN32)
  install(DIRECTORY resources DESTINATION brt)
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/vst3/ DESTINATION brt/vst3plugins/)
  # https://stackoverflow.com/questions/7050997/zip-files-using-cmake
  set(CPACK_GENERATOR "ZIP")
  set(CPACK_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}")
  set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
  set(CPACK_VERBATIM_VARIABLES YES)
  include(CPack)
endif(BRT_CREATE_VST3_INSTALLER)