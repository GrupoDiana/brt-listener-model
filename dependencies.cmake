include(FetchContent)

# Download our fork of avendish
FetchContent_Declare(
  avendish
  GIT_REPOSITORY "https://github.com/GrupoDiana/avendish"
  GIT_TAG  main 
  GIT_PROGRESS true
)
FetchContent_Populate(avendish)

# Donwload the BRT Library
include(FetchContent)
FetchContent_Declare(
    brt
    GIT_REPOSITORY "https://github.com/GrupoDiana/BRTLibrary.git"
    GIT_TAG cmake
    GIT_PROGRESS true
)
FetchContent_MakeAvailable(brt)

set(CMAKE_PREFIX_PATH "${avendish_SOURCE_DIR};${CMAKE_PREFIX_PATH}")
find_package(Avendish REQUIRED)
