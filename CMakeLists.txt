# cmake version to be used
cmake_minimum_required( VERSION 3.0 )

# project name
project( vision2020 )

# flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# files

# include
include_directories( ./include ./src/main ./libs )

# target
add_executable( main ./src/main/main.cpp )

# external libs
find_package( OpenCV REQUIRED )
target_link_libraries( main ${OpenCV_LIBS} )

# add_subdirectory( ./libs/Randomize )
# target_link_libraries( binary randomize )

target_link_libraries( binary ${OpenCV_LIBS} )
