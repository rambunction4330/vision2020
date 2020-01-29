cmake_minimum_required(VERSION 2.8)
project( visualTesting )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package ( OpenCV REQUIRED )
add_executable( visualTesting visualTesting.cpp )
target_link_libraries ( visualTesting ${OpenCV_LIBS})
