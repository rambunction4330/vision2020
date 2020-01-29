cmake_minimum_required(VERSION 2.8)
project( Vision )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package ( Threads REQUIRED )
find_package ( OpenCV REQUIRED )
add_executable( main main.cpp )
target_link_libraries ( main ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS})
