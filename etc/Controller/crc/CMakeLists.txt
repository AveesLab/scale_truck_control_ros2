cmake_minimum_required(VERSION 3.0.2)

project(crc)

enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions(-DZMQ_BUILD_DRAFT_API=1)

set(ZeroMQ_DIR /usr/local/lib/)
find_path(ZMQ_INCLUDE_DIR zmq.h)
find_library(ZMQ_LIBRARY NAMES zmq)
set(ZeroMQ_LIBRARIES ${ZMQ_LIBRARY})
set(ZeroMQ_INCLUDE_DIRS ${ZMQ_INCLUDE_DIR})

set(cppzmq_DIR /usr/local/share/cmake/cppzmq)

set(CMAKE_THREAD_LIBS_INIT "-lpthread")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

add_library(cppzmq INTERFACE)

target_link_libraries(cppzmq INTERFACE $<$<CONFIG:Debug>:libzmq>$<$<CONFIG:Release>:libzmq-static>)
target_include_directories(cppzmq INTERFACE ${cppzmq_DIR})
target_compile_definitions(cppzmq INTERFACE ZMQ_BUILD_DRAFT_API=1)

include_directories(
	include
	${cppzmq_INCLUDE_DIRS}
	${ZeroMQ_INCLUDE_DIRS}
)

set(PROJECT_LIB_FILES
	zmq_class.cpp
	crc.cpp
)

add_library(${PROJECT_NAME}_lib
	${PROJECT_LIB_FILES}
)

add_executable(${PROJECT_NAME}
	main.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
	stdc++
	${cppzmq_LIBRARIES}
	${ZeroMQ_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
	${PROJECT_NAME}_lib
)
