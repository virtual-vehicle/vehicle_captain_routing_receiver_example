# Authors:
# - Christoph Pilz (christoph.pilz@v2c2.at)
#

cmake_minimum_required(VERSION 3.16)

# set the project name
project(cpp_receiver VERSION 0.1 DESCRIPTION "cpp receiver for the V2X Routing Platform")

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)

# add targets
add_subdirectory(vcits)

# add include directories
include_directories(.)
include_directories(vcits/asn1c) #vcits lib headers reference this

# add the executable
add_executable(cppReceiver main.cpp)

# set target properties - CMake does not know without source files
set_target_properties(cppReceiver
        PROPERTIES
        COMPILE_FLAGS "-Wall -Wextra -fno-strict-aliasing -pipe -pthread -lrt -lm -lzmq"
        LINK_FLAGS "-Wall -Wextra -fno-strict-aliasing -pipe -pthread -lrt -lm -lzmq"
        VERSION ${PROJECT_VERSION}
        )

# ZeroMQ configuration
include(FindZeroMQ.cmake)
#message("ZeroMQ_INCLUDE_DIR: ${ZEROMQ_INCLUDE_DIR}")
#message("ZeroMQ_LIBRARY: ${ZEROMQ_LIBRARIES}")

# specify header locations
target_include_directories(cppReceiver
        PUBLIC
        ${ZEROMQ_INCLUDE_DIR}
        )

# link libraries
target_link_libraries(cppReceiver
        PUBLIC
        vcits
        ${ZEROMQ_LIBRARIES}
        )