# cmake recipe to build an OPC-UA server based on the open62541 project.
project(sitara-servo-drive-opcua-server)

# This version requirement comes from open62541
cmake_minimum_required(VERSION 3.0...3.12)
if(${CMAKE_VERSION} VERSION_LESS 3.12)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif()

# Find the open62541 package.
find_package(open62541)

# Add local source files to the executable
add_executable(sitara_servo_drive_opcua_server src/main.c)

# Link with the open62541 library
target_link_libraries(sitara_servo_drive_opcua_server open62541::open62541)

# Install the resulting executable under <prefix>/bin
install(TARGETS sitara_servo_drive_opcua_server DESTINATION bin)
