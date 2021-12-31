########################################################################
# Build Soapy SDR support module for RTL-TCP Devices
########################################################################
cmake_minimum_required(VERSION 2.8.7)
project(SoapyRTLTCP CXX)

find_package(SoapySDR "0.4.0" NO_MODULE REQUIRED)
if (NOT SoapySDR_FOUND)
    message(FATAL_ERROR "Soapy SDR development files not found...")
endif ()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Test for Atomics
include(CheckAtomic)
if(NOT HAVE_CXX_ATOMICS_WITHOUT_LIB OR NOT HAVE_CXX_ATOMICS64_WITHOUT_LIB)
    set(ATOMIC_LIBS "atomic")
endif()

#enable c++11 features
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    #C++11 is a required language feature for this project
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" HAS_STD_CXX11)
    if(HAS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    else(HAS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    endif()

    #Thread support enabled (not the same as -lpthread)
    list(APPEND RTLTCP_LIBRARIES -pthread)
endif()

if (APPLE)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wc++11-extensions")
endif(APPLE)

SOAPY_SDR_MODULE_UTIL(
    TARGET rtltcpSupport
    SOURCES
        SoapyRTLTCP.hpp
        Registration.cpp
        Settings.cpp
        Streaming.cpp
    LIBRARIES
        ${RTLTCP_LIBRARIES}
        ${ATOMIC_LIBS}
)

#network libraries
if (WIN32)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    add_definitions(-D_WINSOCK_DEPRECATED_NO_WARNINGS)
    target_link_libraries(rtltcpSupport PRIVATE ws2_32)
endif (WIN32)

########################################################################
# uninstall target
########################################################################
add_custom_target(uninstall
    "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

