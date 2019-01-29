cmake_minimum_required(VERSION 2.8)

project(n300_txrx_pulse_test)

# Tell CMake to run moc when necessary:
set(CMAKE_AUTOMOC ON)
# As moc files are generated in the binary dir, tell CMake
# to always look for includes there:
set(CMAKE_INCLUDE_CURRENT_DIR ON)


# Add compiler flags for building executables (-fPIE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


aux_source_directory(./source SRC_LIST)

FILE(GLOB_RECURSE INC_LIST "include/*.h" "include/*.hpp")



### Set up build environment ##################################################
# Choose a static or shared-library build (shared is default, and static will
# probably need some special care!)
# Set this to ON in order to link a static build of UHD:
option(UHD_USE_STATIC_LIBS OFF)

# This example also requires Boost:
set(BOOST_REQUIRED_COMPONENTS
    program_options
    system
    thread
    filesystem
)
if(MSVC)
    set(BOOST_ALL_DYN_LINK "${BOOST_ALL_DYN_LINK}" CACHE BOOL "boost enable dynamic linking")
    if(BOOST_ALL_DYN_LINK)
        add_definitions(-DBOOST_ALL_DYN_LINK) #setup boost auto-linking in msvc
    else(BOOST_ALL_DYN_LINK)
        set(BOOST_REQUIRED_COMPONENTS) #empty components list for static link
    endif(BOOST_ALL_DYN_LINK)
endif(MSVC)
find_package(Boost "1.46" REQUIRED ${BOOST_REQUIRED_COMPONENTS})

# To add UHD as a dependency to this project, add a line such as this:
find_package(UHD "3.13" REQUIRED)
#find_package(UHD "4.0.0" REQUIRED)
# The version in  ^^^^^  here is a minimum version.
# To specify an exact version:
#find_package(UHD 3.8.1 EXACT REQUIRED)


message("UHD include dir: ${UHD_INCLUDE_DIRS}")
message("UHD libraries: ${UHD_LIBRARIES}")


### Configure Compiler ########################################################
include_directories(
    ${Boost_INCLUDE_DIRS}
    ${UHD_INCLUDE_DIRS}
    ./include
)

link_directories(${Boost_LIBRARY_DIRS})

### Make the executable #######################################################
add_executable(${PROJECT_NAME} ${SRC_LIST} ${INC_LIST})
message("Include list:" ${INC_LIST})
message("Src list:" ${SRC_LIST})
SET(CMAKE_BUILD_TYPE "Release")
MESSAGE(STATUS "******************************************************************************")
MESSAGE(STATUS "* NOTE: When building your own app, you probably need all kinds of different  ")
MESSAGE(STATUS "* compiler flags. This is just an example, so it's unlikely these settings    ")
MESSAGE(STATUS "* exactly match what you require. Make sure to double-check compiler and     ")
MESSAGE(STATUS "* linker flags to make sure your specific requirements are included.          ")
MESSAGE(STATUS "******************************************************************************")

# Shared library case: All we need to do is link against the library, and
# anything else we need (in this case, some Boost libraries):
if(NOT UHD_USE_STATIC_LIBS)
    message(STATUS "Linking against shared UHD library.")
    target_link_libraries(${PROJECT_NAME} ${UHD_LIBRARIES} ${Boost_LIBRARIES} pthread)
# Shared library case: All we need to do is link against the library, and
# anything else we need (in this case, some Boost libraries):
else(NOT UHD_USE_STATIC_LIBS)
    message(STATUS "Linking against static UHD library.")
    target_link_libraries(${PROJECT_NAME}
        # We could use ${UHD_LIBRARIES}, but linking requires some extra flags,
        # so we use this convenience variable provided to us
        ${UHD_STATIC_LIB_LINK_FLAG}
        # Also, when linking statically, we need to pull in all the deps for
        # UHD as well, because the dependencies don't get resolved automatically
        ${UHD_STATIC_LIB_DEPS}
    )
endif(NOT UHD_USE_STATIC_LIBS)

### Once it's built... ########################################################
# Here, you would have commands to install your program.
# We will skip these in this example.
install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/)
