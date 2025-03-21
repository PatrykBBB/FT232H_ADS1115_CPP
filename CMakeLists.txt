cmake_minimum_required(VERSION 3.1.0)
project(FT232H_ADS1115_CPP VERSION 1.0.0)

# Define version macros
add_definitions(-DFT_VER_MAJOR=1 -DFT_VER_MINOR=0 -DFT_VER_BUILD=0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wno-psabi")

#define names and files
set(APP_EXE_NAME example-cpp)
set(APP_EXE_SRC ./src/example.cpp)

# Specify library path
set(LIB_PATH /usr/local/lib)

# Specify include math
set(INC_PATH /usr/local/include)

include_directories(
    ${CMAKE_SOURCE_DIR}//inc
    ${CMAKE_SOURCE_DIR}//inc/ads1115
    ${INC_PATH}
)

#add source files
file(GLOB SOURCES 
    "src/*.cpp"
)

# Compile an executable
add_executable(${APP_EXE_NAME} ${APP_EXE_SRC} ${SOURCES})

# Link the shared library
target_link_libraries(${APP_EXE_NAME} 
    "${LIB_PATH}/libmpsse.so"
    "${LIB_PATH}/libftd2xx.so"
)

# Set the RPATH to find the shared library at runtime
set_target_properties(${APP_EXE_NAME} PROPERTIES
    LINK_DIRECTORIES ${LIB_PATH}
    INSTALL_RPATH ${LIB_PATH}
)