cmake_minimum_required(VERSION 3.10)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(PROJECT_NAME "yeastcpppathplannertrajectoryfollower")
project(${PROJECT_NAME})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi -Wno-stringop-overflow -Wno-array-bounds -g)
endif()

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(wpilib REQUIRED)
find_package(wpilibNewCommands REQUIRED)

# Separate yeast sources from vendored pathplanner sources
file(GLOB YEAST_SOURCES
    "src/*.cpp"
)

file(GLOB_RECURSE PATHPLANNER_SOURCES
    "src/pathplanner/*.cpp"
)

# Add the library source files
add_library(${PROJECT_NAME}
    ${YEAST_SOURCES}
    ${PATHPLANNER_SOURCES}
)

# Disable warnings on vendored pathplanner source files
set_source_files_properties(${PATHPLANNER_SOURCES} PROPERTIES
    COMPILE_OPTIONS "-w"
)

# Treat warnings as errors for yeast source files
set_source_files_properties(${YEAST_SOURCES} PROPERTIES
    COMPILE_OPTIONS "-Werror"
)

# Yeast includes are PUBLIC, pathplanner headers are SYSTEM to suppress warnings
target_include_directories(${PROJECT_NAME} PUBLIC
    ../yeastcpp/include
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}
    wpiutil
    wpilibc
    wpilibNewCommands
    hal
)

# Install the library and header files
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

install(DIRECTORY "${CMAKE_SOURCE_DIR}/include/" # source directory
        DESTINATION "include" # target directory
        FILES_MATCHING # install only matched files
        PATTERN "*.hpp" # select header files
)