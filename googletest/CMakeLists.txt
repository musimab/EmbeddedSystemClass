cmake_minimum_required(VERSION 3.9.1)
project(Tutorial)
# Locate GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(include)
# Link runTests with what we want to test and the GTest and pthread library
#add_executable(executeTests sqrt_test.cpp)

add_executable(executeTests
  src/StateTest.cpp
)

target_link_libraries(executeTests ${GTEST_LIBRARIES} pthread)



