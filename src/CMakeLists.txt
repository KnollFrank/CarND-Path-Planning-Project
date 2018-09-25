project(Path_Planning_eclipse)

cmake_minimum_required (VERSION 3.5)
find_package(GTest REQUIRED)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

add_definitions(-std=c++17)
file(GLOB alglib_SRC
    "alglib/*.cpp"
)
set(sources main.cpp ${alglib_SRC})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
include_directories(${GTEST_INCLUDE_DIRS})
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS ${GTEST_LIBRARIES} pthread)
