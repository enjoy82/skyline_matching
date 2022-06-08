# 参考: https://qiita.com/shohirose/items/45fb49c6b429e8b204ac
cmake_minimum_required(VERSION 3.0.2)
project(run CXX)

add_subdirectory(skyline)

set(SRCS
    ${PROJECT_SOURCE_DIR}/main.cpp)
add_executable(run ${SRCS})
target_link_libraries(run skyline)
