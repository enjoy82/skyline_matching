file(GLOB_RECURSE SRCS
    *.cpp)
add_library(skyline STATIC ${SRCS})
target_include_directories(skyline PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(skyline PUBLIC ${PROJECT_SOURCE_DIR}/..)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(skyline ${OpenCV_LIBS})