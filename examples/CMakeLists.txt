add_executable(ar_demo ar_demo.cpp)
target_link_libraries(ar_demo PRIVATE magicslam)

add_executable(benchmark benchmark.cpp)
target_link_libraries(benchmark PRIVATE magicslam)
