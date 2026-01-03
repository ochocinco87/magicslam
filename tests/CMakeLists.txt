add_executable(test_types test_types.cpp)
target_link_libraries(test_types PRIVATE magicslam)
add_test(NAME test_types COMMAND test_types)

add_executable(test_imu test_imu.cpp)
target_link_libraries(test_imu PRIVATE magicslam)
add_test(NAME test_imu COMMAND test_imu)

add_executable(test_features test_features.cpp)
target_link_libraries(test_features PRIVATE magicslam)
add_test(NAME test_features COMMAND test_features)
