set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin/)
add_executable(app_vio_evaluator vio_evaluator.cpp)
target_link_libraries(app_vio_evaluator autovision_vio_common)