add_executable(pick_place "${CMAKE_CURRENT_SOURCE_DIR}/src/pick_place.cpp")
target_link_libraries(pick_place ${catkin_LIBRARIES})
install(TARGETS pick_place DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
