add_executable(first src/first.cpp)
target_link_libraries(first ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS first DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
