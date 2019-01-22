add_executable(pickplace src/pickplace.cpp src/pickplacenode.cpp)
target_link_libraries(pickplace ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pickplace DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
