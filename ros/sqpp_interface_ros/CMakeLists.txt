set(MOVEIT_LIB_NAME moveit_sqpp_interface_ros)

add_library(${MOVEIT_LIB_NAME} src/sqpp_interface_ros.cpp src/rosconversions.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# TODO: Build planning server
#add_executable(moveit_sqpp_planner src/sqpp_planner.cpp)
#target_link_libraries(moveit_sqpp_planner ${MOVEIT_LIB_NAME})

add_library(moveit_sqpp_planner_plugin src/sqpp_plugin.cpp)
target_link_libraries(moveit_sqpp_planner_plugin ${MOVEIT_LIB_NAME})

install(TARGETS ${MOVEIT_LIB_NAME} moveit_sqpp_planner_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
