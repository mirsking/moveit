set(MOVEIT_LIB_NAME moveit_collision_detection_physx)

add_library(${MOVEIT_LIB_NAME}
  src/collision_common.cpp
  src/collision_robot_physx.cpp
  src/collision_world_physx.cpp
)

target_link_libraries(${MOVEIT_LIB_NAME} moveit_collision_detection ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${LIBFCL_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(moveit_resources REQUIRED)
  message(moveit_resources: ${moveit_resources_INCLUDE_DIRS})
  include_directories(${moveit_resources_INCLUDE_DIRS})

  catkin_add_gtest(test_physx_collision_detection test/test_physx_collision_detection.cpp)
  target_link_libraries(test_physx_collision_detection  ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})
endif()
