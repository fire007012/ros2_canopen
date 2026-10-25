ament_add_gtest(test_driver_component
test_driver_component.cpp
)
ament_target_dependencies(test_driver_component
  ${dependencies}
)
target_include_directories(test_driver_component PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)

ament_add_gtest(test_command_session test_command_session.cpp)
ament_target_dependencies(test_command_session ${dependencies})
target_include_directories(test_command_session PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include/)

target_link_libraries(test_command_session lely_motion_controller_bridge)
ament_add_gtest(test_managed_commands test_managed_commands.cpp)
ament_target_dependencies(test_managed_commands ${dependencies})
target_include_directories(test_managed_commands PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/)
target_link_libraries(test_managed_commands node_canopen_cia402_driver)
