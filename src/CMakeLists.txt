add_executable(light_sumo_test.exe light_sumo_test.cpp)
target_link_libraries(light_sumo_test.exe
  ${catkin_LIBRARIES} ${ARDRONESDK3_LIBRARIES})

add_executable(rossumo.exe
  rossumo.cpp
  ../include/rossumo/light_sumo.h
  ../include/rossumo/controller_key2string.h
  ../include/rossumo/device_state2string.h
)
target_link_libraries(rossumo.exe
  ${catkin_LIBRARIES} ${ARDRONESDK3_LIBRARIES})

add_executable(sumo_prompt.exe sumo_prompt.cpp)
target_link_libraries(sumo_prompt.exe
  ${catkin_LIBRARIES} ${ARDRONESDK3_LIBRARIES})

add_executable(sumo_teleop_joy.exe          sumo_teleop_joy.cpp)
target_link_libraries(sumo_teleop_joy.exe   ${catkin_LIBRARIES})
