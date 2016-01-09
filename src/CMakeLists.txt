add_executable(light_sumo_test.exe
  light_sumo_test.cpp
  ../include/rossumo/light_sumo.h
)
target_link_libraries(light_sumo_test.exe
  ${catkin_LIBRARIES} ${ARDRONESDK3_LIBRARIES} ${OpenCV_LIBRARIES}
)

add_executable(rossumo.exe
  rossumo.cpp
  ../include/rossumo/light_sumo.h
)
target_link_libraries(rossumo.exe
  ${catkin_LIBRARIES} ${ARDRONESDK3_LIBRARIES} ${OpenCV_LIBRARIES}
)
