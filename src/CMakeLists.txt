SET(ROSSUMO_LIBS ${catkin_LIBRARIES} ${ARDRONESDK3_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(image_localizer.exe image_localizer.cpp )
target_link_libraries(image_localizer.exe ${ROSSUMO_LIBS} v4l2)

add_executable(image2camera_tf.exe image2camera_tf.cpp )
target_link_libraries(image2camera_tf.exe ${ROSSUMO_LIBS})

add_executable(light_sumo_test.exe light_sumo_test.cpp )
target_link_libraries(light_sumo_test.exe ${ROSSUMO_LIBS})

add_executable(rossumo.exe
  rossumo.cpp
  ../include/rossumo/light_sumo.h
  ../include/rossumo/controller_key2string.h
  ../include/rossumo/device_state2string.h
)
target_link_libraries(rossumo.exe ${ROSSUMO_LIBS})

add_executable(speed_calibrator.exe speed_calibrator.cpp )
target_link_libraries(speed_calibrator.exe ${ROSSUMO_LIBS})

add_executable(sumo_prompt.exe sumo_prompt.cpp )
target_link_libraries(sumo_prompt.exe ${ROSSUMO_LIBS})

add_executable(sumo_teleop_joy.exe          sumo_teleop_joy.cpp)
target_link_libraries(sumo_teleop_joy.exe   ${catkin_LIBRARIES})
