add_executable(camera2world_converter.exe camera2world_converter.cpp )
target_link_libraries(camera2world_converter.exe ${catkin_LIBRARIES})

#~ add_executable(image_localizer.exe image_localizer.cpp )
#~ target_link_libraries(image_localizer.exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} v4l2)

add_executable(image2camera_tf.exe image2camera_tf.cpp )
target_link_libraries(image2camera_tf.exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
