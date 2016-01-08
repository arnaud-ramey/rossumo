#ARDrone libs
SET(ARDRONESDK3_PATH /home/arnaud/sumo/out/Unix-base/staging/usr)
include_directories(${ARDRONESDK3_PATH}/include)
link_directories(${ARDRONESDK3_PATH}/lib/)

#file(GLOB_RECURSE ARDRONESDK3_HEADERS ${ARDRONESDK3_PATH}/*.h)
#message("ARDRONESDK3_HEADERS:${ARDRONESDK3_HEADERS}")
add_executable(rossumo.exe
  rossumo.cpp
  ../include/rossumo/rossumo.h
  ${ARDRONESDK3_HEADERS}
)
target_link_libraries(rossumo.exe  ${catkin_LIBRARIES}
  arsal  arcontroller  ardiscovery  ncurses ${OpenCV_LIBRARIES})
