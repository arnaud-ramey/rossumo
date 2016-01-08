# cc -c -o obj/JumpingSumoPiloting.o JumpingSumoPiloting.c -I./ -I /home/arnaud/sumo/out/Unix-base/staging/usr/include/
#[ -d obj ] || mkdir obj
#cc -c -o obj/ihm.o ihm.c -I./ -I /home/arnaud/sumo/out/Unix-base/staging/usr/include/
#cc -o JumpingSumoPiloting obj/JumpingSumoPiloting.o obj/ihm.o -I./ -I /home/arnaud/sumo/out/Unix-base/staging/usr/include/ -L/home/arnaud/sumo/out/Unix-base/staging/usr/lib/ -larsal -larcontroller -lardiscovery -lncurses -Wl,-rpath=/home/arnaud/sumo/out/Unix-base/staging/usr/lib/

add_executable(rossumo.exe
  rossumo.cpp
  ../include/rossumo/rossumo.h
)
target_link_libraries(rossumo.exe  ${catkin_LIBRARIES}  arsal  arcontroller  ardiscovery  ncurses)
