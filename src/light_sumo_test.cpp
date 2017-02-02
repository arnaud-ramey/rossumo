/*!
  \file        light_sumo_test.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/1/9

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

A simple test for the LightSumo class.
 */
#include <rossumo/light_sumo.h>

int main (/*int argc, char **argv*/) {
  LightSumo sumo;
  cv::namedWindow("sumo");
  if (!sumo.connect())
    return -1;
  cv::Mat rgb;
  double v = 50, w = 10;

  unsigned int last_pix_idx = -1;
  while(true) {

    if (sumo.get_pic_idx() != last_pix_idx) {
      sumo.get_pic(rgb);
      if (rgb.empty())
        printf("pic empty!\n");
      else
        cv::imshow("sumo", rgb);
      last_pix_idx = sumo.get_pic_idx();
    }
    char c = cv::waitKey(50);
    // Commands in Sources/ARCONTROLLER_Feature.h - line 1405
    if (c == 'i')  // go forwardS
      sumo.set_speeds(v, 0);
    else if (c == 'u')  // go diagonal
      sumo.set_speeds(v, -w);
    else if (c == 'o') // go diagonal
      sumo.set_speeds(v, w);
    else if (c == 'k')  // go backwards
      sumo.set_speeds(-v, 0);
    else if (c == 'j')  // turn left on place
      sumo.set_speeds(0, -w);
    else if (c == 'l')  // turn left on place
      sumo.set_speeds(0, w);
    else if (c == ' ')
      sumo.high_jump();
    else if (c == '+')
      sumo.increase_volume();
    else if (c == '-')
      sumo.decrease_volume();
    else if (c == 'p') {
      if (rand()%2)
      sumo.set_posture_jumper();
      else
        sumo.set_posture_kicker();
    }
    else if (c == 'q' || c == 27)  //ESC
      break;
    else  // stop robot
      sumo.set_speeds(0, 0);
  }
  return EXIT_SUCCESS;
} // end main()

