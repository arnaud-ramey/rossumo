/*!
  \file        sumo_prompt.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/1/23

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

\todo Description of the file
 */
#include <rossumo/light_sumo.h>
#include <iostream>

void print_help(int /*argc*/, char** argv) {
  printf("Synopsis: %s CMD PARAM\n", argv[0]);
  printf("'vol':  set_volume                    vol(0~100)\n");
  printf("'sou':  play_sound                    sound(string)\n");
  printf("'the':  set_audiotheme                theme:'insect|monster|robot'\n");
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  if (argc < 2) {
    print_help(argc, argv);
    return -1;
  }
  LightSumo sumo;
  if (!sumo.connect()) {
    return -1;
  }

  std::string choice = argv[1];
  unsigned int nparams = argc - 2;
  double param1 = (argc >= 3 ? atof(argv[2]) : -1);
  double param2 = (argc >= 4 ? atof(argv[3]) : -1);
  double param3 = (argc >= 5 ? atof(argv[4]) : -1);
  double param4 = (argc >= 6 ? atof(argv[5]) : -1);
  double param5 = (argc >= 7 ? atof(argv[6]) : -1);
  if (choice == "vol" && nparams == 1)
    printf("retval:%i\n", sumo.set_volume(param1));
  else if (choice == "sou" && nparams == 1)
    printf("retval:%i\n", sumo.play_sound(argv[2]));
  else if (choice == "the" && nparams == 1)
    printf("retval:%i\n", sumo.set_audiotheme(argv[2]));
  else // nothing done
    print_help(argc, argv);

  return 0;
}



