/*!
  \file        device_state2string.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/1/18

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

Converts a device state (eARCONTROLLER_DEVICE_STATE)
into a human-readable string
 */
#ifndef DEVICE_STATE2STRING_H
#define DEVICE_STATE2STRING_H

extern "C" {
#include <libARController/ARCONTROLLER_Device.h>
}

inline const char* device_state2string(eARCONTROLLER_DEVICE_STATE & i) {
  switch (i) {
    case ARCONTROLLER_DEVICE_STATE_STOPPED:
      return "STOPPED";
    case ARCONTROLLER_DEVICE_STATE_STARTING:
      return "STARTING";
    case ARCONTROLLER_DEVICE_STATE_RUNNING:
      return "RUNNING";
    case ARCONTROLLER_DEVICE_STATE_PAUSED:
      return "PAUSED";
    case ARCONTROLLER_DEVICE_STATE_STOPPING:
      return "STOPPING";
    default:
      return "UNKOWNN";
  }
  return "UNKOWNN";
}

#endif // DEVICE_STATE2STRING_H

