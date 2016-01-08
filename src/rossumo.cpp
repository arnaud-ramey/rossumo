/*!
  \file        rossumo.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/1/8

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

\class Rossumo
  The interface between "libsumo" and ROS.

\section Parameters
  - \b "foo"
        [string] (default: "bar")
        Description of the parameter.

\section Subscriptions
  - \b "/foo"
        [xxx]
        Descrption of the subscription

\section Publications
  - \b "~foo"
        [xxx]
        Descrption of the publication
 */
//#include <rossumo/rossumo.h>
//int main(int argc, char **argv) {
//  ros::init(argc, argv, "rossumo");
//  Rossumo sumo;
//  ros::Rate rate(100);
//  while(ros::ok()) {
//    sumo.spinOnce();
//    ros::spinOnce();
//    rate.sleep();
//  }
//  return 0;
//}

/*
    Copyright (C) 2014 Parrot SA

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Parrot nor the names
      of its contributors may be used to endorse or promote products
      derived from this software without specific prior written
      permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
    OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.
*/
/**
 * @file JumpingSumoPiloting.c
 * @brief This file contains sources about basic arsdk example sending commands to a JumpingSumo for piloting it and make it jump it and receiving its battery level
 * @date 15/01/2015
 */

/*****************************************
 *
 *             include file :
 *
 *****************************************/

extern "C" {
#include <stdlib.h>
#include <curses.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#include <libARSAL/ARSAL.h>
#include <libARController/ARController.h>
#include <libARDiscovery/ARDiscovery.h>
}

//#include "JumpingSumoPiloting.h"
//#include "ihm.h"

#define TAG "SDKExample"

#define ERROR_STR_LENGTH 2048

#define JS_IP_ADDRESS "192.168.2.1"
#define JS_DISCOVERY_PORT 44444

#define DISPLAY_WITH_MPLAYER 1

//#define IHM

int gIHMRun = 1;
char gErrorStr[ERROR_STR_LENGTH];
//IHM_t *ihm = NULL;

FILE *videoOut = NULL;
int writeImgs = 0;
int frameNb = 0;
ARSAL_Sem_t stateSem;

////////////////////////////////////////////////////////////////////////////////

int main (int argc, char *argv[])
{
  // local declarations
  int failed = 0;
  ARDISCOVERY_Device_t *device = NULL;
  ARCONTROLLER_Device_t *deviceController = NULL;
  eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
  eARCONTROLLER_DEVICE_STATE deviceState = ARCONTROLLER_DEVICE_STATE_MAX;
  pid_t child = 0;
  ARSAL_Sem_Init (&(stateSem), 0, 0);

  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- Jumping Sumo Piloting --");

  if (failed) { printf("Fail at line %i\n", __LINE__); exit(-1); }
  if (DISPLAY_WITH_MPLAYER)
  {
    // fork the process to launch ffplay
    if ((child = fork()) == 0)
    {
      execlp("xterm", "xterm", "-e", "mplayer", "-demuxer",  "lavf", "video_fifo.mjpg", "-benchmark", "-really-quiet", NULL);
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Missing mplayer, you will not see the video. Please install mplayer and xterm.");
      return -1;
    }
  }
  else
  {
    // create the video folder to store video images
    char answer = 'N';
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Do you want to write image files on your file system ? You should have at least 50Mb. Y or N");
    scanf("%c", &answer);
    if (answer == 'Y' || answer == 'y')
    {
      ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "You choose to write image files.");
      writeImgs = 1;
      mkdir("video", S_IRWXU);
    }
    else
    {
      ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "You did not choose to write image files.");
    }
  }

  if (DISPLAY_WITH_MPLAYER)
  {
    videoOut = fopen("./video_fifo.mjpg", "w");
  }

#ifdef IHM
  ihm = IHM_New (&onInputEvent);
  if (ihm != NULL)
  {
    gErrorStr[0] = '\0';
    ARSAL_Print_SetCallback (customPrintCallback); //use a custom callback to print, for not disturb ncurses IHM

    IHM_PrintHeader (ihm, "-- Jumping Sumo Piloting --");
  }
  else
  {
    ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "Creation of IHM failed.");
    failed = 1;
  }
#endif

  // create a discovery device
  if (failed) { printf("Fail at line %i\n", __LINE__); exit(-1); }
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- init discovey device ... ");
  eARDISCOVERY_ERROR errorDiscovery = ARDISCOVERY_OK;

  device = ARDISCOVERY_Device_New (&errorDiscovery);

  if (errorDiscovery == ARDISCOVERY_OK)
  {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - ARDISCOVERY_Device_InitWifi ...");
    // create a JumpingSumo discovery device (ARDISCOVERY_PRODUCT_JS)
    errorDiscovery = ARDISCOVERY_Device_InitWifi (device, ARDISCOVERY_PRODUCT_JS, "JS", JS_IP_ADDRESS, JS_DISCOVERY_PORT);

    if (errorDiscovery != ARDISCOVERY_OK)
    {
      failed = 1;
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Discovery error :%s", ARDISCOVERY_Error_ToString(errorDiscovery));
    }
  }
  else
  {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Discovery error :%s", ARDISCOVERY_Error_ToString(errorDiscovery));
    failed = 1;
  }

  // create a device controller
  if (failed) { printf("Fail at line %i\n", __LINE__); exit(-1); }
  deviceController = ARCONTROLLER_Device_New (device, &error);

  if (error != ARCONTROLLER_OK)
  {
    ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "Creation of deviceController failed.");
    failed = 1;
  }
  else
  {
    //IHM_setCustomData(ihm, deviceController);
  }

  if (failed) { printf("Fail at line %i\n", __LINE__); exit(-1); }
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- delete discovey device ... ");
  ARDISCOVERY_Device_Delete (&device);

  // add the state change callback to be informed when the device controller starts, stops...
  if (failed) { printf("Fail at line %i\n", __LINE__); exit(-1); }
  //error = ARCONTROLLER_Device_AddStateChangedCallback (deviceController, stateChanged, deviceController);

  if (error != ARCONTROLLER_OK)
  {
    ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "add State callback failed.");
    failed = 1;
  }

  // add the command received callback to be informed when a command has been received from the device
  if (failed) { printf("Fail at line %i\n", __LINE__); exit(-1); }
  //error = ARCONTROLLER_Device_AddCommandReceivedCallback (deviceController, commandReceived, deviceController);

  if (error != ARCONTROLLER_OK)
  {
    ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "add callback failed.");
    failed = 1;
  }

  // add the frame received callback to be informed when a streaming frame has been received from the device
  if (failed) { printf("Fail at line %i\n", __LINE__); exit(-1); }
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- set Video callback ... ");
  //error = ARCONTROLLER_Device_SetVideoStreamCallbacks (deviceController, decoderConfigCallback, didReceiveFrameCallback, NULL , NULL);

  if (error != ARCONTROLLER_OK)
  {
    failed = 1;
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%", ARCONTROLLER_Error_ToString(error));
  }

  if (failed) { printf("Fail at line %i\n", __LINE__); exit(-1); }
  //IHM_PrintInfo(ihm, "Connecting ...");
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Connecting ...");
  error = ARCONTROLLER_Device_Start (deviceController);

  if (error != ARCONTROLLER_OK)
  {
    failed = 1;
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(error));
  }

  if (failed) { printf("Fail at line %i\n", __LINE__); exit(-1); }
  // wait state update update
  ARSAL_Sem_Wait (&(stateSem));

  deviceState = ARCONTROLLER_Device_GetState (deviceController, &error);

  if ((error != ARCONTROLLER_OK) || (deviceState != ARCONTROLLER_DEVICE_STATE_RUNNING))
  {
    failed = 1;
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- deviceState :%d", deviceState);
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(error));
  }

  // send the command that tells to the Jumping to begin its streaming
  if (failed) { printf("Fail at line %i\n", __LINE__); exit(-1); }
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- send StreamingVideoEnable ... ");
  error = deviceController->jumpingSumo->sendMediaStreamingVideoEnable (deviceController->jumpingSumo, 1);
  if (error != ARCONTROLLER_OK)
  {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(error));
    failed = 1;
  }

  if (failed) { printf("Fail at line %i\n", __LINE__); exit(-1); }
  //IHM_PrintInfo(ihm, "Running ... (Arrow keys to move ; Spacebar to jump ; 'q' to quit)");

#ifdef IHM
  while (gIHMRun)
  {
    usleep(50);
  }
#else
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- sleep 20 ... ");
  sleep(20);
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- sleep end ... ");
#endif

#ifdef IHM
  IHM_Delete (&ihm);
#endif

  // we are here because of a disconnection or user has quit IHM, so safely delete everything
  if (deviceController != NULL)
  {
    deviceState = ARCONTROLLER_Device_GetState (deviceController, &error);
    if ((error == ARCONTROLLER_OK) && (deviceState != ARCONTROLLER_DEVICE_STATE_STOPPED))
    {
      //IHM_PrintInfo(ihm, "Disconnecting ...");
      ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Disconnecting ...");

      error = ARCONTROLLER_Device_Stop (deviceController);

      if (error == ARCONTROLLER_OK)
      {
        // wait state update update
        ARSAL_Sem_Wait (&(stateSem));
      }
    }

    //IHM_PrintInfo(ihm, "");
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "ARCONTROLLER_Device_Delete ...");
    ARCONTROLLER_Device_Delete (&deviceController);

    if (DISPLAY_WITH_MPLAYER)
    {
      fflush (videoOut);
      fclose (videoOut);

      if (child > 0)
      {
        kill(child, SIGKILL);
      }
    }
  }

  ARSAL_Sem_Destroy (&(stateSem));

  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- END --");

  return EXIT_SUCCESS;
}

/*****************************************
 *
 *             private implementation:
 *
 ****************************************/

////////////////////////////////////////////////////////////////////////////////

// called when the state of the device controller has changed
void stateChanged (eARCONTROLLER_DEVICE_STATE newState, eARCONTROLLER_ERROR error, void *customData)
{
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - stateChanged newState: %d .....", newState);

  switch (newState)
  {
    case ARCONTROLLER_DEVICE_STATE_STOPPED:
      ARSAL_Sem_Post (&(stateSem));
      //stop
      gIHMRun = 0;

      break;

    case ARCONTROLLER_DEVICE_STATE_RUNNING:
      ARSAL_Sem_Post (&(stateSem));
      break;

    default:
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////

// called when a command has been received from the drone
void commandReceived (eARCONTROLLER_DICTIONARY_KEY commandKey,
                      ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary,
                      void *customData)
{
  printf("commandReceived(%i)\n", commandKey);
  ARCONTROLLER_Device_t *deviceController = (ARCONTROLLER_Device_t *) customData;
  eARCONTROLLER_ERROR error = ARCONTROLLER_OK;

  if (deviceController != NULL)
  {
    // if the command received is a battery state changed
    if (commandKey == ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED)
    {
      ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
      ARCONTROLLER_DICTIONARY_ELEMENT_t *singleElement = NULL;

      if (elementDictionary != NULL)
      {
        // get the command received in the device controller
        HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, singleElement);

        if (singleElement != NULL)
        {
          // get the value
          HASH_FIND_STR (singleElement->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED_PERCENT, arg);

          if (arg != NULL)
          {
            // update UI
            //batteryStateChanged (arg->value.U8);
          }
          else
          {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "arg is NULL");
          }
        }
        else
        {
          ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "singleElement is NULL");
        }
      }
      else
      {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "elements is NULL");
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

void batteryStateChanged (uint8_t percent)
{
  printf("batteryStateChanged(%i)\n", percent);
  // callback of changing of battery level

  //  if (ihm != NULL)
  //  {
  //    IHM_PrintBattery (ihm, percent);
  //  }

}

////////////////////////////////////////////////////////////////////////////////

eARCONTROLLER_ERROR decoderConfigCallback (ARCONTROLLER_Stream_Codec_t codec, void *customData)
{
  printf("decoderConfigCallback()\n");
  ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "decoderConfigCallback codec.type :%d", codec.type);

  return ARCONTROLLER_OK;
}

////////////////////////////////////////////////////////////////////////////////

eARCONTROLLER_ERROR didReceiveFrameCallback (ARCONTROLLER_Frame_t *frame, void *customData)
{
  printf("decoderConfigCallback()\n");
  if (videoOut != NULL)
  {
    if (frame != NULL)
    {
      if (DISPLAY_WITH_MPLAYER)
      {
        fwrite(frame->data, frame->used, 1, videoOut);

        fflush (videoOut);
      }
      else if (writeImgs)
      {
        char filename[20];
        snprintf(filename, sizeof(filename), "video/img_%d.jpg", frameNb);

        frameNb++;
        FILE *img = fopen(filename, "w");
        fwrite(frame, frame->used, 1, img);
        fclose(img);
      }
    }
    else
    {
      ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "frame is NULL.");
    }
  }
  else
  {
    ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "videoOut is NULL.");
  }

  return ARCONTROLLER_OK;
}

////////////////////////////////////////////////////////////////////////////////

// IHM callbacks:

//void onInputEvent (eIHM_INPUT_EVENT event, void *customData)
//{
//  // Manage IHM input events
//  ARCONTROLLER_Device_t *deviceController = (ARCONTROLLER_Device_t *)customData;
//  eARCONTROLLER_ERROR error = ARCONTROLLER_OK;

//  switch (event)
//  {
//    case IHM_INPUT_EVENT_EXIT:
//      IHM_PrintInfo(ihm, "IHM_INPUT_EVENT_EXIT ...");
//      gIHMRun = 0;
//      break;
//    case IHM_INPUT_EVENT_JUMP:
//      if(deviceController != NULL)
//      {
//        // send a jump command to the JS
//        error = deviceController->jumpingSumo->sendAnimationsJump (deviceController->jumpingSumo, ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_JUMP_TYPE_HIGH);
//        IHM_PrintInfo(ihm, "IHM_INPUT_EVENT_JUMP ...");
//      }
//      break;
//    case IHM_INPUT_EVENT_FORWARD:
//      if(deviceController != NULL)
//      {
//        // set the flag and speed value of the piloting command
//        error = deviceController->jumpingSumo->setPilotingPCMDFlag (deviceController->jumpingSumo, 1);
//        error = deviceController->jumpingSumo->setPilotingPCMDSpeed (deviceController->jumpingSumo, 50);
//      }
//      break;
//    case IHM_INPUT_EVENT_BACK:
//      if(deviceController != NULL)
//      {
//        error = deviceController->jumpingSumo->setPilotingPCMDFlag (deviceController->jumpingSumo, 1);
//        error = deviceController->jumpingSumo->setPilotingPCMDSpeed (deviceController->jumpingSumo, -50);
//      }
//      break;
//    case IHM_INPUT_EVENT_RIGHT:
//      if(deviceController != NULL)
//      {
//        error = deviceController->jumpingSumo->setPilotingPCMDFlag (deviceController->jumpingSumo, 1);
//        error = deviceController->jumpingSumo->setPilotingPCMDTurn (deviceController->jumpingSumo, 50);
//      }
//      break;
//    case IHM_INPUT_EVENT_LEFT:
//      if(deviceController != NULL)
//      {
//        error = deviceController->jumpingSumo->setPilotingPCMDFlag (deviceController->jumpingSumo, 1);
//        error = deviceController->jumpingSumo->setPilotingPCMDTurn (deviceController->jumpingSumo, -50);
//      }
//      break;
//    case IHM_INPUT_EVENT_NONE:
//      if(deviceController != NULL)
//      {
//        error = deviceController->jumpingSumo->setPilotingPCMDFlag (deviceController->jumpingSumo, 0);
//        error = deviceController->jumpingSumo->setPilotingPCMDSpeed (deviceController->jumpingSumo, 0);
//        error = deviceController->jumpingSumo->setPilotingPCMDTurn (deviceController->jumpingSumo, 0);
//      }
//      break;
//    default:
//      break;

//  }

//  // This should be improved, here it just displays that one error occured
//  if (error != ARCONTROLLER_OK)
//  {
//    IHM_PrintInfo(ihm, "Error sending an event");
//  }
//}

//int customPrintCallback (eARSAL_PRINT_LEVEL level, const char *tag, const char *format, va_list va)
//{
//  // Custom callback used when ncurses is runing for not disturb the IHM

//  if ((level == ARSAL_PRINT_ERROR) && (strcmp(TAG, tag) == 0))
//  {
//    // Save the last Error
//    vsnprintf(gErrorStr, (ERROR_STR_LENGTH - 1), format, va);
//    gErrorStr[ERROR_STR_LENGTH - 1] = '\0';
//  }

//  return 1;
//}
