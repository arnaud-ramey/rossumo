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

*/
/**
 * @file JumpingSumoPiloting.c
 * @brief This file contains sources about basic arsdk example sending commands to a JumpingSumo for piloting it and make it jump it and receiving its battery level
 * @date 15/01/2015
 */

extern "C" {
#include <libARSAL/ARSAL.h>
#include <libARController/ARController.h>
#include <libARDiscovery/ARDiscovery.h>
}

#include "opencv2/highgui/highgui.hpp"
#include <boost/thread/mutex.hpp>

#define TAG "rossumo"
#define JS_IP_ADDRESS "192.168.2.1"
#define JS_DISCOVERY_PORT 44444

ARSAL_Sem_t stateSem;
cv::Mat pic;
bool has_new_pic = false;
boost::mutex pic_mutex;

////////////////////////////////////////////////////////////////////////////////

// called when the state of the device controller has changed
void stateChanged (eARCONTROLLER_DEVICE_STATE newState, eARCONTROLLER_ERROR /*error*/, void */*customData*/)
{
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - stateChanged newState: %d .....", newState);

  switch (newState)
  {
    case ARCONTROLLER_DEVICE_STATE_STOPPED:
      ARSAL_Sem_Post (&(stateSem));
      //ros::shutdown();
      exit(-1);

      break;

    case ARCONTROLLER_DEVICE_STATE_RUNNING:
      ARSAL_Sem_Post (&(stateSem));
      break;

    default:
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////

//! callback of changing of battery level
void batteryStateChanged (uint8_t percent) {
  printf("batteryStateChanged(%i)\n", percent);
}

////////////////////////////////////////////////////////////////////////////////

// called when a command has been received from the drone
void commandReceived (eARCONTROLLER_DICTIONARY_KEY commandKey,
                      ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary,
                      void *customData)
{
  printf("commandReceived(%i)\n", commandKey);
  ARCONTROLLER_Device_t *deviceController = (ARCONTROLLER_Device_t *) customData;
  //eARCONTROLLER_ERROR error = ARCONTROLLER_OK;

  if (deviceController == NULL)
    return;
  // if the command received is a battery state changed
  if (commandKey == ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED)
  {
    if (elementDictionary == NULL)
      return;
    // get the command received in the device controller
    ARCONTROLLER_DICTIONARY_ELEMENT_t *singleElement = NULL;
    HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, singleElement);
    if (singleElement == NULL)
      return;
    // get the value
    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    HASH_FIND_STR (singleElement->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED_PERCENT, arg);
    if (arg == NULL)
      return;
    // update UI
    batteryStateChanged (arg->value.U8);
  }
} // end commandReceived();

////////////////////////////////////////////////////////////////////////////////

eARCONTROLLER_ERROR decoderConfigCallback (ARCONTROLLER_Stream_Codec_t codec, void */*customData*/)
{
  printf("decoderConfigCallback()\n");
  ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "decoderConfigCallback codec.type :%d", codec.type);
  return ARCONTROLLER_OK;
} // end decoderConfigCallback()

////////////////////////////////////////////////////////////////////////////////

eARCONTROLLER_ERROR didReceiveFrameCallback (ARCONTROLLER_Frame_t *frame, void */*customData*/)
{
  //ROS_INFO_THROTTLE(1, "didReceiveFrameCallback(%i)", frame->used);
  std::vector<uchar> ans_vector (frame->data, frame->data + frame->used);
  pic_mutex.lock();
  pic = cv::imdecode(cv::Mat (ans_vector), -1);
  has_new_pic = true;
  pic_mutex.unlock();
  return ARCONTROLLER_OK;
} // end didReceiveFrameCallback()

////////////////////////////////////////////////////////////////////////////////

int main (int argc, char *argv[]) {
  //ros::init(argc, argv, "rossumo");
  //ros::NodeHandle nh_public;
  cv::namedWindow("sumo");
  // local declarations
  ARDISCOVERY_Device_t *device = NULL;
  ARCONTROLLER_Device_t *deviceController = NULL;
  eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
  eARCONTROLLER_DEVICE_STATE deviceState = ARCONTROLLER_DEVICE_STATE_MAX;
  ARSAL_Sem_Init (&(stateSem), 0, 0);
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- Jumping Sumo Piloting --");

  // create a discovery device
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- init discovey device ... ");
  eARDISCOVERY_ERROR errorDiscovery = ARDISCOVERY_OK;

  device = ARDISCOVERY_Device_New (&errorDiscovery);
  if (errorDiscovery == ARDISCOVERY_OK) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - ARDISCOVERY_Device_InitWifi ...");
    // create a JumpingSumo discovery device (ARDISCOVERY_PRODUCT_JS)
    errorDiscovery = ARDISCOVERY_Device_InitWifi (device, ARDISCOVERY_PRODUCT_JS, "JS", JS_IP_ADDRESS, JS_DISCOVERY_PORT);
    if (errorDiscovery != ARDISCOVERY_OK) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Discovery error :%s", ARDISCOVERY_Error_ToString(errorDiscovery));
      printf("Fail at line %i\n", __LINE__); exit(-1);
    }
  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Discovery error :%s", ARDISCOVERY_Error_ToString(errorDiscovery));
    printf("Fail at line %i\n", __LINE__); exit(-1);
  }

  // create a device controller
  deviceController = ARCONTROLLER_Device_New (device, &error);
  if (error != ARCONTROLLER_OK) {
    ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "Creation of deviceController failed.");
    printf("Fail at line %i\n", __LINE__); exit(-1);
  }

  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- delete discovey device ... ");
  ARDISCOVERY_Device_Delete (&device);

  // add the state change callback to be informed when the device controller starts, stops...
  error = ARCONTROLLER_Device_AddStateChangedCallback (deviceController, stateChanged, deviceController);
  if (error != ARCONTROLLER_OK) {
    ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "add State callback failed.");
    printf("Fail at line %i\n", __LINE__); exit(-1);
  }

  // add the command received callback to be informed when a command has been received from the device
  error = ARCONTROLLER_Device_AddCommandReceivedCallback (deviceController, commandReceived, deviceController);
  if (error != ARCONTROLLER_OK) {
    ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "add callback failed.");
    printf("Fail at line %i\n", __LINE__); exit(-1);
  }

  // add the frame received callback to be informed when a streaming frame has been received from the device
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- set Video callback ... ");
  error = ARCONTROLLER_Device_SetVideoStreamCallbacks (deviceController, decoderConfigCallback, didReceiveFrameCallback, NULL , NULL);
  if (error != ARCONTROLLER_OK) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%", ARCONTROLLER_Error_ToString(error));
    printf("Fail at line %i\n", __LINE__); exit(-1);
  }

  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Connecting ...");
  error = ARCONTROLLER_Device_Start (deviceController);
  if (error != ARCONTROLLER_OK) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(error));
    printf("Fail at line %i\n", __LINE__); exit(-1);
  }

  // wait state update update
  ARSAL_Sem_Wait (&(stateSem));

  deviceState = ARCONTROLLER_Device_GetState (deviceController, &error);
  if ((error != ARCONTROLLER_OK) || (deviceState != ARCONTROLLER_DEVICE_STATE_RUNNING)) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- deviceState :%d", deviceState);
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(error));
    printf("Fail at line %i\n", __LINE__); exit(-1);
  }

  // send the command that tells to the Jumping to begin its streaming
  ARCONTROLLER_FEATURE_JumpingSumo_t* js = deviceController->jumpingSumo;
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- send StreamingVideoEnable ... ");
  error = js->sendMediaStreamingVideoEnable (js, 1);
  if (error != ARCONTROLLER_OK) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(error));
    printf("Fail at line %i\n", __LINE__); exit(-1);
  }

  //while(ros::ok()) {
  while(true) {
    //ros::spinOnce();
    pic_mutex.lock();
    if (pic.empty())
      printf("pic empty!\n");
    else if (has_new_pic)
      cv::imshow("sumo", pic);
    has_new_pic = false;
    pic_mutex.unlock();
    char c = cv::waitKey(50);
    // Commands in Sources/ARCONTROLLER_Feature.h - line 1405
    if (c == 'i') { // go forward
      error = js->setPilotingPCMDFlag (js, 1);
      error = js->setPilotingPCMDSpeed (js, 50); // @param speed Speed value [-100:100]
    }
    else if (c == 'u') { // go diagonal
      error = js->setPilotingPCMDFlag (js, 1);
      error = js->setPilotingPCMDSpeed (js, 50);
      error = js->sendPilotingAddCapOffset(js, -.05); // @param offset Offset value in radians
      //error = js->setPilotingPCMDTurn (js, 50);
    }
    else if (c == 'o') { // go diagonal
      error = js->setPilotingPCMDFlag (js, 1);
      error = js->setPilotingPCMDSpeed (js, 50);
      error = js->sendPilotingAddCapOffset(js, .05); // @param offset Offset value in radians
    }
    else if (c == 'k') { // go backwards
      error = js->setPilotingPCMDFlag (js, 1);
      error = js->setPilotingPCMDSpeed (js, -50);
    }
    else if (c == 'j') { // turn left on place
      error = js->setPilotingPCMDFlag (js, 1);
      error = js->sendPilotingAddCapOffset(js, -.05); // @param offset Offset value in radians
    }
    else if (c == 'l') { // turn left on place
      error = js->sendPilotingAddCapOffset(js, .05); // @param offset Offset value in radians
    }
    else if (c == 'q' || c == 27) { //ESC
      break;
    }
    else { // stop robot
      error = js->setPilotingPCMDFlag (js, 0);
      error = js->setPilotingPCMDSpeed (js, 0);
      error = js->setPilotingPCMDTurn (js, 0);
    }
  }

  // we are here because of a disconnection or user has quit IHM, so safely delete everything
  if (deviceController != NULL) {
    deviceState = ARCONTROLLER_Device_GetState (deviceController, &error);
    if ((error == ARCONTROLLER_OK) && (deviceState != ARCONTROLLER_DEVICE_STATE_STOPPED)) {
      //IHM_PrintInfo(ihm, "Disconnecting ...");
      ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Disconnecting ...");

      error = ARCONTROLLER_Device_Stop (deviceController);
      if (error == ARCONTROLLER_OK) {
        // wait state update update
        ARSAL_Sem_Wait (&(stateSem));
      }
    }

    //IHM_PrintInfo(ihm, "");
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "ARCONTROLLER_Device_Delete ...");
    ARCONTROLLER_Device_Delete (&deviceController);
  }

  ARSAL_Sem_Destroy (&(stateSem));
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- END --");
  return EXIT_SUCCESS;
} // end main()
