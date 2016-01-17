/*!
  \file        light_sumo.h
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

\class LightSumo
  Wrapper of the ARDroneSDK3 sample "JumpingSumoPiloting.c" as a C++ lightweight class.

  The available functions in the SDK are in include/libARController/ARCONTROLLER_Feature.h
  from line 1156.
 */
#ifndef LIGHT_SUMO_H
#define LIGHT_SUMO_H

extern "C" {
#include <libARSAL/ARSAL.h>
#include <libARController/ARController.h>
#include <libARDiscovery/ARDiscovery.h>
}

#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/mutex.hpp>

#include "controller_key2string.h"

#define TAG "rossumo"
#define JS_IP_ADDRESS "192.168.2.1"
#define JS_DISCOVERY_PORT 44444

#define CHECK_ERROR(errorController) { \
  if (errorController != ARCONTROLLER_OK) { \
  ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%", ARCONTROLLER_Error_ToString(errorController)); \
  printf("Fail at line %i\n", __LINE__); \
  return false; \
  } \
  }

class LightSumo {
public:
  LightSumo() {
    // all initializations done in connect()
  }

  ~LightSumo() {
    // we are here because of a disconnection or user has quit IHM, so safely delete everything
    if (deviceController != NULL) {
      deviceState = ARCONTROLLER_Device_GetState (deviceController, &errorController);
      if ((errorController == ARCONTROLLER_OK) && (deviceState != ARCONTROLLER_DEVICE_STATE_STOPPED)) {
        //IHM_PrintInfo(ihm, "Disconnecting ...");
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Disconnecting ...");

        errorController = ARCONTROLLER_Device_Stop (deviceController);
        if (errorController == ARCONTROLLER_OK) {
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
  } // end dtor

  //////////////////////////////////////////////////////////////////////////////

  void set_speeds(double v, double w) {
    //printf("set_speeds(%i, %i)\n", v, w);
    //    errorController = js()->setPilotingPCMDFlag     (js(), 1);
    //    errorController = js()->setPilotingPCMDSpeed    (js(), v); // @param speed Speed value [-100:100]
    //    errorController = js()->setPilotingPCMDTurn     (js(), w); // @param turn Turn value. [-100:100]
    errorController = js()->setPilotingPCMD(js(), 1, v, w);
    //errorController = js()->sendPilotingAddCapOffset(js(), w); // @param offset Offset value in radians
  }

  //////////////////////////////////////////////////////////////////////////////
  /// postures
  //////////////////////////////////////////////////////////////////////////////

  inline unsigned int get_posture() const { return _posture; }
  void set_posture_standing() {
    errorController = js()->sendPilotingPosture(js(), ARCOMMANDS_JUMPINGSUMO_PILOTING_POSTURE_TYPE_STANDING);
  }
  void set_posture_jumper() {
    errorController = js()->sendPilotingPosture(js(), ARCOMMANDS_JUMPINGSUMO_PILOTING_POSTURE_TYPE_JUMPER);
  }
  void set_posture_kicker() {
    errorController = js()->sendPilotingPosture(js(), ARCOMMANDS_JUMPINGSUMO_PILOTING_POSTURE_TYPE_KICKER);
  }

  //////////////////////////////////////////////////////////////////////////////
  /// jumps
  //////////////////////////////////////////////////////////////////////////////

  void high_jump() {
    errorController = js()->sendAnimationsJump (js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_JUMP_TYPE_HIGH);
  }
  void long_jump() {
    errorController = js()->sendAnimationsJump (js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_JUMP_TYPE_LONG);
  }

  //////////////////////////////////////////////////////////////////////////////
  /// animations
  //////////////////////////////////////////////////////////////////////////////

  inline void anim_spin() {
    js()->sendAnimationsSimpleAnimation(js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_SIMPLEANIMATION_ID_SPIN);
  }
  inline void anim_tap() {
    js()->sendAnimationsSimpleAnimation(js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_SIMPLEANIMATION_ID_TAP);
  }
  inline void anim_slowshake() {
    js()->sendAnimationsSimpleAnimation(js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_SIMPLEANIMATION_ID_SLOWSHAKE);
  }
  inline void anim_metronome() {
    js()->sendAnimationsSimpleAnimation(js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_SIMPLEANIMATION_ID_METRONOME);
  }
  inline void anim_ondulation() {
    js()->sendAnimationsSimpleAnimation(js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_SIMPLEANIMATION_ID_ONDULATION);
  }
  inline void anim_spinJump() {
    js()->sendAnimationsSimpleAnimation(js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_SIMPLEANIMATION_ID_SPINJUMP);
  }
  inline void anim_spinToPosture() {
    js()->sendAnimationsSimpleAnimation(js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_SIMPLEANIMATION_ID_SPINTOPOSTURE);
  }
  inline void anim_spiral() {
    js()->sendAnimationsSimpleAnimation(js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_SIMPLEANIMATION_ID_SPIRAL);
  }
  inline void anim_slalom() {
    js()->sendAnimationsSimpleAnimation(js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_SIMPLEANIMATION_ID_SLALOM);
  }

  //////////////////////////////////////////////////////////////////////////////
  /// pictures from the camera
  //////////////////////////////////////////////////////////////////////////////

  inline void  enable_pic_decoding() { _pix_decoding_enabled = true; }
  inline void disable_pic_decoding() { _pix_decoding_enabled = false; }
  inline void get_pic(cv::Mat& out) {
    pic_mutex.lock();
    pic.copyTo(out);
    pic_mutex.unlock();
  }
  inline unsigned int get_pic_idx() const { return pic_idx; }

  //////////////////////////////////////////////////////////////////////////////

  bool connect() {
    // add the speed callback
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- set speed callback ... ");
    ARCOMMANDS_Decoder_SetJumpingSumoPilotingStateSpeedChangedCallback(speedChangedCb, this);

    device = NULL;
    deviceController = NULL;
    errorController = ARCONTROLLER_OK;
    deviceState = ARCONTROLLER_DEVICE_STATE_MAX;
    pic_idx = -1;
    enable_pic_decoding();
    ARSAL_Sem_Init (&(stateSem), 0, 0);
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- Jumping Sumo Piloting --");

    // create a discovery device
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- init discovey device ... ");
    eARDISCOVERY_ERROR errorDiscovery = ARDISCOVERY_OK;

    device = ARDISCOVERY_Device_New (&errorDiscovery);
    if (errorDiscovery != ARDISCOVERY_OK) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Discovery error :%s", ARDISCOVERY_Error_ToString(errorDiscovery));
      printf("Fail at line %i\n", __LINE__);
      return false;
    }

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - ARDISCOVERY_Device_InitWifi ...");
    // create a JumpingSumo discovery device (ARDISCOVERY_PRODUCT_JS)
    errorDiscovery = ARDISCOVERY_Device_InitWifi (device, ARDISCOVERY_PRODUCT_JS, "JS", JS_IP_ADDRESS, JS_DISCOVERY_PORT);
    if (errorDiscovery != ARDISCOVERY_OK) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Discovery error :%s", ARDISCOVERY_Error_ToString(errorDiscovery));
      printf("Fail at line %i\n", __LINE__);
      return false;
    }

    // create a device controller
    deviceController = ARCONTROLLER_Device_New (device, &errorController);
    if (errorController != ARCONTROLLER_OK) {
      ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "Creation of deviceController failed.");
      printf("Fail at line %i\n", __LINE__);
      return false;
    }

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- delete discovey device ... ");
    ARDISCOVERY_Device_Delete (&device);

    // add the state change callback to be informed when the device controller starts, stops...
    errorController = ARCONTROLLER_Device_AddStateChangedCallback (deviceController, stateChanged, this);
    if (errorController != ARCONTROLLER_OK) {
      ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "add State callback failed.");
      printf("Fail at line %i\n", __LINE__);
      return false;
    }

    // add the command received callback to be informed when a command has been received from the device
    errorController = ARCONTROLLER_Device_AddCommandReceivedCallback (deviceController, commandReceived, this);
    if (errorController != ARCONTROLLER_OK) {
      ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "add callback failed.");
      printf("Fail at line %i\n", __LINE__);
      return false;
    }

    // add the frame received callback to be informed when a streaming frame has been received from the device
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- set Video callback ... ");
    errorController = ARCONTROLLER_Device_SetVideoStreamCallbacks (deviceController, decoderConfigCallback, didReceiveFrameCallback, NULL , this);
    CHECK_ERROR(errorController);

    // connection
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Connecting ...");
    errorController = ARCONTROLLER_Device_Start (deviceController);
    CHECK_ERROR(errorController);

    // wait state update update
    ARSAL_Sem_Wait (&(stateSem));

    deviceState = ARCONTROLLER_Device_GetState (deviceController, &errorController);
    if ((errorController != ARCONTROLLER_OK) || (deviceState != ARCONTROLLER_DEVICE_STATE_RUNNING)) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- deviceState :%d", deviceState);
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(errorController));
      printf("Fail at line %i\n", __LINE__);
      return false;
    }

    // send the command that tells to the Jumping to begin its streaming
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- send StreamingVideoEnable ... ");
    errorController = js()->sendMediaStreamingVideoEnable (js(), 1);
    CHECK_ERROR(errorController);
    return true;
  } // end connect()

  //////////////////////////////////////////////////////////////////////////////
protected:
  //////////////////////////////////////////////////////////////////////////////

  inline ARCONTROLLER_FEATURE_JumpingSumo_t* js() {
    return deviceController->jumpingSumo;
  }

  ////////////////////////////////////////////////////////////////////////////////

  static void speedChangedCb (int8_t speed, int16_t realSpeed, void *customData) {
    printf("speedChangedCb(speed:%i, realSpeed:%i)\n", speed, realSpeed);
    //LightSumo* this_ptr = (LightSumo*) customData;
  }

  ////////////////////////////////////////////////////////////////////////////////

  // called when the state of the device controller has changed
  static void stateChanged (eARCONTROLLER_DEVICE_STATE newState, eARCONTROLLER_ERROR /*error*/, void *customData) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - stateChanged newState: %d .....", newState);
    LightSumo* this_ptr = (LightSumo*) customData;

    switch (newState)
    {
      case ARCONTROLLER_DEVICE_STATE_STOPPED:
        ARSAL_Sem_Post (&(this_ptr->stateSem));
        //ros::shutdown();
        exit(-1);

        break;

      case ARCONTROLLER_DEVICE_STATE_RUNNING:
        ARSAL_Sem_Post (&(this_ptr->stateSem));
        break;

      default:
        break;
    }
  }

  ////////////////////////////////////////////////////////////////////////////////

  //! callback of posture
  virtual void postureChanged (uint8_t posture) {
    printf("postureChanged(%i)\n", posture);
    _posture = posture;
  }
  ////////////////////////////////////////////////////////////////////////////////

  //! callback of changing of battery level
  virtual void batteryStateChanged (uint8_t percent) {
    printf("batteryStateChanged(%i)\n", percent);
  }

  ////////////////////////////////////////////////////////////////////////////////

  //! callback called when a new image is received
  virtual void imageChanged () {
  }

  ////////////////////////////////////////////////////////////////////////////////

  static inline bool safe_get_arg(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary,
                                  const char* key,
                                  unsigned int & value) {
    if (elementDictionary == NULL)
      return false;
    // get the command received in the device controller
    ARCONTROLLER_DICTIONARY_ELEMENT_t *singleElement = NULL;
    HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, singleElement);
    if (singleElement == NULL)
      return false;
    // get the value
    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    HASH_FIND_STR (singleElement->arguments, key, arg);
    if (arg == NULL)
      return false;
    value = arg->value.U8;
    return true;
  } // end safe_get_arg()

  ////////////////////////////////////////////////////////////////////////////////

  // called when a command has been received from the drone
  static void commandReceived (eARCONTROLLER_DICTIONARY_KEY commandKey,
                               ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary,
                               void *customData)
  {
    printf("commandReceived(%i = '%s')\n", commandKey, controller_key2string(commandKey));
    LightSumo* this_ptr = (LightSumo*) customData;
    ARCONTROLLER_Device_t *deviceController = this_ptr->deviceController;
    //eARCONTROLLER_ERROR error = ARCONTROLLER_OK;

    if (deviceController == NULL)
      return;
    unsigned int val = 0;

    // speed change
    if (commandKey == ARCONTROLLER_DICTIONARY_KEY_JUMPINGSUMO_PILOTINGSTATE_SPEEDCHANGED)
      printf("Speed changed!\n");
    // posture
    else if (commandKey == ARCONTROLLER_DICTIONARY_KEY_JUMPINGSUMO_PILOTINGSTATE_POSTURECHANGED
             && safe_get_arg(elementDictionary, ARCONTROLLER_DICTIONARY_KEY_JUMPINGSUMO_PILOTINGSTATE_POSTURECHANGED_STATE, val))
      this_ptr->postureChanged(val);
    // battery
    else if (commandKey == ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED
             && safe_get_arg(elementDictionary, ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED_PERCENT, val))
      this_ptr->batteryStateChanged (val);
    // TODO
    // JUMPINGSUMO_AUDIOSETTINGSSTATE_MASTERVOLUMECHANGED
    // JUMPINGSUMO_SPEEDSETTINGSSTATE_OUTDOORCHANGED
    // JUMPINGSUMO_PILOTINGSTATE_ALERTSTATECHANGED
    // JUMPINGSUMO_NETWORKSTATE_LINKQUALITYCHANGED
  } // end commandReceived();

  ////////////////////////////////////////////////////////////////////////////////

  static eARCONTROLLER_ERROR decoderConfigCallback (ARCONTROLLER_Stream_Codec_t codec, void */*customData*/)
  {
    printf("decoderConfigCallback()\n");
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "decoderConfigCallback codec.type :%d", codec.type);
    return ARCONTROLLER_OK;
  } // end decoderConfigCallback()

  ////////////////////////////////////////////////////////////////////////////////

  static eARCONTROLLER_ERROR didReceiveFrameCallback (ARCONTROLLER_Frame_t *frame, void *customData)
  {
    //ROS_INFO_THROTTLE(1, "didReceiveFrameCallback(%i)", frame->used);
    LightSumo* this_ptr = (LightSumo*) customData;
    if (this_ptr->_pix_decoding_enabled) {
      std::vector<uchar> ans_vector (frame->data, frame->data + frame->used);
      this_ptr->pic_mutex.lock();
      this_ptr->pic = cv::imdecode(cv::Mat (ans_vector), -1);
      this_ptr->pic_idx++;
      this_ptr->pic_mutex.unlock();
      this_ptr->imageChanged();
    }
    return ARCONTROLLER_OK;
  } // end didReceiveFrameCallback()

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  ARDISCOVERY_Device_t *device;
  ARCONTROLLER_Device_t *deviceController;
  ARSAL_Sem_t stateSem;
  eARCONTROLLER_ERROR errorController;
  eARCONTROLLER_DEVICE_STATE deviceState;
  unsigned int _posture;
  cv::Mat pic;
  bool _pix_decoding_enabled;
  unsigned int pic_idx;
  boost::mutex pic_mutex;
}; // end class LightSumo

#endif // LIGHT_SUMO_H

