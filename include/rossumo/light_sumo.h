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

#include "device_state2string.h"
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
  /// speed and turn
  //////////////////////////////////////////////////////////////////////////////

  void set_speeds(double v, double w) {
    //printf("set_speeds(%i, %i)\n", v, w);
    errorController = js()->setPilotingPCMD(js(), 1, v, w);
  }

  /*!
   * \param w in radians
   */
  void sharp_turn(double w) {
    //printf("sharp_turn(%i)\n", w);
    errorController = js()->sendPilotingAddCapOffset(js(), w); // @param offset Offset value in radians
  }

  //////////////////////////////////////////////////////////////////////////////
  /// postures
  //////////////////////////////////////////////////////////////////////////////

  inline unsigned int get_posture() const { return _posture; }
  bool set_posture_standing() {
    return (js()->sendPilotingPosture(js(), ARCOMMANDS_JUMPINGSUMO_PILOTING_POSTURE_TYPE_STANDING) == ARCONTROLLER_OK);
  }
  bool set_posture_jumper() {
    return (js()->sendPilotingPosture(js(), ARCOMMANDS_JUMPINGSUMO_PILOTING_POSTURE_TYPE_JUMPER) == ARCONTROLLER_OK);
  }
  bool set_posture_kicker() {
    return (js()->sendPilotingPosture(js(), ARCOMMANDS_JUMPINGSUMO_PILOTING_POSTURE_TYPE_KICKER) == ARCONTROLLER_OK);
  }
  bool set_posture(const std::string & p) {
    if (p == "standing") return set_posture_standing();
    else if (p == "kicker") return set_posture_kicker();
    else if (p == "jumper") return set_posture_jumper();
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "unknown posture '%s'", p.c_str());
    return false;
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

  inline bool anim_spin() {
    return (js()->sendAnimationsSimpleAnimation(js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_SIMPLEANIMATION_ID_SPIN)== ARCONTROLLER_OK);
  }
  inline bool anim_tap() {
    return (js()->sendAnimationsSimpleAnimation(js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_SIMPLEANIMATION_ID_TAP)== ARCONTROLLER_OK);
  }
  inline bool anim_slowshake() {
    return (js()->sendAnimationsSimpleAnimation(js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_SIMPLEANIMATION_ID_SLOWSHAKE)== ARCONTROLLER_OK);
  }
  inline bool anim_metronome() {
    return (js()->sendAnimationsSimpleAnimation(js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_SIMPLEANIMATION_ID_METRONOME)== ARCONTROLLER_OK);
  }
  inline bool anim_ondulation() {
    return (js()->sendAnimationsSimpleAnimation(js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_SIMPLEANIMATION_ID_ONDULATION)== ARCONTROLLER_OK);
  }
  inline bool anim_spinJump() {
    return (js()->sendAnimationsSimpleAnimation(js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_SIMPLEANIMATION_ID_SPINJUMP)== ARCONTROLLER_OK);
  }
  inline bool anim_spinToPosture() {
    return (js()->sendAnimationsSimpleAnimation(js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_SIMPLEANIMATION_ID_SPINTOPOSTURE)== ARCONTROLLER_OK);
  }
  inline bool anim_spiral() {
    return (js()->sendAnimationsSimpleAnimation(js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_SIMPLEANIMATION_ID_SPIRAL)== ARCONTROLLER_OK);
  }
  inline bool anim_slalom() {
    return (js()->sendAnimationsSimpleAnimation(js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_SIMPLEANIMATION_ID_SLALOM)== ARCONTROLLER_OK);
  }
  bool anim(const std::string & p) {
    if (p == "spin") return anim_spin();
    else if (p == "tap") return anim_tap();
    else if (p == "slowshake") return anim_slowshake();
    else if (p == "metronome") return anim_metronome();
    else if (p == "ondulation") return anim_ondulation();
    else if (p == "spinJump") return anim_spinJump();
    else if (p == "spinToPosture") return anim_spinToPosture();
    else if (p == "spiral") return anim_spiral();
    else if (p == "slalom") return anim_slalom();
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "unknown anim '%s'", p.c_str());
    return false;
  }

  //////////////////////////////////////////////////////////////////////////////
  /// pictures from the camera
  //////////////////////////////////////////////////////////////////////////////

  inline void  enable_pic_decoding() { _pix_decoding_enabled = true; }
  inline void disable_pic_decoding() { _pix_decoding_enabled = false; }
  inline void get_pic(cv::Mat& out) {
    pic_mutex.lock();
    _pic.copyTo(out);
    pic_mutex.unlock();
  }
  inline unsigned int get_pic_idx() const { return _pic_idx; }

  //////////////////////////////////////////////////////////////////////////////

  bool connect() {
    // add the speed callback
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- set speed callback ... ");
    ARCOMMANDS_Decoder_SetJumpingSumoPilotingStateSpeedChangedCallback(speedChangedCb, this);

    device = NULL;
    deviceController = NULL;
    errorController = ARCONTROLLER_OK;
    deviceState = ARCONTROLLER_DEVICE_STATE_MAX;
    _posture = _battery_percentage = _volume = _pic_idx = -1;
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

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "ARDISCOVERY_Device_InitWifi ...");
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

  static void speedChangedCb (int8_t speed, int16_t realSpeed, void */*customData*/) {
    printf("speedChangedCb(speed:%i, realSpeed:%i)\n", speed, realSpeed);
    //LightSumo* this_ptr = (LightSumo*) customData;
  }

  ////////////////////////////////////////////////////////////////////////////////

  // called when the state of the device controller has changed
  static void stateChanged (eARCONTROLLER_DEVICE_STATE newState, eARCONTROLLER_ERROR /*error*/, void *customData) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "stateChanged newState: %d='%s' .....",
                newState, device_state2string(newState));
    LightSumo* this_ptr = (LightSumo*) customData;

    switch (newState)
    {
      case ARCONTROLLER_DEVICE_STATE_STOPPED:
        ARSAL_Sem_Post (&(this_ptr->stateSem));
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Trying to reconnect...");
        sleep(1);
        this_ptr->connect();
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
  virtual void batteryChanged (uint8_t battery_percentage) {
    printf("batteryChanged(%i%%)\n", battery_percentage);
    _battery_percentage = battery_percentage;
  }

  ////////////////////////////////////////////////////////////////////////////////

  //! callback of changing of volume level
  virtual void volumeChanged (uint8_t volume) {
    printf("volumeChanged(%i)\n", volume);
    _volume = volume;
  }

  ////////////////////////////////////////////////////////////////////////////////

  //! callback of changing of alert level
  virtual void alertChanged (uint8_t alert) {
    printf("alertChanged(%i)\n", alert);
//  ARCOMMANDS_JUMPINGSUMO_PILOTINGSTATE_ALERTSTATECHANGED_STATE_NONE = 0,    ///< No alert
//  ARCOMMANDS_JUMPINGSUMO_PILOTINGSTATE_ALERTSTATECHANGED_STATE_CRITICAL_BATTERY,    ///< Critical battery alert
//  ARCOMMANDS_JUMPINGSUMO_PILOTINGSTATE_ALERTSTATECHANGED_STATE_LOW_BATTERY,    ///< Low battery alert
    _alert = alert;
  }

  ////////////////////////////////////////////////////////////////////////////////

  //! callback of changing of outdoor level
  virtual void outdoorChanged (uint8_t outdoor) {
    printf("outdoorChanged(%i)\n", outdoor);
    _outdoor = outdoor;
  }

  ////////////////////////////////////////////////////////////////////////////////

  //! callback of changing of link_quality level
  virtual void link_qualityChanged (uint8_t link_quality) {
    printf("link_qualityChanged(%i)\n", link_quality);
    _link_quality = link_quality;
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
      this_ptr->batteryChanged (val);
    // volume
    else if (commandKey == ARCONTROLLER_DICTIONARY_KEY_JUMPINGSUMO_AUDIOSETTINGSSTATE_MASTERVOLUMECHANGED
             && safe_get_arg(elementDictionary, ARCONTROLLER_DICTIONARY_KEY_JUMPINGSUMO_AUDIOSETTINGSSTATE_MASTERVOLUMECHANGED_VOLUME, val))
      this_ptr->volumeChanged (val);
    // link_quality
    else if (commandKey == ARCONTROLLER_DICTIONARY_KEY_JUMPINGSUMO_NETWORKSTATE_LINKQUALITYCHANGED
             && safe_get_arg(elementDictionary, ARCONTROLLER_DICTIONARY_KEY_JUMPINGSUMO_NETWORKSTATE_LINKQUALITYCHANGED_QUALITY, val))
      this_ptr->link_qualityChanged (val);
    // alert
    else if (commandKey == ARCONTROLLER_DICTIONARY_KEY_JUMPINGSUMO_PILOTINGSTATE_ALERTSTATECHANGED
             && safe_get_arg(elementDictionary, ARCONTROLLER_DICTIONARY_KEY_JUMPINGSUMO_PILOTINGSTATE_ALERTSTATECHANGED_STATE, val))
      this_ptr->alertChanged(val);
    // outdoor
    else if (commandKey == ARCONTROLLER_DICTIONARY_KEY_JUMPINGSUMO_SPEEDSETTINGSSTATE_OUTDOORCHANGED
             && safe_get_arg(elementDictionary, ARCONTROLLER_DICTIONARY_KEY_JUMPINGSUMO_SPEEDSETTINGSSTATE_OUTDOORCHANGED_OUTDOOR, val))
      this_ptr->outdoorChanged (val);
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
      this_ptr->_pic = cv::imdecode(cv::Mat (ans_vector), -1);
      this_ptr->_pic_idx++;
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
  unsigned int _posture, _battery_percentage, _volume, _link_quality, _alert, _outdoor;
  cv::Mat _pic;
  bool _pix_decoding_enabled;
  unsigned int _pic_idx;
  boost::mutex pic_mutex;
}; // end class LightSumo

#endif // LIGHT_SUMO_H

