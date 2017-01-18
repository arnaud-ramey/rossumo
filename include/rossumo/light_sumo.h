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

// check version
#include <boost/preprocessor/stringize.hpp>
#define ROSSUMO_ARSDK_VERSION_MAJOR (3)
#define ROSSUMO_ARSDK_VERSION_MINOR (1)
#if ARCOMMANDS_VERSION_MAJOR != ROSSUMO_ARSDK_VERSION_MAJOR || \
  ARCOMMANDS_VERSION_MINOR != ROSSUMO_ARSDK_VERSION_MINOR
#pragma message "rossumo ARSDK version:" \
  BOOST_PP_STRINGIZE(ROSSUMO_ARSDK_VERSION_MAJOR) \
  "." BOOST_PP_STRINGIZE(ROSSUMO_ARSDK_VERSION_MINOR) \
  ". Your ARSDK version is " \
  BOOST_PP_STRINGIZE(ARCOMMANDS_VERSION_MAJOR) \
  "." BOOST_PP_STRINGIZE(ARCOMMANDS_VERSION_MINOR)
#error "rossumo is not compatible with your ARSDK version."
#endif

#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/mutex.hpp>

#include "device_state2string.h"
#include "controller_key2string.h"

#define TAG "rossumo"

#define CHECK_ERROR(errorController) { \
  if (errorController != ARCONTROLLER_OK) { \
  ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(errorController)); \
  printf("Fail at line %i\n", __LINE__); \
  return false; \
  } \
  }

class LightSumo {
public:
  LightSumo() : DEFAULT_IP_ADDRESS ("192.168.2.1"), DEFAULT_DISCOVERY_PORT (44444) {
    set_connection_params(DEFAULT_IP_ADDRESS, DEFAULT_DISCOVERY_PORT);
    _max_vel_lin = _max_vel_ang = 100;
    // all initializations done in connect()
  }

  ~LightSumo() {
    disconnect();
  }

  void disconnect() {
    // we are here because of a disconnection or user has quit IHM, so safely delete everything
    if (_deviceController == NULL)
      return;
    _deviceState = ARCONTROLLER_Device_GetState (_deviceController, &_error_code);
    if ((_error_code == ARCONTROLLER_OK) && (_deviceState != ARCONTROLLER_DEVICE_STATE_STOPPED)) {
      //IHM_PrintInfo(ihm, "Disconnecting ...");
      ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Disconnecting ...");

      _error_code = ARCONTROLLER_Device_Stop (_deviceController);
      if (_error_code == ARCONTROLLER_OK) {
        // wait state update update
        ARSAL_Sem_Wait (&(_stateSem));
      }
    }

    //IHM_PrintInfo(ihm, "");
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "ARCONTROLLER_Device_Delete ...");
    ARCONTROLLER_Device_Delete (&_deviceController);

    ARSAL_Sem_Destroy (&(_stateSem));
    _deviceController = NULL;
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- END --");
  } // end dtor

  //////////////////////////////////////////////////////////////////////////////

  //! get the latest error
  inline eARCONTROLLER_ERROR get_last_error() const { return _error_code; }

  //////////////////////////////////////////////////////////////////////////////

  void set_connection_params(const std::string & ip_address, int discovery_port) {
    _ip_address = ip_address;
    _discovery_port = discovery_port;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool connect() {
    // add the speed callback - unused
    //ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- set speed callback ... ");
    //ARCOMMANDS_Decoder_SetJumpingSumoPilotingStateSpeedChangedCallback(speedChangedCb, this);

    _device = NULL;
    _deviceController = NULL;
    _error_code = ARCONTROLLER_OK;
    _deviceState = ARCONTROLLER_DEVICE_STATE_MAX;
    _posture = _battery_percentage = _volume = _pic_idx = -1;
    ARSAL_Sem_Init (&(_stateSem), 0, 0);
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- Jumping Sumo Piloting --");

    // create a discovery device
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- init discovey device ... ");
    eARDISCOVERY_ERROR errorDiscovery = ARDISCOVERY_OK;

    _device = ARDISCOVERY_Device_New (&errorDiscovery);
    if (errorDiscovery != ARDISCOVERY_OK) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Discovery error :%s", ARDISCOVERY_Error_ToString(errorDiscovery));
      printf("Fail at line %i\n", __LINE__);
      return false;
    }

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "ARDISCOVERY_Device_InitWifi ...");
    // create a JumpingSumo discovery device (ARDISCOVERY_PRODUCT_JS)
    errorDiscovery = ARDISCOVERY_Device_InitWifi (_device, ARDISCOVERY_PRODUCT_JS, "JS",
                                                  _ip_address.c_str(), _discovery_port);
    if (errorDiscovery != ARDISCOVERY_OK) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Discovery error :%s", ARDISCOVERY_Error_ToString(errorDiscovery));
      printf("Fail at line %i\n", __LINE__);
      return false;
    }

    // create a device controller
    _deviceController = ARCONTROLLER_Device_New (_device, &_error_code);
    if (_error_code != ARCONTROLLER_OK) {
      ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "Creation of deviceController failed.");
      printf("Fail at line %i\n", __LINE__);
      return false;
    }
    //deviceController->jumpingSumo = ARCONTROLLER_FEATURE_JumpingSumo_New (deviceController->privatePart->networkController, &localError);
    //ARCONTROLLER_Device.c line 191
    // deviceController->jumpingSumoDebug = ARCONTROLLER_FEATURE_JumpingSumoDebug_New (deviceController->privatePart->networkController, &localError);

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- delete discovey device ... ");
    ARDISCOVERY_Device_Delete (&_device);

    // add the state change callback to be informed when the device controller starts, stops...
    _error_code = ARCONTROLLER_Device_AddStateChangedCallback (_deviceController, stateChanged, this);
    if (_error_code != ARCONTROLLER_OK) {
      ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "add State callback failed.");
      printf("Fail at line %i\n", __LINE__);
      return false;
    }

    // add the command received callback to be informed when a command has been received from the device
    _error_code = ARCONTROLLER_Device_AddCommandReceivedCallback (_deviceController, commandReceived, this);
    if (_error_code != ARCONTROLLER_OK) {
      ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "add callback failed.");
      printf("Fail at line %i\n", __LINE__);
      return false;
    }

    // add the frame received callback to be informed when a streaming frame has been received from the device
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- set Video callback ... ");
    _error_code = ARCONTROLLER_Device_SetVideoStreamCallbacks (_deviceController, decoderConfigCallback, didReceiveFrameCallback, NULL , this);
    CHECK_ERROR(_error_code);

    // connection
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Connecting ...");
    _error_code = ARCONTROLLER_Device_Start (_deviceController);
    CHECK_ERROR(_error_code);

    // wait state update update
    ARSAL_Sem_Wait (&(_stateSem));

    _deviceState = ARCONTROLLER_Device_GetState (_deviceController, &_error_code);
    if ((_error_code != ARCONTROLLER_OK) || (_deviceState != ARCONTROLLER_DEVICE_STATE_RUNNING)) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- deviceState :%d", _deviceState);
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(_error_code));
      printf("Fail at line %i\n", __LINE__);
      return false;
    }

    _pix_decoding_enabled = false;
    enable_pic_decoding();
    return true;
  } // end connect()

  //////////////////////////////////////////////////////////////////////////////
  /// speed and turn
  //////////////////////////////////////////////////////////////////////////////

  //! clamp a value between boundaries
  inline static int clamp(int x, int min, int max) {
    return (x < min ? min : x > max ? max : x);
  }

  //! v, w in [-100, 100]
  bool set_speeds(int v, int w) {
    //printf("set_speeds(%i, %i)\n", v, w);
    v = clamp(v, -_max_vel_lin, _max_vel_lin);
    w = clamp(w, -_max_vel_ang, _max_vel_ang);
    /*
    typedef eARCONTROLLER_ERROR (*ARCONTROLLER_FEATURE_JumpingSumo_SetPilotingPCMD_t)
    (ARCONTROLLER_FEATURE_JumpingSumo_t *feature, uint8_t flag, int8_t speed, int8_t turn);
    @param speed Speed value [-100:100].
    @param turn Turn value. [-100:100]
    */
    _error_code = js()->setPilotingPCMD(js(), 1, v, w);
    return _error_code == ARCONTROLLER_OK;
  }

  /*!
   * \param w in radians
   */
  void sharp_turn(double w) {
    //printf("sharp_turn(%i)\n", w);
    _error_code = js()->sendPilotingAddCapOffset(js(), w); // @param offset Offset value in radians
  }

  //////////////////////////////////////////////////////////////////////////////
  /// postures
  //////////////////////////////////////////////////////////////////////////////

  inline unsigned int get_posture() const { return _posture; }
  inline std::string get_posture2str() const {
    switch (_posture) {
      case ARCOMMANDS_JUMPINGSUMO_PILOTING_POSTURE_TYPE_STANDING:
        return "standing";
      case ARCOMMANDS_JUMPINGSUMO_PILOTING_POSTURE_TYPE_JUMPER:
        return "kicker";
      case ARCOMMANDS_JUMPINGSUMO_PILOTING_POSTURE_TYPE_KICKER:
        return "jumper";
      default:
        return "unknown";
    }
  }
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
    _error_code = js()->sendAnimationsJump (js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_JUMP_TYPE_HIGH);
  }
  void long_jump() {
    _error_code = js()->sendAnimationsJump (js(), ARCOMMANDS_JUMPINGSUMO_ANIMATIONS_JUMP_TYPE_LONG);
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
  /// sounds
  //////////////////////////////////////////////////////////////////////////////

  //! \arg vol in 0~100
  inline bool set_volume(int vol) {
    printf("set_volume(%i)\n", vol);
    if (vol > 100) vol = 100;
    if (vol < 0)   vol = 0;
    return (js()->sendAudioSettingsMasterVolume(js(), vol)== ARCONTROLLER_OK);
  }
  inline unsigned int get_volume() const { return _volume; }
  inline bool increase_volume() { return set_volume( get_volume() + 5 ); }
  inline bool decrease_volume() { return set_volume( ((int) get_volume()) - 5 ); }

  // ARCOMMANDS_JUMPINGSUMO_AUDIOSETTINGS_THEME_THEME_INSECT
  bool set_audiotheme_insect() {
    return (js()->sendAudioSettingsTheme(js(), ARCOMMANDS_JUMPINGSUMO_AUDIOSETTINGS_THEME_THEME_INSECT) == ARCONTROLLER_OK);
  }
  bool set_audiotheme_monster() {
    return (js()->sendAudioSettingsTheme(js(), ARCOMMANDS_JUMPINGSUMO_AUDIOSETTINGS_THEME_THEME_INSECT) == ARCONTROLLER_OK);
  }
  bool set_audiotheme_robot() {
    return (js()->sendAudioSettingsTheme(js(), ARCOMMANDS_JUMPINGSUMO_AUDIOSETTINGS_THEME_THEME_INSECT) == ARCONTROLLER_OK);
  }
  bool set_audiotheme(const std::string & p) {
    if (p == "insect") return set_audiotheme_insect();
    else if (p == "monster") return set_audiotheme_monster();
    else if (p == "robot") return set_audiotheme_robot();
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "unknown audiotheme '%s'", p.c_str());
    return false;
  }

  //////////////////////////////////////////////////////////////////////////////
  /// pictures from the camera
  //////////////////////////////////////////////////////////////////////////////

  //! send the command that tells to the Jumping to begin its streaming
  inline bool enable_pic_decoding() {
    if (_pix_decoding_enabled)
      return true;
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- send StreamingVideoEnable ... ");
    if (js()->sendMediaStreamingVideoEnable (js(), 1) != ARCONTROLLER_OK)
      return false;
    _pix_decoding_enabled = true;
    return true;
  }
  //! send the command that tells to the Jumping to stop its streaming
  inline bool disable_pic_decoding() {
    if (!_pix_decoding_enabled)
      return true;
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- send StreamingVideoDisable ... ");
    if (js()->sendMediaStreamingVideoEnable (js(), false) != ARCONTROLLER_OK)
      return false;
    _pix_decoding_enabled = false;
    return true;
  }
  inline void get_pic(cv::Mat& out) {
    pic_mutex.lock();
    _pic.copyTo(out);
    pic_mutex.unlock();
  }
  inline unsigned int get_pic_idx() const { return _pic_idx; }

  //////////////////////////////////////////////////////////////////////////////
protected:
  //////////////////////////////////////////////////////////////////////////////

  inline ARCONTROLLER_FEATURE_JumpingSumo_t* js() {
    return _deviceController->jumpingSumo;
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
        ARSAL_Sem_Post (&(this_ptr->_stateSem));
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Trying to reconnect...");
        sleep(1);
        this_ptr->connect();
        break;

      case ARCONTROLLER_DEVICE_STATE_RUNNING:
        ARSAL_Sem_Post (&(this_ptr->_stateSem));
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
    _alert = alert;
  }
  inline std::string get_alert2str() const {
    switch (_alert) {
      case ARCOMMANDS_JUMPINGSUMO_PILOTINGSTATE_ALERTSTATECHANGED_STATE_NONE:
        return "none";
      case ARCOMMANDS_JUMPINGSUMO_PILOTINGSTATE_ALERTSTATECHANGED_STATE_CRITICAL_BATTERY:
        return "critical_battery";
      case ARCOMMANDS_JUMPINGSUMO_PILOTINGSTATE_ALERTSTATECHANGED_STATE_LOW_BATTERY:
        return "low_battery";
      default:
        break;
    }
    return "unknown";
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
    ARCONTROLLER_Device_t *deviceController = this_ptr->_deviceController;
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
    //printf("didReceiveFrameCallback(%i)\n", frame->used);
    LightSumo* this_ptr = (LightSumo*) customData;
    std::vector<uchar> ans_vector (frame->data, frame->data + frame->used);
    this_ptr->pic_mutex.lock();
    this_ptr->_pic = cv::imdecode(cv::Mat (ans_vector), -1);
    this_ptr->_pic_idx++;
    this_ptr->pic_mutex.unlock();
    this_ptr->imageChanged();
    return ARCONTROLLER_OK;
  } // end didReceiveFrameCallback()

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  std::string _ip_address, DEFAULT_IP_ADDRESS;
  int _discovery_port, DEFAULT_DISCOVERY_PORT;
  ARDISCOVERY_Device_t *_device;
  ARCONTROLLER_Device_t *_deviceController;
  ARSAL_Sem_t _stateSem;
  eARCONTROLLER_ERROR _error_code;
  eARCONTROLLER_DEVICE_STATE _deviceState;
  int _max_vel_lin, _max_vel_ang;
  unsigned int _posture, _battery_percentage, _volume, _link_quality, _alert, _outdoor;
  cv::Mat _pic;
  bool _pix_decoding_enabled;
  unsigned int _pic_idx;
  boost::mutex pic_mutex;
}; // end class LightSumo

#endif // LIGHT_SUMO_H

