/*!
  \file
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/1/24

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
#include <iostream>

extern "C" {
#include <libARSAL/ARSAL.h>
#include <libARController/ARController.h>
#include <libARDiscovery/ARDiscovery.h>
}

#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/mutex.hpp>

#include <rossumo/device_state2string.h>
#include <rossumo/controller_key2string.h>

#define TAG "rosspider"
#define JS_IP_ADDRESS "192.168.2.1"
#define JS_DISCOVERY_PORT 44444

#define CHECK_ERROR(errorController) { \
  if (errorController != ARCONTROLLER_OK) { \
  ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%", ARCONTROLLER_Error_ToString(errorController)); \
  printf("Fail at line %i\n", __LINE__); \
  return false; \
  } \
  }

class LightSpider {
public:
  LightSpider() {
    // all initializations done in connect()
  }

  ~LightSpider() {
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

  bool connect() {
    // add the speed callback
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- set speed callback ... ");
    ARCOMMANDS_Decoder_SetMiniDronePilotingStateSpeedChangedCallback(speedChangedCb, this);

    device = NULL;
    deviceController = NULL;
    errorController = ARCONTROLLER_OK;
    deviceState = ARCONTROLLER_DEVICE_STATE_MAX;
    _posture = _battery_percentage = _volume = _pic_idx = -1;
    ARSAL_Sem_Init (&(stateSem), 0, 0);
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- Jumping spider Piloting --");

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
    // create a MiniDrone discovery device (ARDISCOVERY_PRODUCT_MINIDRONE)
    errorDiscovery = ARDISCOVERY_Device_InitWifi (device, ARDISCOVERY_PRODUCT_MINIDRONE, "JS", JS_IP_ADDRESS, JS_DISCOVERY_PORT);
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
    //deviceController->MiniDrone = ARCONTROLLER_FEATURE_MiniDrone_New (deviceController->privatePart->networkController, &localError);
    //ARCONTROLLER_Device.c line 191
    // deviceController->MiniDroneDebug = ARCONTROLLER_FEATURE_MiniDroneDebug_New (deviceController->privatePart->networkController, &localError);

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

    _pix_decoding_enabled = false;
    enable_pic_decoding();
    return true;
  } // end connect()

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

  inline bool play_sound(const std::string & filename) {
    printf("play_sound('%s')\n", filename.c_str());
    //ARCOMMANDS_Generator_GenerateMiniDroneDebugAudioPlaySoundWithName
    // https://stackoverflow.com/questions/7352099/stdstring-to-char
    char *cstr =new char[filename.length() + 1];
    strcpy(cstr, filename.c_str());
    // do stuff
    bool ok = (jsd()->sendAudioPlaySoundWithName(jsd(), cstr) == ARCONTROLLER_OK);
    delete [] cstr;
    return ok;
  }
  // ARCOMMANDS_MiniDrone_AUDIOSETTINGS_THEME_THEME_INSECT
  bool set_audiotheme_insect() {
    return (js()->sendAudioSettingsTheme(js(), ARCOMMANDS_MiniDrone_AUDIOSETTINGS_THEME_THEME_INSECT) == ARCONTROLLER_OK);
  }
  bool set_audiotheme_monster() {
    return (js()->sendAudioSettingsTheme(js(), ARCOMMANDS_MiniDrone_AUDIOSETTINGS_THEME_THEME_INSECT) == ARCONTROLLER_OK);
  }
  bool set_audiotheme_robot() {
    return (js()->sendAudioSettingsTheme(js(), ARCOMMANDS_MiniDrone_AUDIOSETTINGS_THEME_THEME_INSECT) == ARCONTROLLER_OK);
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

  inline ARCONTROLLER_FEATURE_MiniDrone_t* js() {
    return deviceController->miniDrone;
  }
  inline ARCONTROLLER_FEATURE_MiniDroneDebug_t* jsd() {
    return deviceController->miniDroneDebug;
  }

  ////////////////////////////////////////////////////////////////////////////////

  static void speedChangedCb (int8_t speed, int16_t realSpeed, void */*customData*/) {
    printf("speedChangedCb(speed:%i, realSpeed:%i)\n", speed, realSpeed);
    //LightSpider* this_ptr = (LightSpider*) customData;
  }

  ////////////////////////////////////////////////////////////////////////////////

  // called when the state of the device controller has changed
  static void stateChanged (eARCONTROLLER_DEVICE_STATE newState, eARCONTROLLER_ERROR /*error*/, void *customData) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "stateChanged newState: %d='%s' .....",
                newState, device_state2string(newState));
    LightSpider* this_ptr = (LightSpider*) customData;

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
    _alert = alert;
  }
  inline std::string get_alert2str() const {
    switch (_alert) {
      case ARCOMMANDS_MiniDrone_PILOTINGSTATE_ALERTSTATECHANGED_STATE_NONE:
        return "none";
      case ARCOMMANDS_MiniDrone_PILOTINGSTATE_ALERTSTATECHANGED_STATE_CRITICAL_BATTERY:
        return "critical_battery";
      case ARCOMMANDS_MiniDrone_PILOTINGSTATE_ALERTSTATECHANGED_STATE_LOW_BATTERY:
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
    LightSpider* this_ptr = (LightSpider*) customData;
    ARCONTROLLER_Device_t *deviceController = this_ptr->deviceController;
    //eARCONTROLLER_ERROR error = ARCONTROLLER_OK;

    if (deviceController == NULL)
      return;
    unsigned int val = 0;

    // speed change
    if (commandKey == ARCONTROLLER_DICTIONARY_KEY_MiniDrone_PILOTINGSTATE_SPEEDCHANGED)
      printf("Speed changed!\n");
    // posture
    else if (commandKey == ARCONTROLLER_DICTIONARY_KEY_MiniDrone_PILOTINGSTATE_POSTURECHANGED
             && safe_get_arg(elementDictionary, ARCONTROLLER_DICTIONARY_KEY_MiniDrone_PILOTINGSTATE_POSTURECHANGED_STATE, val))
      this_ptr->postureChanged(val);
    // battery
    else if (commandKey == ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED
             && safe_get_arg(elementDictionary, ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED_PERCENT, val))
      this_ptr->batteryChanged (val);
    // volume
    else if (commandKey == ARCONTROLLER_DICTIONARY_KEY_MiniDrone_AUDIOSETTINGSSTATE_MASTERVOLUMECHANGED
             && safe_get_arg(elementDictionary, ARCONTROLLER_DICTIONARY_KEY_MiniDrone_AUDIOSETTINGSSTATE_MASTERVOLUMECHANGED_VOLUME, val))
      this_ptr->volumeChanged (val);
    // link_quality
    else if (commandKey == ARCONTROLLER_DICTIONARY_KEY_MiniDrone_NETWORKSTATE_LINKQUALITYCHANGED
             && safe_get_arg(elementDictionary, ARCONTROLLER_DICTIONARY_KEY_MiniDrone_NETWORKSTATE_LINKQUALITYCHANGED_QUALITY, val))
      this_ptr->link_qualityChanged (val);
    // alert
    else if (commandKey == ARCONTROLLER_DICTIONARY_KEY_MiniDrone_PILOTINGSTATE_ALERTSTATECHANGED
             && safe_get_arg(elementDictionary, ARCONTROLLER_DICTIONARY_KEY_MiniDrone_PILOTINGSTATE_ALERTSTATECHANGED_STATE, val))
      this_ptr->alertChanged(val);
    // outdoor
    else if (commandKey == ARCONTROLLER_DICTIONARY_KEY_MiniDrone_SPEEDSETTINGSSTATE_OUTDOORCHANGED
             && safe_get_arg(elementDictionary, ARCONTROLLER_DICTIONARY_KEY_MiniDrone_SPEEDSETTINGSSTATE_OUTDOORCHANGED_OUTDOOR, val))
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
    LightSpider* this_ptr = (LightSpider*) customData;
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
}; // end class LightSpider

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

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
  LightSpider spider;
  if (!spider.connect()) {
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
    printf("retval:%i\n", spider.set_volume(param1));
  else if (choice == "sou" && nparams == 1)
    printf("retval:%i\n", spider.play_sound(argv[2]));
  else if (choice == "the" && nparams == 1)
    printf("retval:%i\n", spider.set_audiotheme(argv[2]));
  else // nothing done
    print_help(argc, argv);

  return 0;
}





