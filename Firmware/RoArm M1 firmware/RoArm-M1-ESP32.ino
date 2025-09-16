// WIFI_AP settings.
const char* AP_SSID = "ESP32_DEV";
const char* AP_PWD  = "12345678";

// WIFI_STA settings.
const char* STA_SSID = "JSBZY-2.4G";
const char* STA_PWD  = "waveshare0755";

// the MAC address of the device you want to ctrl.
uint8_t broadcastAddress[] = {0x94, 0xB5, 0x55, 0x13, 0x45, 0x08};
// uint8_t broadcastAddress[] = {0x40, 0x91, 0x51, 0xA6, 0xCA, 0x5C};
// uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


typedef struct struct_message {
  int S1_pos;
  int S2_pos;
  int S3_pos;
  int S4_pos;
  int S5_pos;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// set the default wifi mode here.
// 1 as [AP] mode, it will not connect other wifi.
// 2 as [STA] mode, it will connect to know wifi(STA_SSID).
#define DEFAULT_WIFI_MODE 1

// set the default role here.
// 0 as normal mode.
// 1 as leader, ctrl other device via ESP-NOW.
// 2 as follower, can be controled via ESP-NOW.
#define DEFAULT_ROLE 0

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

// the IIC used to control OLED screen.
// GPIO 21 - S_SDA, GPIO 22 - S_SCL, as default.
#define S_SCL 22
#define S_SDA 21

// the GPIO used to control RGB LEDs.
// GPIO 23, as default.
#define RGB_LED    23
#define NUMPIXELS  2
#define BRIGHTNESS 25

// DebugMode - 0:OFF 1:ON
#define DebugMode 1

// select the ctrl mode here.
// 1 - via web.
// 2 - via serial.
// 3 - via esp-now.
// THIS VALUE IS NOT SET BY HUMAN.
#define CTRL_VIA_WEB     1
#define CTRL_VIA_SERIAL  2
#define CTRL_VIA_ESPNOW  3
int CtrlModeSelect;


#define DEV_NAME     "RoArm-M1"
#define COMPANY_NAME "WAVESHARE"
#define MORE_INFO    "www.waveshare.com"
#define GITHUB_LINK  "github.com/waveshare/RoArm-M1"


// set the interval of the threading.
#define threadingInterval 600
#define clientInterval    10

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

int stepNumMax;
int processType = -1;
int globalStopFlag = 1;

// you can set the default espNowMode here.
// 1: leader
// 2: follower
// 3: normal
// DEFAULT: 3
int espNowMode = 3;

// set the default torqueLockStatus.
// DEFAULT: true
bool torqueLockStatus    = true;

// you can find the instructions here.
const char jsonCtrlInstructions[] PROGMEM = R"rawliteral(
EMERGENCY_STOP: {"T":0}

ANGLE_CTRL: T:cmdType, P1-5:angleInput, S:speedInput, A:accInput
{"T":1,"P1":180,"P2":0,"P3":90,"P4":90,"P5":180,"S1":200,"S2":200,"S3":200,"S4":200,"S5":200,"A1":60,"A2":60,"A3":60,"A4":60,"A5":60}

ANGLE_CTRL_INIT: move to angleCtrl initPos.
{"T":11}

COORD_CTRL: T:cmdType, P1-3:coordInput, P4:thetaAngle, P5:grabberAngle, S1:stepDelay, S5:grabberSpeed
{"T":2,"P1":277.5104065,"P2":-13.75,"P3":276.5822754,"P4":90,"P5":180,"S1":10,"S5":200}

COORD_CTRL_INIT: move to coordCtrl initPos.
{"T":12}

ST_POS_CTRL: T:cmdType, P1-5:posInput, S1-5:speedInput, A1-5:accInput.
{"T":3,"P1":2047,"P2":0,"P3":3500,"P4":1500,"P5":2047,"S1":200,"S2":200,"S3":200,"S4":200,"S5":200,"A1":60,"A2":60,"A3":60,"A4":60,"A5":60}

ST_POS_CTRL_INIT: move to posCtrl initPos.
{"T":13}

--- --- --- --- --- ---
GET_DEV_INFO: get device info.
{"T":4}

GET_ANGTOR_INFO: get the angle $ torque info of every servo.
{"T":5}

GET_INFO_BUFFER: get the buffer of IK.
{"T":6}

PLACEHOLDER: Null.
{"T":7}

--- --- === RECORD_REPLAY === --- ---
ALL_TORQUE_OFF: {"T":8,"P1":0}
 ALL_TORQUE_ON: {"T":8,"P1":1}
 GET_STEP_INFO: {"T":8,"P1":2,"P2":stepNum}
  MOVE_TO_STEP: {"T":8,"P1":3,"P2":stepNum}
   REMOVE_STEP: {"T":8,"P1":4,"P2":stepNum}
    RECORD_POS: {"T":8,"P1":5,"P2":stepNum,"P3":speed(delay every step - 5)}
  RECORD_DELAY: {"T":8,"P1":15,"P2":stepNum,"P3":delayInput - ms}
        REPLAY: {"T":8,"P1":6,"P2":stepDelay(delayTime between steps),"P3":loopTime(-1 -forever)}
NVS_SPACE_LEFT: {"T":8,"P1":7}
     NVS_CLEAR: {"T":8,"P1":777}


--- --- === ROARM_M1_CONFIG === --- ---
      CONFIG_ALL_INIT: {"T":9,"P1":6} all servos move to pos2047.
CONFIG_TORQUE_ALL_OFF: {"T":9,"P1":7}
 CONFIG_TORQUE_ALL_ON: {"T":9,"P1":8}
    CONFIG_TORQUE_OFF: {"T":9,"P1":servoNum,"P2":0}
     CONFIG_TORQUE_ON: {"T":9,"P1":servoNum,"P2":1}
          CONFIG_MOVE: {"T":9,"P1":servoNum,"P2":2,"P3":PosInput}
    CONFIG_SET_MIDDLE: {"T":9,"P1":servoNum,"P2":10}

--- --- === HELP === --- ---
HELP:{"T":10}
)rawliteral";

// Set DevName here.
const char* DevName = DEV_NAME;


// https://randomnerdtutorials.com/esp32-useful-wi-fi-functions-arduino/
#include <esp_now.h>
#include <WiFi.h>
#include <WebServer.h>
#include "WebPage.h"
#include <ArduinoJson.h>

String MAC_ADDRESS;
IPAddress IP_ADDRESS;
byte   DEV_ROLE;
byte   WIFI_MODE;
int    WIFI_RSSI;

// Create AsyncWebServer object on port 80
WebServer server(80);

// get the info from nvs.
#include "PreferencesConfig.h"

#include "RGB_CTRL.h"
#include "STSCTRL.h"
#include "ARM_CTRL.h"
#include "JSON_CTRL.h"
#include "COMMAND_DEFINE.h"
#include "CONNECT.h"
#include "BOARD_DEV.h"


void setup() {
  delay(1000);
  Serial.begin(115200);
  while(!Serial) {}

  preferences.begin("RoArm-M1", false);
  stepNumMax = preferences.getInt("stepNumMax", 0);

  InitRGB();

  getMAC();
  
  boardDevInit();

  servoInit();

  bootPosCheck();

  wifiInit();

  espNowInit();

  webServerSetup();

  threadInit();

  if(!torqueLockStatus){
    torqueCtrlAll(0);
  }
}


void loop() {
  server.handleClient();
  serialCtrl();
  delay(10);
}