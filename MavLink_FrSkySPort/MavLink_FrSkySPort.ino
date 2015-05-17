
/*
APM2.5 Mavlink to FrSky X8R SPort interface using Teensy 3.1  http://www.pjrc.com/teensy/index.html
 based on ideas found here http://code.google.com/p/telemetry-convert/
 ******************************************************
 Cut board on the backside to separate Vin from VUSB

 Connection on Teensy 3.1:
 SPort S --> TX1
 SPort + --> Vin
 SPort  - --> GND

 APM Telemetry DF13-5  Pin 2 --> RX2
 APM Telemetry DF13-5  Pin 3 --> TX2
 APM Telemetry DF13-5  Pin 5 --> GND

 Note that when used with other telemetry device (3DR Radio 433 or 3DR Bluetooth tested) in parallel on the same port the Teensy should only Receive, so please remove it's TX output (RX input on PixHawk or APM)

 Analog input  --> A0 (pin14) on Teensy 3.1 ( max 3.3 V ) - Not used


 This is the data we send to FrSky, you can change this to have your own
 set of data

******************************************************
Data transmitted to FrSky Taranis:

Cell            ( Voltage of Cell=Cells/(Number of cells). [V])
Cells           ( Voltage from LiPo [V] )
A2              ( HDOP value * 25 - 8 bit resolution)
A3              ( Roll angle from -Pi to +Pi radians, converted to a value between 0 and 1024)
A4              ( Pitch angle from -Pi/2 to +Pi/2 radians, converted to a value between 0 and 1024)
Alt             ( Altitude from baro.  [m] )
GAlt            ( Altitude from GPS   [m])
HDG             ( Compass heading  [deg]) v
Rpm             ( Throttle when ARMED [%] *100 + % battery remaining as reported by Mavlink)
VSpd            ( Vertical speed [m/s] )
Speed           ( Ground speed from GPS,  [km/h] )
T1              ( GPS status = ap_sat_visible*10) + ap_fixtype )
T2              ( Armed Status and Mavlink Messages :- 16 bit value: bit 1: armed - bit 2-5: severity +1 (0 means no message - bit 6-15: number representing a specific text)
Vfas            ( same as Cells )
Longitud        ( Longitud )
Latitud         ( Latitud )
Dist            ( Will be calculated by FrSky Taranis as the distance from first received lat/long = Home Position )
Fuel            ( Current Flight Mode reported by Mavlink )

AccX            ( X Axis average vibration m/s?)
AccY            ( Y Axis average vibration m/s?)
AccZ            ( Z Axis average vibration m/s?)

******************************************************

 */

#include <GCS_MAVLink.h>
#include "FrSkySPort.h"
#include <FastLED.h>

#define debugSerial           Serial
#define _MavLinkSerial      Serial2
#define START                   1
#define MSG_RATE            10              // Hertz

#define LED_PIN     6
#define NUM_LEDS    60
#define NUM_ARMS    6
#define MAXBRIGHTNESS  150 // maximum brightness for normal lights
#define DIMNESS 20 // minimum brightness for breathing
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB

//ap_base_mode values
#define DISARMED 0
#define ARMED 1
//ap_custom_mode values
#define STABILIZED 0
#define ACRO 1
#define ALT_HOLD 2
#define AUTO 3
#define GUIDED 4
#define LOITER 5
#define RTL 6
#define CIRCLE 7
#define LAND 9
#define OF_LOITER 10
#define DRIFT 11
#define SPORT 13
#define FLIP 14
#define AUTO_TUNE 15
#define POSITION 16


const int kBreathingCodeDelay = 50; // how often should the breathing code excute (ms)
const int kStrobeDelay = 80; // Strobe blink delay (ms)
const int kStrobeBlinkNumber = 2; // Number of strobe blinks each cycle
const int kStrobeBlinkPause = 1500; // Time between strobe blink sets (ms)
const int kChaseCycleDelay = 150; // chase cycle delay (ms)
const int kCircleCycleDelay = 75; // circle cycle delay (ms)
const int kLandingLightsOnAltitude = 300; // Altitude (cm) below which landing lights should be ON if decending
const float kBatteryFailsafeVoltage = 14000.0; // Low battery (millivolts)
const float kBatteryDimVoltage = 15000.0; // Voltage below which the leds shold start dimming (millivolts)
const float kBatteryFullVoltage = 16800.0; // Full battery (millivolts)
const int kDefaultDelay = 500;//how often should the check flight mode code excute (ms)


CRGB leds[NUM_LEDS];


CRGBPalette16 currentPalette;
TBlendType    currentBlending;

//#define DEBUG_VFR_HUD
//#define DEBUG_GPS_RAW
//#define DEBUG_ACC
//#define DEBUG_BAT
//#define DEBUG_MODE
//#define DEBUG_STATUS
//#define DEBUG_ATTITUDE
//#define DEBUG_STROBE

//#define DEBUG_FRSKY_SENSOR_REQUEST

//#define DEBUG_AVERAGE_VOLTAGE
#define DEBUG_PARSE_STATUS_TEXT

// ******************************************
// Message #0  HEARTHBEAT
uint8_t    ap_type = 0;
uint8_t    ap_autopilot = 0;
uint8_t    ap_base_mode = 0;
int32_t    ap_custom_mode = -1;
uint8_t    ap_system_status = 0;
uint8_t    ap_mavlink_version = 0;

// Message # 1  SYS_STATUS
uint16_t   ap_voltage_battery = 0;       // 1000 = 1V
int16_t    ap_current_battery = 0;      //  10 = 1A
int8_t    ap_battery_remaining = 0;   // Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery

// Message #24  GPS_RAW_INT
uint8_t    ap_fixtype = 3;                //   0= No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
uint8_t    ap_sat_visible = 0;            // number of visible satelites

// FrSky Taranis uses the first recieved lat/long as homeposition.
int32_t    ap_latitude = 0;               // 585522540;
int32_t    ap_longitude = 0;              // 162344467;
int32_t    ap_gps_altitude = 0;           // 1000 = 1m
int32_t    ap_gps_speed = 0;
uint16_t   ap_gps_hdop = 255;             // GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
// uint16_t    ap_vdop=0;                 // GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
uint16_t   my_gps_hdop = 9999;             //
uint32_t    ap_cog = 0;                // Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
uint16_t   last_ap_gps_hdop = 0;

// Message #74 VFR_HUD
uint32_t  ap_groundspeed = 0;       // Current ground speed in m/s
uint32_t  ap_heading = 0;           // Current heading in degrees, in compass units (0..360, 0=north)
uint16_t  ap_throttle = 0;          // Current throttle setting in integer percent, 0 to 100

// uint32_t  ap_alt=0;             //Current altitude (MSL), in meters
// int32_t   ap_airspeed = 0;      // Current airspeed in m/s


// FrSky Taranis uses the first recieved value after 'PowerOn' or  'Telemetry Reset'  as zero altitude
int32_t    ap_bar_altitude = 0;    // 100 = 1m
int32_t    ap_climb_rate = 0;      // 100= 1m/s

// Messages needed to use current Angles and axis speeds
// Message #30 ATTITUDE           //MAVLINK_MSG_ID_ATTITUDE

int32_t ap_roll_angle = 0;    //Roll angle (deg -180/180)
int32_t ap_pitch_angle = 0;   //Pitch angle (deg -180/180)
uint32_t ap_yaw_angle = 0;     //Yaw angle (rad)
uint32_t ap_roll_speed = 0;    //Roll angular speed (rad/s)
uint32_t ap_pitch_speed = 0;   //Pitch angular speed (rad/s)
uint32_t ap_yaw_speed = 0;     //Yaw angular speed (rad/s)


// Message #253 MAVLINK_MSG_ID_STATUSTEXT
uint16_t   ap_status_severity = 255;
uint16_t   ap_status_send_count = 0;
uint16_t   ap_status_text_id = 0;
mavlink_statustext_t statustext;

/*

  MAV_SEVERITY_EMERGENCY=0, System is unusable. This is a "panic" condition.
  MAV_SEVERITY_ALERT=1, Action should be taken immediately. Indicates error in non-critical systems.
  MAV_SEVERITY_CRITICAL=2, Action must be taken immediately. Indicates failure in a primary system.
  MAV_SEVERITY_ERROR=3, Indicates an error in secondary/redundant systems.
  MAV_SEVERITY_WARNING=4, Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.
  MAV_SEVERITY_NOTICE=5, An unusual event has occured, though not an error condition. This should be investigated for the root cause.
  MAV_SEVERITY_INFO=6, Normal operational messages. Useful for logging. No action is required for these messages.
  MAV_SEVERITY_DEBUG=7, Useful non-operational messages that can assist in debugging. These should not occur during normal operation.
  MAV_SEVERITY_ENUM_END=8,

*/


// ******************************************
// These are special for FrSky
int32_t     vfas = 0;                // 100 = 1,0V
int32_t     gps_status = 0;     // (ap_sat_visible * 10) + ap_fixtype
// ex. 83 = 8 sattelites visible, 3D lock
uint8_t   ap_cell_count = 0;

// ******************************************
uint8_t     MavLink_Connected;
uint8_t     buf[MAVLINK_MAX_PACKET_LEN];

uint16_t  hb_count;

unsigned long MavLink_Connected_timer;
unsigned long hb_timer;

int led = 13;

boolean arm_flag = false;
boolean strobe_flag = true;
boolean breathe_flag = false;
boolean chase_flag = false;
boolean circle_flag = false;
boolean circle_toggle = false;
boolean arm_blink_array[6] =
{
  false, false, false, false, false, false
};

uint8_t arm_number = 0;
uint8_t flight_mode = 0;
uint8_t last_flight_mode = 50;
uint8_t chase_arm = 0;
uint8_t chase_position = 9;
uint8_t circle_arm = 0;
uint8_t strobe_blink_counter = 0;
uint8_t hue;
uint8_t brightness = MAXBRIGHTNESS;

float gas_tank = 0.0;

unsigned long last_chase = 0;
unsigned long last_circle = 0;
unsigned long last_blink = 0;
unsigned long last_strobe = 0;
unsigned long test_counter = 0;
unsigned long last_breath = 0;
unsigned long last_default = 0;

mavlink_message_t msg;

// ******************************************
void setup()  {

  FrSkySPort_Init();
  _MavLinkSerial.begin(57600);
  //debugSerial.begin(57600);
  MavLink_Connected = 0;
  MavLink_Connected_timer = millis();
  hb_timer = millis();
  hb_count = 0;


  pinMode(led, OUTPUT);
  pinMode(12, OUTPUT);

  pinMode(14, INPUT);
  analogReference(DEFAULT);

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(brightness);

  currentPalette = RainbowColors_p;
  currentBlending = NOBLEND;

}


// ******************************************
void loop()  {
  uint16_t len;

  if (millis() - hb_timer > 1500) {
    hb_timer = millis();
    if (!MavLink_Connected) {   // Start requesting data streams from MavLink
      digitalWrite(led, HIGH);
      mavlink_msg_request_data_stream_pack(0xFF, 0xBE, &msg, 1, 1, MAV_DATA_STREAM_EXTENDED_STATUS, MSG_RATE, START);
      len = mavlink_msg_to_send_buffer(buf, &msg);
      _MavLinkSerial.write(buf, len);
      delay(10);
      mavlink_msg_request_data_stream_pack(0xFF, 0xBE, &msg, 1, 1, MAV_DATA_STREAM_EXTRA2, MSG_RATE, START);
      len = mavlink_msg_to_send_buffer(buf, &msg);
      _MavLinkSerial.write(buf, len);
      delay(10);
      mavlink_msg_request_data_stream_pack(0xFF, 0xBE, &msg, 1, 1, MAV_DATA_STREAM_RAW_SENSORS, MSG_RATE, START);
      len = mavlink_msg_to_send_buffer(buf, &msg);
      _MavLinkSerial.write(buf, len);
      digitalWrite(led, LOW);
    }
  }

  if ((millis() - MavLink_Connected_timer) > 1500)  {  // if no HEARTBEAT from APM  in 1.5s then we are not connected
    MavLink_Connected = 0;
    hb_count = 0;
  }

  _MavLink_receive();                   // Check MavLink communication

  FrSkySPort_Process();               // Check FrSky S.Port communication



  if ((millis() - last_default) > 100)
  {
    last_default = millis();

    // calculate porportion of voltage remaining (gas_tank)
    gas_tank = float((ap_voltage_battery - kBatteryFailsafeVoltage) / (kBatteryFullVoltage - kBatteryFailsafeVoltage));

    // calculate how much brightness is reasonable given the ap_voltage_battery
    if (ap_voltage_battery <= kBatteryFailsafeVoltage) brightness = 0;
    if (ap_voltage_battery <= kBatteryDimVoltage) brightness = map(ap_voltage_battery, kBatteryFailsafeVoltage, kBatteryDimVoltage, 20, MAXBRIGHTNESS);
    if (ap_voltage_battery > kBatteryDimVoltage) brightness = MAXBRIGHTNESS;

    // **********STROBE code *******************

#ifdef DEBUG_STROBE
    debugSerial.print(millis());
        debugSerial.println("Im in the strobe ");
#endif
  strobe_blink_counter++; // if enough millis have passed since last blink then increment the number of blinks
    if (strobe_blink_counter <= (kStrobeBlinkNumber * 2)) //if armed and in need of a blink
    {
      if (strobe_flag)
      {
        for (uint8_t i = 9; i <= 60; i += 10)
        {
          leds[i] = CRGB::White;
        }
      }
      else
      {
        for (uint8_t i = 9; i <= 60; i += 10)
        {
          leds[i] = CRGB::Black;
        }
      }
      FastLED.setBrightness(255);
      FastLED.show();
      strobe_flag = !strobe_flag; // toggle color from on to off
    }
    if ((millis() - last_blink) > kStrobeBlinkPause)  // if enought millis have passed since the blink sequence then Reset the blink counter
    {
      last_blink = millis();
      strobe_blink_counter = 0;
    }
//Basic Lights setup and display
    SetUpNormalPalette();
    FastLED.setBrightness(brightness);
    RefreshArms();

    /*

              // *************** Chase Code ****************
              if (chase_flag)
              {
                uint8_t led_position;

                if ((millis() - last_chase) > kChaseCycleDelay)
                {
                  last_chase = millis();

                  for (chase_arm = 0 ; chase_arm <= 5 ; chase_arm++)
                  {
                    SetArmColors(chase_arm);

                    led_position = chase_arm * 10 + chase_position;

                    leds[led_position] = CRGB::Black;
                    if (chase_position < 8) leds[led_position - 1] = CRGB::Black;

                    FastLED.show();
                  }
                  chase_position--;

                  if (chase_position <= 0) chase_position = 9;

                }
              }

              /// ********** CIRCLE Code ****************

              if (circle_flag)
              {
                if ((millis() - last_circle) > kCircleCycleDelay)
                {
                  last_circle = millis();
                  if (circle_toggle)
                  {
                    SetArmColorsBlack(circle_arm);
                  }
                  else
                  {
                    SetArmColors(circle_arm);
                  }
                  circle_arm++;
                  if (circle_arm >= 6)
                  {
                    circle_toggle = !circle_toggle;
                    circle_arm = 0;
                  }
                }
              }

              // ************** Breathing Code **************
              if (breathe_flag)
              {
                if ((millis() - last_breath) > kBreathingCodeDelay)
                {
                  last_breath = millis();
                  uint8_t breathing = (millis() / 15) % 255;
                  //FastLED.setBrightness(map(quadwave8(breathing), 0, 255, DIMNESS, MAXBRIGHTNESS));
                  brightness = (map(quadwave8(breathing), 0, 255, DIMNESS, MAXBRIGHTNESS));
                  SetGasTankLeds();
                  FastLED.show();
                }
              }
              else
              {
                brightness = MAXBRIGHTNESS;
              }
              */
  }
}


void _MavLink_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (_MavLinkSerial.available())
  {
    uint8_t c = _MavLinkSerial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      switch (msg.msgid)
      {
      case MAVLINK_MSG_ID_HEARTBEAT:  // 0
        ap_base_mode = (mavlink_msg_heartbeat_get_base_mode(&msg) & 0x80) > 7;
        ap_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
#ifdef DEBUG_MODE
        debugSerial.print(millis());
        debugSerial.print("\tMAVLINK_MSG_ID_SYS_STATUS: base_mode: ");
        debugSerial.print((mavlink_msg_heartbeat_get_base_mode(&msg) & 0x80) > 7);
        debugSerial.print(", custom_mode: ");
        debugSerial.print(mavlink_msg_heartbeat_get_custom_mode(&msg));
        debugSerial.println();
#endif
        MavLink_Connected_timer = millis();
        if (!MavLink_Connected);
        {
          hb_count++;
          if ((hb_count++) > 10) {       // If  received > 10 heartbeats from MavLink then we are connected
            MavLink_Connected = 1;
            hb_count = 0;
            digitalWrite(led, HIGH);     // LED will be ON when connected to MavLink, else it will slowly blink
          }
        }
        break;
      case MAVLINK_MSG_ID_STATUSTEXT:     //253
        mavlink_msg_statustext_decode(&msg, &statustext);
        ap_status_severity = statustext.severity;
        ap_status_send_count = 5;
        parseStatusText(statustext.severity, statustext.text);

#ifdef DEBUG_STATUS
        debugSerial.print(millis());
        debugSerial.print("\tMAVLINK_MSG_ID_STATUSTEXT: severity ");
        debugSerial.print(statustext.severity);
        debugSerial.print(", text");
        debugSerial.print(statustext.text);
        debugSerial.println();
#endif
        break;
        break;
      case MAVLINK_MSG_ID_SYS_STATUS :   // 1
        ap_voltage_battery = mavlink_msg_sys_status_get_voltage_battery(&msg);  // 1 = 1mV
        ap_current_battery = mavlink_msg_sys_status_get_current_battery(&msg);     // 1=10mA
        ap_battery_remaining = mavlink_msg_sys_status_get_battery_remaining(&msg); //battery capacity reported in %
        storeVoltageReading(ap_voltage_battery);
        storeCurrentReading(ap_current_battery);

#ifdef DEBUG_BAT
        debugSerial.print(millis());
        debugSerial.print("\tMAVLINK_MSG_ID_SYS_STATUS: voltage_battery: ");
        debugSerial.print(mavlink_msg_sys_status_get_voltage_battery(&msg));
        debugSerial.print(", current_battery: ");
        debugSerial.print(mavlink_msg_sys_status_get_current_battery(&msg));
        debugSerial.println();
#endif
        uint8_t temp_cell_count;
        if (ap_voltage_battery > 21000) temp_cell_count = 6;
        else if (ap_voltage_battery > 17500) temp_cell_count = 5;
        else if (ap_voltage_battery > 12750) temp_cell_count = 4;
        else if (ap_voltage_battery > 8500) temp_cell_count = 3;
        else if (ap_voltage_battery > 4250) temp_cell_count = 2;
        else temp_cell_count = 0;
        if (temp_cell_count > ap_cell_count)
          ap_cell_count = temp_cell_count;
        break;

      case MAVLINK_MSG_ID_GPS_RAW_INT:   // 24
        ap_fixtype = mavlink_msg_gps_raw_int_get_fix_type(&msg);                               // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix
        ap_sat_visible =  mavlink_msg_gps_raw_int_get_satellites_visible(&msg);          // numbers of visible satelites
        gps_status = (ap_sat_visible * 10) + ap_fixtype;
        ap_gps_hdop = mavlink_msg_gps_raw_int_get_eph(&msg) / 4;
        my_gps_hdop = mavlink_msg_gps_raw_int_get_eph(&msg);
        // Max 8 bit
        if (ap_gps_hdop == 0 || ap_gps_hdop > 255)
          ap_gps_hdop = 255;
        if (ap_fixtype == 3)  {
          ap_latitude = mavlink_msg_gps_raw_int_get_lat(&msg);
          ap_longitude = mavlink_msg_gps_raw_int_get_lon(&msg);
          ap_gps_altitude = mavlink_msg_gps_raw_int_get_alt(&msg);      // 1m =1000
          ap_gps_speed = mavlink_msg_gps_raw_int_get_vel(&msg);         // 100 = 1m/s
          ap_cog = mavlink_msg_gps_raw_int_get_cog(&msg) / 100;
        }
        else
        {
          ap_gps_speed = 0;
        }
#ifdef DEBUG_GPS_RAW
        debugSerial.print(millis());
        debugSerial.print("\tMAVLINK_MSG_ID_GPS_RAW_INT: fixtype: ");
        debugSerial.print(ap_fixtype);
        debugSerial.print(", visiblesats: ");
        debugSerial.print(ap_sat_visible);
        debugSerial.print(", status: ");
        debugSerial.print(gps_status);
        debugSerial.print(", gpsspeed: ");
        debugSerial.print(mavlink_msg_gps_raw_int_get_vel(&msg) / 100.0);
        debugSerial.print(", hdop: ");
        debugSerial.print(mavlink_msg_gps_raw_int_get_eph(&msg) / 100.0);
        debugSerial.print(", alt: ");
        debugSerial.print(mavlink_msg_gps_raw_int_get_alt(&msg));
        debugSerial.print(", cog: ");
        debugSerial.print(mavlink_msg_gps_raw_int_get_cog(&msg));
        debugSerial.println();
#endif
        break;

      case MAVLINK_MSG_ID_RAW_IMU:   // 27
        storeAccX(mavlink_msg_raw_imu_get_xacc(&msg) / 10);
        storeAccY(mavlink_msg_raw_imu_get_yacc(&msg) / 10);
        storeAccZ(mavlink_msg_raw_imu_get_zacc(&msg) / 10);

#ifdef DEBUG_ACC
        debugSerial.print(millis());
        debugSerial.print("\tMAVLINK_MSG_ID_RAW_IMU: xacc: ");
        debugSerial.print(mavlink_msg_raw_imu_get_xacc(&msg));
        debugSerial.print(", yacc: ");
        debugSerial.print(mavlink_msg_raw_imu_get_yacc(&msg));
        debugSerial.print(", zacc: ");
        debugSerial.print(mavlink_msg_raw_imu_get_zacc(&msg));
        debugSerial.println();
#endif
        break;

      case MAVLINK_MSG_ID_ATTITUDE:     //30
        ap_roll_angle = mavlink_msg_attitude_get_roll(&msg) * 180 / 3.1416; //value comes in rads, convert to deg
        // Not upside down
        if (abs(ap_roll_angle) <= 90)
        {
          ap_pitch_angle = mavlink_msg_attitude_get_pitch(&msg) * 180 / 3.1416; //value comes in rads, convert to deg
        }
        // Upside down
        else
        {
          ap_pitch_angle = 180 - mavlink_msg_attitude_get_pitch(&msg) * 180 / 3.1416; //value comes in rads, convert to deg
        }
        ap_yaw_angle = (mavlink_msg_attitude_get_yaw(&msg) + 3.1416) * 162.9747; //value comes in rads, add pi and scale to 0 to 1024

#ifdef DEBUG_ATTITUDE
        debugSerial.print("MAVLINK Roll Angle: ");
        debugSerial.print(mavlink_msg_attitude_get_roll(&msg));
        debugSerial.print(" (");
        debugSerial.print(ap_roll_angle);
        debugSerial.print("deg)");
        debugSerial.print("\tMAVLINK Pitch Angle: ");
        debugSerial.print(mavlink_msg_attitude_get_pitch(&msg));
        debugSerial.print(" (");
        debugSerial.print(ap_pitch_angle);
        debugSerial.print("deg)");
        debugSerial.print("\tMAVLINK Yaw Angle: ");
        debugSerial.print(mavlink_msg_attitude_get_yaw(&msg));
        debugSerial.println();
#endif
        break;
      case MAVLINK_MSG_ID_VFR_HUD:   //  74
        ap_groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);      // 100 = 1m/s
        ap_heading = mavlink_msg_vfr_hud_get_heading(&msg);              // 100 = 100 deg
        ap_throttle = mavlink_msg_vfr_hud_get_throttle(&msg);            //  100 = 100%
        ap_bar_altitude = mavlink_msg_vfr_hud_get_alt(&msg) * 100;       //  m
        ap_climb_rate = mavlink_msg_vfr_hud_get_climb(&msg) * 100;       //  m/s
#ifdef DEBUG_VFR_HUD
        debugSerial.print(millis());
        debugSerial.print("\tMAVLINK_MSG_ID_VFR_HUD: groundspeed: ");
        debugSerial.print(ap_groundspeed);
        debugSerial.print(", heading: ");
        debugSerial.print(ap_heading);
        debugSerial.print(", throttle: ");
        debugSerial.print(ap_throttle);
        debugSerial.print(", alt: ");
        debugSerial.print(ap_bar_altitude);
        debugSerial.print(", climbrate: ");
        debugSerial.print(ap_climb_rate);
        debugSerial.println();
#endif
        break;
      default:
        break;
      }

    }
  }
}

//*****************ADDITIONAL FUNCTIONS FOR LEDS *********************


void Reset()  // Used to turn off arm array flags after change of flight mode
{
  for (uint8_t i = 0; i <= 5; i++)
  {
    arm_blink_array[i] = false;
  }
  breathe_flag = false;
  chase_flag = false;
  circle_flag = false;
}

void RefreshArms()  //Make sure arms are all the right color
{
  for (uint8_t i = 0; i <= 5; i++)
  {
    SetArmColors(i);
  }

  FastLED.setBrightness(brightness);
  FastLED.show();
}

void SetArmColors(uint8_t arm_number) // calculate gas gage and set arm colors
{

  for (int j = (arm_number * 10); (j <= (arm_number * 10 + 8)); j++)
  {
    leds[j] = ColorFromPalette(currentPalette, arm_number * 16, 255, currentBlending);
  }

  if (arm_number == 3)
  {
    SetGasTankLeds();
  }
  FastLED.setBrightness(brightness);
  FastLED.show();

}

void SetGasTankLeds()
{
  //set arm 3 leds orange to reflect voltage (FULL)
  for (uint8_t i = 30; i <= 30 + round(gas_tank * 9); i++)
  {
    leds[i] = CHSV(32, 255, 160);

  }
  FastLED.setBrightness(brightness);
  FastLED.show();
}

void SetArmColorsBlack(uint8_t arm_number) // calculate gas gage and set arm colors
{
  if (arm_number == 3)
  {
    //set arm 3 leds orange to reflect voltage (FULL)
    for (uint8_t i = 30; i <= 30 + round(gas_tank * 9); i++)
    {
      leds[i] = CHSV(32, 255, 50);
    }
    //set other arm 3 leds to black (EMPTY)
    for (uint8_t i = 30 + round(gas_tank * 9 + 1); i <= 38; i++)
    {
      leds[i] = CRGB::Black;

    }
  }
  else
  {
    // if not arm 3 then set all the pixels to black
    for (uint8_t i = arm_number * 10; i <= ((arm_number * 10) + 8); i++)
    {
      leds[i] = CRGB::Black;

    }
  }
  FastLED.setBrightness(brightness);
  FastLED.show();
}

void SetUpDisarmedPalette()
{
  fill_solid( currentPalette, 16, CRGB::Black);
}

void SetUpLandingPalette()
{
  fill_solid( currentPalette, 16, CRGB::White);
}

void SetupReturnToLaunchPalette()
{
  // 'black out' all 16 palette entries...
  fill_solid( currentPalette, 16, CRGB::Black);
  // and set every fourth one to white.
  currentPalette[0] = CRGB::White;
  currentPalette[4] = CRGB::White;
  currentPalette[8] = CRGB::White;
  currentPalette[12] = CRGB::White;

}

void SetUpStabilizePalette()
{
  CRGB white = CRGB::White;
  CRGB black  = CRGB::Black;
  CRGB red = CHSV(HUE_RED, 255, 255);
  CRGB green  = CHSV( HUE_GREEN, 255, 255);
  CRGB blue = CHSV(HUE_BLUE, 255, 255);


  fill_solid( currentPalette, 16, CRGB::Black);
  currentPalette = CRGBPalette16(
                     white, green, black, blue,
                     black, red, black, black,
                     black, black, black, black,
                     black, black, black, black );

}

void SetUpNormalPalette()
{
  CRGB white = CRGB::White;
  CRGB black  = CRGB::Black;
  CRGB red = CHSV(HUE_RED, 255, 255);
  CRGB green  = CHSV( HUE_GREEN, 255, 255);
  CRGB blue = CHSV(HUE_BLUE, 255, 255);


  fill_solid( currentPalette, 16, CRGB::Black);
  currentPalette = CRGBPalette16(
                     white, green, green, blue,
                     red, red, black, black,
                     black, black, black, black,
                     black, black, black, black );

}

void SetUpAutoPalette()
{
  CRGB white = CRGB::White;
  CRGB black  = CRGB::Black;
  CRGB red = CHSV(HUE_RED, 255, 255);
  CRGB green  = CHSV( HUE_GREEN, 255, 255);
  CRGB blue = CHSV(HUE_BLUE, 255, 255);


  fill_solid( currentPalette, 16, CRGB::Black);
  currentPalette = CRGBPalette16(
                     white, green, white, blue,
                     white, red, black, black,
                     black, black, black, black,
                     black, black, black, black );
}


void ChangeFlightMode() // used for testing purposes...
{
  if ((millis() - test_counter) > 4000)
  {
    flight_mode++;
    test_counter = millis();
  }
  if (flight_mode > 15) flight_mode = 0;


}




