// *****************************************************************************
// *************** Code for Measuring Acceleration and Speed **************
//
// UPDATE!!! After espressif ESP32 Migration 2.x to 3.0 
// Syntax for Interrupt Timer changed
//
// 09.09.2024 Klaus S. from electronics unmessed on YouTube
//
// Speedo measures speed and acceleration using 3 infrared sensors
// it measures microseconds time steps using a pretty sophisticated interrupt routine
// that is able to work with multiple triggering by the sensors, which happens in
// real measurement scenarios
// a number of options controlled by serial commands: left, right, reports, ..
// some fun features controlling the LEDs are also included
// when paired, bluetooth works in parallel to serial interface
// *** 18.03.2024 ks
//
// This example code is in the Public Domain (or CC0 licensed, at your option.)
// By Evandro Copercini - 2018
//
// This example creates a bridge between Serial and Classical Bluetooth (SPP)
// and also demonstrate that SerialBT have the same functionalities of a normal Serial

#include "BluetoothSerial.h"

// #define USE_PIN // Uncomment this to use PIN during pairing. The pin is specified on the line below
// const char *pin = "7859";  // Change this to more secure PIN.

String device_name = "Speedo";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;
boolean confirmRequestPending = true;  // for PW

void BTConfirmRequestCallback(uint32_t numVal) {
  confirmRequestPending = true;
  Serial.println(numVal);
}

void BTAuthCompleteCallback(boolean success) {
  confirmRequestPending = false;
  if (success) {
    Serial.println("Pairing success!!");
  } else {
    Serial.println("Pairing failed, rejected by user!!");
  };
}

void BlueTooth_Pairing() {
  if (confirmRequestPending) {
    if (Serial.available()) {
      int dat = Serial.read();
      if (dat == 'Y' || dat == 'y') {
        SerialBT.confirmReply(true);
      } else {
        SerialBT.confirmReply(false);
        // Serial.print(".");
      };
    };
  } else {
    if (Serial.available()) {
      SerialBT.write(Serial.read());
    };
    if (SerialBT.available()) {
      Serial.write(SerialBT.read());
    };
    delay(20);
  };
}

#include <Arduino.h>
#include <Preferences.h>  // EEPROM beschreiben
Preferences pref;

String Version = "8.2";

String Direction = "left";
// String Top_Acc_Speed = "off";
String Event_Flash = "on";
String Report_Depth = "light";  // medium, full

String string_Ser_Input_1 = "";
String string_Ser_Input_2 = "";

const int startPin = 5;   // Start-Pin for ir sensors
const int endPin_1 = 18;  // End-Pin-1
const int endPin_2 = 19;  // End-Pin-2

const int L1_LED_Pin = 25;  // LED (On/Off)                 for measurement OK:        green (was 14)
const int L2_LED_Pin = 12;  // LED (On/Off)                 for measurement NOK:       red   (was 12)
const int L3_LED_Pin = 27;  // LED (accelerating flashing)  for new top speed:         ??
const int L4_LED_Pin = 26;  // LED (accelerating flashing)  for new top acceleration:  ??
const int L5_LED_Pin = 14;  // LED (blink/On)               for BlueTooth connection:  blue  (was 25)
const int L6_LED_Pin = 33;  // LED (On/Off)                 for direction left:        ??
const int L7_LED_Pin = 32;  // LED (On/Off)                 for direction right:       ??
const int L8_LED_Pin = 13;  // LED (event flash)            for ??

long LED_Timer_L3 = 0;
long LED_Timer_L3_high = 100;

long LED_Timer_L4 = 0;
long LED_Timer_L4_high = 100;

long LED_Timer_L5 = 0;
long LED_Timer_L5_high = 500;

long LED_Timer_L8 = 0;
long LED_Timer_L8_high = 100;

long delta_T_0 = 0;
long delta_T_1 = 0;
long delta_T_2 = 0;
long delta_T_3 = 0;

long rest_T_0 = 0;  // time to reset

long Lowest_Speed = 10;  // in cm/sec reset T_0, if next step missing

float velo_low = 0;

float T_1 = 0;
float T_2 = 0;

float acceleration = 0;
float velo_0 = 0;
float velo_1 = 0;
float velo_2 = 0;

float top_acc = 0;
float top_velo = 0;

// LEDs distance
float streck_1 = 0.0725;  // IR_Sens_0 to IR_Sens_1
float streck_2 = 0.145;   // IR_Sens_0 to IR_Sens_2

int print = 0;
int error_1 = 0;
int error_2 = 0;
int Err_1 = 0;
int Err_2 = 0;
int Err_3 = 0;
int Timing = 0;

long count = 0;
long count_old = 0;

// Initialisiere den Timer
hw_timer_t *timer = NULL;

// Interrupt routine

void IRAM_ATTR onTimer() {

  static uint64_t t_0 = 0;
  static uint64_t t_1 = 0;
  static uint64_t t_2 = 0;
  static uint64_t t_3 = 0;

  static uint64_t timing = 0;
  static uint64_t reset_t_0 = 0;

  static uint64_t error_1 = 1;
  static uint64_t error_2 = 0;
  static uint64_t error_3 = 0;

  static uint64_t lock_0 = 0;

  reset_t_0 = rest_T_0;

  if (Direction == "left") {

    if ((digitalRead(endPin_1) == LOW) && (lock_0 == 0)) {
      lock_0 = 1;
      t_3 = timerReadMicros(timer);
    };

    if (((timerReadMicros(timer) - t_3) > (0.9 * reset_t_0)) && (digitalRead(startPin) == LOW) && (timing == 0) && (digitalRead(endPin_1) == HIGH) && (digitalRead(endPin_2) == HIGH)) {
      timing = 1;
      t_0 = timerReadMicros(timer);
    };

    if (((timerReadMicros(timer) - t_0) > reset_t_0) && (timing == 1)) {
      timing = 0;
      lock_0 = 0;
    };

    if ((digitalRead(endPin_1) == LOW) && (timing == 1) && (digitalRead(endPin_2) == HIGH)) {
      timing = 2;
      t_1 = timerReadMicros(timer);
    };

    if (((timerReadMicros(timer) - t_1) > (1 * reset_t_0)) && (timing == 2)) {
      timing = 0;
    };

    if ((digitalRead(endPin_2) == LOW) && (timing == 2)) {
      timing = -3;
      t_2 = timerReadMicros(timer);
      t_3 = timerReadMicros(timer);
      delta_T_0 = t_0;
      delta_T_1 = t_1;
      delta_T_2 = t_2;
      Err_3 = error_3;
      print = 1;
      error_3 = 0;
      lock_0 = 0;
    };

    if (timing == -3) {
      timing = Timing;
    };

    if ((digitalRead(startPin) == LOW) && (digitalRead(endPin_1) == LOW) && (digitalRead(endPin_2) == LOW)) {
      error_3 = 1;
    };

  } else {
    // +++++++++ else !!
    if ((digitalRead(endPin_1) == LOW) && (lock_0 == 0)) {
      lock_0 = 1;
      t_3 = timerReadMicros(timer);
    };

    if (((timerReadMicros(timer) - t_3) > (0.9 * reset_t_0)) && (digitalRead(endPin_2) == LOW) && (timing == 0) && (digitalRead(endPin_1) == HIGH) && (digitalRead(startPin) == HIGH)) {
      timing = 1;
      t_0 = timerReadMicros(timer);
    };

    if (((timerReadMicros(timer) - t_0) > reset_t_0) && (timing == 1)) {
      timing = 0;
      lock_0 = 0;
    };

    if ((digitalRead(endPin_1) == LOW) && (timing == 1) && (digitalRead(startPin) == HIGH)) {
      timing = 2;
      t_1 = timerReadMicros(timer);
    };

    if (((timerReadMicros(timer) - t_1) > (1 * reset_t_0)) && (timing == 2)) {
      timing = 0;
    };

    if ((digitalRead(startPin) == LOW) && (timing == 2)) {
      timing = -3;
      t_2 = timerReadMicros(timer);
      t_3 = timerReadMicros(timer);
      delta_T_0 = t_0;
      delta_T_1 = t_1;
      delta_T_2 = t_2;
      Err_3 = error_3;
      print = 1;
      error_3 = 0;
      lock_0 = 0;
    };

    if (timing == -3) {
      timing = Timing;
    };

    if ((digitalRead(startPin) == LOW) && (digitalRead(endPin_1) == LOW) && (digitalRead(endPin_2) == LOW)) {
      error_3 = 1;
    };
  };
}

void setup() {

  pref.begin("Direction", false);                   //started preferences
  Direction = pref.getString("Direction", "left");  // saving value
  pref.end();

  pref.begin("Event_Flash", false);                   //started preferences
  Event_Flash = pref.getString("Event_Flash", "on");  // saving value
  pref.end();

  pref.begin("Report_Depth", false);                       //started preferences
  Report_Depth = pref.getString("Report_Depth", "light");  // saving value
  pref.end();

  pref.begin("velo_low", false);              //started preferences
  velo_low = pref.getFloat("velo_low", 0.1);  // saving value
  pref.end();

  pinMode(L1_LED_Pin, OUTPUT);
  pinMode(L2_LED_Pin, OUTPUT);
  pinMode(L3_LED_Pin, OUTPUT);
  pinMode(L4_LED_Pin, OUTPUT);
  pinMode(L5_LED_Pin, OUTPUT);
  pinMode(L6_LED_Pin, OUTPUT);
  pinMode(L7_LED_Pin, OUTPUT);
  pinMode(L8_LED_Pin, OUTPUT);

  pinMode(startPin, INPUT_PULLUP);
  pinMode(endPin_1, INPUT_PULLUP);
  pinMode(endPin_2, INPUT_PULLUP);

  // Konfiguriere den Timer
  // timer = timerBegin(0, 80, true);  // Timer 0, Teiler 80 (1 Mikrosekunde), Autoreload

  // ******** NEW NEW NEW Syntax ****************
  timer = timerBegin(1000000);  // New Syntax Version of espressif !!!!

  // timerAttachInterrupt(timer, &onTimer, true);  // Timer-Interrupt aktivieren
  // timerAlarmWrite(timer, 1000, true);           // Timer alle 1000 Mikrosekunden auslösen
  // timerAlarmEnable(timer);                      // Timer starten

  attachInterrupt(startPin, onTimer, FALLING);
  attachInterrupt(endPin_1, onTimer, FALLING);
  attachInterrupt(endPin_2, onTimer, FALLING);

  rest_T_0 = 100000000 * streck_1 / Lowest_Speed;
  velo_low = 0.01 * float(Lowest_Speed);

  Serial.begin(115200);
  // Serial.flush();
  SerialBT.enableSSP();
  SerialBT.onConfirmRequest(BTConfirmRequestCallback);
  SerialBT.onAuthComplete(BTAuthCompleteCallback);
  SerialBT.begin(device_name);  //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth PW!");
  Serial.print("Click on connect to '");
  Serial.print(device_name);
  Serial.println("' confirm identical code with 'y'");
  // Serial.flush();
}

// ********** MAIN LOOP ******************************

void loop() {

  BlueTooth_LED();
  Direction_LED();

  if (Event_Flash == "on") {
    Top_Speed_LED();
    Top_Acc_LED();
    Flash_LED();
  };

  BlueTooth_Pairing();

  Read_Ser();
  Read_SerBT();

  if (count == 0) {
    count = count + 1;
    Serial_Version_Parameter();
    Serial_Print_Settings();
    SerialBT_Version_Parameter();
    SerialBT_Print_Settings();
  };

  if (print == 1) {

    Velo_Acc();
    Err_Test();

    if (Report_Depth == "light") {
      Top_Speed_Acc_Simple();
      Serial_Print_Valid();
      Serial_Print_Simple();
      SerialBT_Print_Valid();
      SerialBT_Print_Simple();
    } else {
      Top_Speed_Acc();
      Serial_Print_Valid();
      Serial_Print();
      SerialBT_Print_Valid();
      SerialBT_Print();
    };

    count = count + 1;

    if (Report_Depth == "full") {
      Serial_Print_Times();
      Serial_Print_Error();
      SerialBT_Print_Times();
      SerialBT_Print_Error();
    };

    delayMicroseconds(10);

    print = 0;
    Err_1 = 0;
    Err_2 = 0;
    Err_3 = 0;
  };
}

void Reset_Parameter() {
  Direction = "left";
  pref.begin("Direction", false);          //started preferences with slot_length namespace
  pref.putString("Direction", Direction);  // saving value
  pref.end();
  pref.begin("Direction", false);                   //started preferences
  Direction = pref.getString("Direction", "left");  // getting value
  pref.end();

  Report_Depth = "light";
  pref.begin("Report_Depth", false);             //started preferences with slot_length namespace
  pref.putString("Report_Depth", Report_Depth);  // saving value
  pref.end();
  pref.begin("Report_Depth", false);                       //started preferences
  Report_Depth = pref.getString("Report_Depth", "light");  // getting value
  pref.end();

  Event_Flash = "on";
  pref.begin("Event_Flash", false);            //started preferences with slot_length namespace
  pref.putString("Event_Flash", Event_Flash);  // saving value
  pref.end();
  pref.begin("Event_Flash", false);                   //started preferences
  Event_Flash = pref.getString("Event_Flash", "on");  // getting value
  pref.end();

  count = 0;
  top_velo = 0;
  top_acc = 0;
}

void Reset_Top() {
  count = 0;
  top_velo = 0;
  top_acc = 0;
}

void Read_Ser() {
  // Serial.flush();
  if (Serial.available()) {

    string_Ser_Input_1 = Serial.readStringUntil('=');
    // string_Ser_Input_2 = Serial.readStringUntil('\n');
    string_Ser_Input_2 = Serial.readStringUntil(';');

    // +++ BT function is eating the first character .. so I need to bring it back

    if (string_Ser_Input_1 == "irection") {
      string_Ser_Input_1 = "direction";
    };
    if (string_Ser_Input_1 == "eport") {
      string_Ser_Input_1 = "report";
    };
    if (string_Ser_Input_1 == "etting") {
      string_Ser_Input_1 = "setting";
    };
    if (string_Ser_Input_1 == "lash") {
      string_Ser_Input_1 = "flash";
    };
    if (string_Ser_Input_1 == "eset") {
      string_Ser_Input_1 = "reset";
    };

    // +++++++


    if (string_Ser_Input_1 == "direction") {
      Direction = string_Ser_Input_2;
      pref.begin("Direction", false);          //started preferences with slot_length namespace
      pref.putString("Direction", Direction);  // saving value
      pref.end();
      pref.begin("Direction", false);                   //started preferences with slot_length namespace
      Direction = pref.getString("Direction", "left");  // getting value
      pref.end();
      Serial_Print_Settings();
      // Serial.flush();
    };

    if (string_Ser_Input_1 == "report") {
      Report_Depth = string_Ser_Input_2;
      pref.begin("Report_Depth", false);             //started preferences with slot_length namespace
      pref.putString("Report_Depth", Report_Depth);  // saving value
      pref.end();
      pref.begin("Report_Depth", false);                       //started preferences with slot_length namespace
      Report_Depth = pref.getString("Report_Depth", "light");  // getting value
      pref.end();
      Serial_Print_Settings();
      // Serial.flush();
    };

    if (string_Ser_Input_1 == "flash") {
      Event_Flash = string_Ser_Input_2;
      pref.begin("Event_Flash", false);            //started preferences with slot_length namespace
      pref.putString("Event_Flash", Event_Flash);  // saving value
      pref.end();
      pref.begin("Event_Flash", false);                   //started preferences with slot_length namespace
      Event_Flash = pref.getString("Event_Flash", "on");  // getting value
      pref.end();
      Serial_Print_Settings();
      // Serial.flush();
    };

    if ((string_Ser_Input_1 == "setting") && (string_Ser_Input_2 == "show")) {
      Serial_Print_Settings();
      //Serial.flush();
    };

    if ((string_Ser_Input_1 == "reset") && (string_Ser_Input_2 == "all")) {
      Serial.println();
      Serial.println("All parameters are set to default! ");
      Serial.println("Top values are reset! ");

      Reset_Parameter();
      // Serial.flush();
    };

    if ((string_Ser_Input_1 == "reset") && (string_Ser_Input_2 == "top")) {
      Reset_Top();
      Serial.println();
      Serial.println("Top values are reset! ");
      //Serial.flush();
    };

    if (((string_Ser_Input_1 != "direction") && (string_Ser_Input_1 != "report")) && (string_Ser_Input_1 != "setting") && (string_Ser_Input_1 != "reset") && (string_Ser_Input_1 != "flash")) {
      Serial.println(string_Ser_Input_1);
      // Serial.println("invalid command! ");
      Serial.println("valid commands: ");
      Serial.println();
      Serial.println("Setting:      setting=show; ");
      Serial.println("Direction:    direction=left; direction=right;                   (saved permanently)");
      Serial.println("Report:       report=light; report=medium; report=full;          (saved permanently)");
      Serial.println("Flash LED:    flash=on; flash=off;                               (saved permanently)");
      Serial.println("Reset:        top values: reset=top;   defaults: reset=all;                         ");

      Serial.print("BlueTooth:    connect '");
      Serial.print(device_name);
      Serial.println("', then confirm PW with 'y' (PW saved permanently)");

      Serial.println();
      Serial.flush();
      string_Ser_Input_1 = "";
      string_Ser_Input_2 = "";
    };
  };
}

void Read_SerBT() {
  // SerialBT.flush();
  if (SerialBT.available()) {

    string_Ser_Input_1 = SerialBT.readStringUntil('=');
    // string_Ser_Input_2 = Serial.readStringUntil('\n');
    string_Ser_Input_2 = SerialBT.readStringUntil(';');

    if (string_Ser_Input_1 == "direction") {
      Direction = string_Ser_Input_2;
      pref.begin("Direction", false);          //started preferences with slot_length namespace
      pref.putString("Direction", Direction);  // saving value
      pref.end();
      pref.begin("Direction", false);                   //started preferences with slot_length namespace
      Direction = pref.getString("Direction", "left");  // getting value
      pref.end();
      SerialBT_Print_Settings();
    };

    if (string_Ser_Input_1 == "report") {
      Report_Depth = string_Ser_Input_2;
      pref.begin("Report_Depth", false);             //started preferences with slot_length namespace
      pref.putString("Report_Depth", Report_Depth);  // saving value
      pref.end();
      pref.begin("Report_Depth", false);                       //started preferences with slot_length namespace
      Report_Depth = pref.getString("Report_Depth", "light");  // getting value
      pref.end();
      SerialBT_Print_Settings();
    };

    if (string_Ser_Input_1 == "flash") {
      Event_Flash = string_Ser_Input_2;
      pref.begin("Event_Flash", false);            //started preferences with slot_length namespace
      pref.putString("Event_Flash", Event_Flash);  // saving value
      pref.end();
      pref.begin("Event_Flash", false);                   //started preferences with slot_length namespace
      Event_Flash = pref.getString("Event_Flash", "on");  // getting value
      pref.end();
      SerialBT_Print_Settings();
    };

    if ((string_Ser_Input_1 == "setting") && (string_Ser_Input_2 == "show")) {
      SerialBT_Print_Settings();
    };

    if ((string_Ser_Input_1 == "reset") && (string_Ser_Input_2 == "all")) {

      SerialBT.println("All parameters are set to default! ");
      SerialBT.println("Top values are reset! ");
      Reset_Parameter();
      // SerialBT_Print_Settings();
    };

    if ((string_Ser_Input_1 == "reset") && (string_Ser_Input_2 == "top")) {
      Reset_Top();
      SerialBT.println("Top values are reset! ");
    };

    if (((string_Ser_Input_1 != "direction") && (string_Ser_Input_1 != "report")) && (string_Ser_Input_1 != "setting") && (string_Ser_Input_1 != "reset") && (string_Ser_Input_1 != "flash")) {
      SerialBT.println();
      // SerialBT.println("invalid command! ");
      SerialBT.println("valid commands: ");
      SerialBT.println();
      SerialBT.println("Setting:      setting=show; ");
      SerialBT.println("Direction:    direction=left; direction=right;                   (saved permanently)");
      SerialBT.println("Report:       report=light; report=medium; report=full;          (saved permanently)");
      SerialBT.println("Flash LED:    flash=on; flash=off;                               (saved permanently)");
      SerialBT.println("Reset:        top values: reset=top;   defaults: reset=all;      (saved permanently)");

      SerialBT.print("BlueTooth:    connect '");
      SerialBT.print(device_name);
      SerialBT.println("', then confirm PW with 'y' (PW saved permanently)");

      SerialBT.println();
      SerialBT.flush();
      string_Ser_Input_1 = "";
      string_Ser_Input_2 = "";
    };
  };
}

void Err_Test() {
  if (((delta_T_2 - delta_T_1) < 1000) || ((delta_T_1 - delta_T_0) < 1000)) {
    Err_1 = 1;
  };
  if ((velo_0 < 0) || (velo_1 < 0) || (velo_2 < 0)) {
    Err_2 = 1;
  };
}

void Velo_Acc() {
  T_1 = float(delta_T_1 - delta_T_0);
  T_2 = float(delta_T_2 - delta_T_0);
  T_1 = 0.000001 * T_1;
  T_2 = 0.000001 * T_2;
  acceleration = 2 * ((streck_2 / T_2) - (streck_1 / T_1)) / (T_2 - T_1);
  velo_0 = (streck_2 / T_2) - 0.5 * acceleration * T_2;
  velo_2 = velo_0 + acceleration * T_2;
  velo_1 = velo_0 + acceleration * T_1;
}

void Top_Speed_Acc() {
  if ((abs(acceleration) > abs(top_acc)) && (Err_1 == 0) && (Err_2 == 0)) {
    top_acc = acceleration;
    Serial_Print_Top_Acc();
    SerialBT_Print_Top_Acc();
    // LED_Timer_L3_high = 200;
  };
  if ((velo_0 > top_velo) && (Err_1 == 0) && (Err_2 == 0)) {
    top_velo = velo_0;
    Serial_Print_Top_Speed();
    SerialBT_Print_Top_Speed();
  };
  if ((velo_1 > top_velo) && (Err_1 == 0) && (Err_2 == 0)) {
    top_velo = velo_1;
    Serial_Print_Top_Speed();
    SerialBT_Print_Top_Speed();
  };
  if ((velo_2 > top_velo) && (Err_1 == 0) && (Err_2 == 0)) {
    top_velo = velo_2;
    Serial_Print_Top_Speed();
    SerialBT_Print_Top_Speed();
  };
}

void Top_Speed_Acc_Simple() {

  if ((abs(acceleration) > abs(top_acc)) && (Err_1 == 0) && (Err_2 == 0)) {
    top_acc = acceleration;
    Serial_Print_Top_Acc();
    SerialBT_Print_Top_Acc();
  };
  if ((velo_2 > top_velo) && (Err_1 == 0) && (Err_2 == 0)) {
    top_velo = velo_2;
    Serial_Print_Top_Speed();
    SerialBT_Print_Top_Speed();
  };
}

void Serial_Print_Settings() {
  Serial.println();
  Serial.print("Settings:     ");
  Serial.print("Direction: ");
  Serial.print(Direction);
  Serial.print(" ; ");
  Serial.print("Report: ");
  Serial.print(Report_Depth);
  Serial.print(" ; ");
  Serial.print("LED Flash: ");
  Serial.println(Event_Flash);
  Serial.println();
}

void SerialBT_Print_Settings() {
  SerialBT.println();
  SerialBT.print("Settings:     ");
  SerialBT.print("Direction: ");
  SerialBT.print(Direction);
  SerialBT.print(" ; ");
  SerialBT.print("Report: ");
  SerialBT.print(Report_Depth);
  SerialBT.print(" ; ");
  SerialBT.print("LED Flash: ");
  SerialBT.println(Event_Flash);
  SerialBT.println();
}

void Serial_Print_Times() {
  Serial.print("Time Report:  ");
  Serial.print("T_0 ");
  Serial.print(delta_T_0);
  Serial.print(" microsec ; ");
  Serial.print("T_1 ");
  Serial.print(delta_T_1);
  Serial.print(" microsec ; ");
  Serial.print("T_2 ");
  Serial.print(delta_T_2);
  Serial.println(" microsec ; ");
}

void SerialBT_Print_Times() {
  SerialBT.print("Time Report:  ");
  SerialBT.print("T_0 ");
  SerialBT.print(delta_T_0);
  SerialBT.print(" microsec ; ");
  SerialBT.print("T_1 ");
  SerialBT.print(delta_T_1);
  SerialBT.print(" microsec ; ");
  SerialBT.print("T_2 ");
  SerialBT.print(delta_T_2);
  SerialBT.println(" microsec ; ");
}

void Serial_Print_Error() {
  Serial.print("Error Report: ");
  Serial.print("Err: ");
  Serial.print(Err_1);
  Serial.print(" ; ");
  Serial.print(Err_2);
  Serial.print(" ; ");
  Serial.print(Err_3);
  Serial.println(" ; ");
}

void SerialBT_Print_Error() {
  SerialBT.print("Error Report: ");
  SerialBT.print("Err: ");
  SerialBT.print(Err_1);
  SerialBT.print(" ; ");
  SerialBT.print(Err_2);
  SerialBT.print(" ; ");
  SerialBT.print(Err_3);
  SerialBT.println(" ; ");
}

void Serial_Version_Parameter() {
  Serial.flush();
  Serial.println();
  Serial.print("Version:      Speedo ");
  Serial.println(Version);
  Serial.println();
  Serial.print("Parameters:   ");
  Serial.print("S_1: ");
  Serial.print(streck_1 * 1000);
  Serial.print(" mm ; ");
  Serial.print("S_2: ");
  Serial.print(streck_2 * 1000);
  Serial.print(" mm ; ");
  Serial.print("v_low: ");
  Serial.print(velo_low);
  Serial.println(" m/sec ; ");
}

void SerialBT_Version_Parameter() {
  SerialBT.flush();
  SerialBT.println();
  SerialBT.print("Version:      Speedo ");
  SerialBT.println(Version);
  SerialBT.println();
  SerialBT.print("Parameters:   ");
  SerialBT.print("S_1: ");
  SerialBT.print(streck_1 * 1000);
  SerialBT.print(" mm ; ");
  SerialBT.print("S_2: ");
  SerialBT.print(streck_2 * 1000);
  SerialBT.print(" mm ; ");
  SerialBT.print("v_low: ");
  SerialBT.print(velo_low);
  SerialBT.println(" m/sec ; ");
}

void Serial_Print_Valid() {
  if ((Err_1 == 1) || (Err_2 == 1)) {
    NOK_LED();
    Serial.print("nok ");
  } else {
    OK_LED();
    Serial.print(" ok ");
  };
}

void SerialBT_Print_Valid() {
  if ((Err_1 == 1) || (Err_2 == 1)) {
    SerialBT.print("nok ");
  } else {
    SerialBT.print(" ok ");
  };
}

void Serial_Print() {
  Serial.flush();
  Serial.print("Report:   ");
  Serial.print("No: ");
  Serial.print(count);
  Serial.print(" ; v_0: ");
  Serial.print(velo_0);
  Serial.print(" m/sec  v_1: ");
  Serial.print(velo_1);
  Serial.print(" m/sec  v_2: ");
  Serial.print(velo_2);
  Serial.print(" m/sec  acc: ");
  Serial.print(acceleration);
  Serial.println(" m/sec^2 ");
  Timing = 0;
}

void SerialBT_Print() {
  SerialBT.flush();
  SerialBT.print("Report:   ");
  SerialBT.print("No: ");
  SerialBT.print(count);
  SerialBT.print(" ; v_0: ");
  SerialBT.print(velo_0);
  SerialBT.print(" m/sec  v_1: ");
  SerialBT.print(velo_1);
  SerialBT.print(" m/sec  v_2: ");
  SerialBT.print(velo_2);
  SerialBT.print(" m/sec  acc: ");
  SerialBT.print(acceleration);
  SerialBT.println(" m/sec^2 ");
  Timing = 0;
}

void Serial_Print_Simple() {
  Serial.flush();
  Serial.print("Report:   ");
  Serial.print("No: ");
  Serial.print(count);
  Serial.print("  v_2: ");
  Serial.print(velo_2);
  Serial.print(" m/sec  acc: ");
  Serial.print(acceleration);
  Serial.print(" m/sec^2 ");

  Serial.print("    v_2: ");
  Serial.print(velo_2 * 3.6);
  Serial.print(" km/h  acc: ");
  Serial.print(acceleration / 9.81);
  Serial.println(" g ");


  Timing = 0;
}

void SerialBT_Print_Simple() {
  SerialBT.flush();
  SerialBT.print("Report:   ");
  SerialBT.print("No: ");
  SerialBT.print(count);
  SerialBT.print("  v_2: ");
  SerialBT.print(velo_2);
  SerialBT.print(" m/sec  acc: ");
  SerialBT.print(acceleration);
  SerialBT.print(" m/sec^2 ");

  SerialBT.print("    v_2: ");
  SerialBT.print(velo_2 * 3.6);
  SerialBT.print(" km/h  acc: ");
  SerialBT.print(acceleration / 9.81);
  SerialBT.println(" g ");

  Timing = 0;
}

void Serial_Print_Top_Speed() {
  Serial.print("Top Report:   ");
  Serial.print("top_v: ");
  Serial.print(top_velo);
  Serial.print(" m/sec ");
  Serial.println();
  LED_Timer_L4_high = 200;
  //Top_Speed_LED();
}

void SerialBT_Print_Top_Speed() {
  SerialBT.print("Top Report:   ");
  SerialBT.print("top_v: ");
  SerialBT.print(top_velo);
  SerialBT.print(" m/sec ");
  SerialBT.println();
}

void Serial_Print_Top_Acc() {
  Serial.print("Top Report:   ");
  Serial.print("top_acc: ");
  Serial.print(top_acc);
  Serial.print(" m/sec^2  ");
  Serial.println();

  LED_Timer_L3_high = 200;
}

void SerialBT_Print_Top_Acc() {
  SerialBT.print("Top Report:   ");
  SerialBT.print("top_acc: ");
  SerialBT.print(top_acc);
  SerialBT.print(" m/sec^2  ");
  SerialBT.println();
}

void OK_LED() {
  digitalWrite(L1_LED_Pin, HIGH);
  digitalWrite(L2_LED_Pin, LOW);
}

void NOK_LED() {
  digitalWrite(L1_LED_Pin, LOW);
  digitalWrite(L2_LED_Pin, HIGH);
}


void Top_Acc_LED() {

  if ((millis() - LED_Timer_L3) > LED_Timer_L3_high) {
    LED_Timer_L3 = millis();
    digitalWrite(L3_LED_Pin, HIGH);
    LED_Timer_L3_high = LED_Timer_L3_high - 10;
  };

  if ((millis() - LED_Timer_L3) > (0.5 * LED_Timer_L3_high)) {
    digitalWrite(L3_LED_Pin, LOW);
  };
}

void Top_Speed_LED() {

  if ((millis() - LED_Timer_L4) > LED_Timer_L4_high) {
    LED_Timer_L4 = millis();
    digitalWrite(L4_LED_Pin, HIGH);
    LED_Timer_L4_high = LED_Timer_L4_high - 10;
  };

  if ((millis() - LED_Timer_L4) > (0.5 * LED_Timer_L4_high)) {
    digitalWrite(L4_LED_Pin, LOW);
  };
}

void BlueTooth_LED() {
  if (SerialBT.hasClient()) {
    digitalWrite(L5_LED_Pin, HIGH);
  } else {
    if ((millis() - LED_Timer_L5) > LED_Timer_L5_high) {
      LED_Timer_L5 = millis();
      digitalWrite(L5_LED_Pin, HIGH);
    };
    if ((millis() - LED_Timer_L5) > (0.5 * LED_Timer_L5_high)) {
      digitalWrite(L5_LED_Pin, LOW);
    };
  };
}

void Direction_LED() {
  // L6 = left
  if (Direction == "left") {
    digitalWrite(L6_LED_Pin, HIGH);
    digitalWrite(L7_LED_Pin, LOW);
  };

  if (Direction == "right") {
    digitalWrite(L6_LED_Pin, LOW);
    digitalWrite(L7_LED_Pin, HIGH);
  };
}

void Flash_LED() {

  if (count != count_old) {
    LED_Timer_L8 = millis();
  };

  count_old = count;

  if ((millis() - LED_Timer_L8) < LED_Timer_L8_high) {
    // LED_Timer_L8 = millis();
    digitalWrite(L8_LED_Pin, HIGH);
  };
  if ((millis() - LED_Timer_L8) > LED_Timer_L8_high) {
    digitalWrite(L8_LED_Pin, LOW);
  };
}
