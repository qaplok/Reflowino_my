#include <Arduino.h>

// Web Updater
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>

#include <WiFiUdp.h>
#include <Syslog.h>

#include "config.h"
#include <melody_player.h>
#include <melody_factory.h>

#include <math.h> // for log()
#include <GyverPortal.h>
#include <Ticker.h>

#ifndef VERSION
  #define VERSION   NAME " 1.0 " __DATE__ " " __TIME__
#endif


GyverPortal ui;
Ticker tick_time;


uint16_t _duty = 100;               // ssr pwm percent. Make oven useful without WLAN
bool _fixed_duty = true;            // true: decouple from temperature control

// Analog read samples
const uint16_t A_samples = A_SAMPLES;
const uint16_t A_max = A_MAX;
uint32_t _a_sum = 0;                // sum of last analog reads

// NTC characteristics (datasheet)
static const uint32_t B = NTC_B;
static const uint32_t R_n = NTC_R_N; // Ohm
static const uint32_t T_n = NTC_T_N; // Celsius

static const uint32_t R_v = NTC_R_V; // Ohm, voltage divider resistor for NTC

uint32_t _r_ntc = 0;                // Ohm, resistance updated with each analog read
double _temp_c = 0;                 // Celsius, calculated from NTC and R_v
String _str_temp_c;
uint16_t _temp_target = 250;      // adjust _duty to reach this temperature
uint16_t _time_target = 60;
uint16_t _timer;  
String _str_timer;  
uint16_t _secunde = 0;    
String _str_secunde;
bool _tic_state = false;
bool _tic_state_old;
// PID stuff
double _pid_kp = PID_K_P;
double _pid_ki = PID_K_I;
double _pid_kd = PID_K_D;
bool end;
bool _Start_oven = false;
bool _Ten1 = true;
bool _Ten2 = true;
bool _the_end_of_coock = false;


MelodyPlayer player1(HAS_BEEPER_1);

#ifdef ESP32
MelodyPlayer player2(HAS_BEEPER_2, 2);
#else
MelodyPlayer player2(HAS_BEEPER_2);
#endif

// Temperature history
int16_t _t[8640];      // 1 day centicelsius temperature history in 10s intervals
uint16_t _t_pos;

void build();
void action();


void build() {
  GP.BUILD_BEGIN();
  GP.THEME(GP_DARK);

  GP.UPDATE("Temp_now, Timer_now, Minute_now, swch1, Timer, Temperature, Ten1, Ten2");

  GP.BOX_BEGIN(GP_CENTER);
    GP.LABEL("Вкл/Выкл");
    GP.SWITCH("swch1", _Start_oven, GP_RED);
    GP.LABEL_BLOCK(_str_temp_c, "Temp_now", GP_RED_B, 24);
  GP.BOX_END();

  GP.HR();
  GP.BOX_BEGIN(GP_CENTER);  
    GP.LABEL_BLOCK(_str_timer, "Timer_now", GP_GREEN_B, 44);
    GP.LABEL_BLOCK(":", "Razdel", GP_GREEN_B, 44);
    GP.LABEL_BLOCK(_str_secunde, "Minute_now", GP_GREEN_B, 44);
  GP.BOX_END();
  GP.HR(); 
  GP.LABEL("Установка таймера в минутах");
  GP.BOX_BEGIN(GP_CENTER);
    GP.BUTTON("btn1", "-");
    GP.NUMBER_F("Timer", "Timer", _time_target, 0, "100px");
    GP.BUTTON("btn2", "+");
  GP.BOX_END();   
  
  GP.BREAK(); 
  GP.HR(); 
  GP.LABEL("Установка температуры в град. Цельсия");

  GP.BOX_BEGIN(GP_CENTER);
    GP.BUTTON("btn3", "-");
    GP.NUMBER_F("Temperature", "Temp", _temp_target, 0, "100px");
    GP.BUTTON("btn4", "+");
  GP.BOX_END(); 

  GP.BREAK(); 
  GP.HR(); 

  GP.BOX_BEGIN(GP_CENTER);
    GP.LABEL("Верхний тэн");
    GP.SWITCH("Ten1", _Ten1, GP_RED);
  GP.BOX_END();   

  GP.BOX_BEGIN(GP_CENTER);
    GP.LABEL("Нижний тэн");
    GP.SWITCH("Ten2", _Ten2, GP_RED);
  GP.BOX_END(); 

  GP.BUILD_END();
}

// Initiate connection to Wifi but dont wait for it to be established
void setup_Wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.hostname(NAME);
  WiFi.begin(AP_SSID, AP_PASS);
  
  //digitalWrite(ONLINE_LED_PIN, HIGH);
  
}

void changeState()
{
  _tic_state = !_tic_state;//Invert Current State 
}

void handleTemp() {
  if (_temp_c > _temp_target + 1 ){
    if (_Ten1){
      digitalWrite(SWITCH_PIN_1, LOW);
    }
    if (_Ten2){
      digitalWrite(SWITCH_PIN_2, LOW);
    }
  } 
  if (_temp_c < _temp_target - 1 ){
    if (_Ten1){
      digitalWrite(SWITCH_PIN_1, HIGH);
    }
    if (_Ten2){
      digitalWrite(SWITCH_PIN_2, HIGH);
    }
  } 
  if (!_Ten1){
      digitalWrite(SWITCH_PIN_1, LOW);
    }
    if (!_Ten2){
      digitalWrite(SWITCH_PIN_2, LOW);
    }
}

void updateTemperature( const uint32_t r_ntc, double &temp_c ) {
  // only print if measurement decimals change 
  static int16_t t_prev = 0;

  // if( _fixed_duty ) {
  temp_c = 1.0 / (1.0/(273.15+T_n) + log((double)r_ntc/R_n)/B) - 273.15;
  //temp_c = 1.0 / (1.0/(273.15+T_n) + log((double)r_ntc/R_n)/B) - 273.15;
  // }
  // else {
  //   simulate_temp(temp_c);
  // }

  int16_t temp = (int16_t)(temp_c * 100 + 0.5); // rounded centi celsius
  if( (temp - t_prev) * (temp - t_prev) >= 10 * 10 ) { // only report changes >= 0.1
    // Serial.printf("Temperature: %01d.%01d degree Celsius\n", temp/100, (temp/10)%10);
    t_prev = temp;
  }
}


void updateResistance( const uint32_t a_sum, uint32_t &r_ntc, double &temp_c ) {
  static const uint32_t Max_sum = (uint32_t)A_max * A_samples;

  r_ntc = (int64_t)R_v * a_sum / (Max_sum - a_sum);

  updateTemperature(r_ntc, temp_c);
}


void handleAnalog( uint32_t &a_sum, uint32_t &r_ntc, double &temp_c ) {
  static uint16_t a[A_samples] = { 0 }; // last analog reads
  static uint16_t a_pos = A_samples;    // sample index

  if( millis() % 40 > 10 ) { // now and then release analog for wifi //было 10
    int value = analogRead(A0);
    
    // first time init
    if( a_pos == A_samples ) {
      while( a_pos-- ) {
        a[a_pos] = (uint16_t)value;
        a_sum += (uint32_t)value;
      }
    }
    else {
      if( ++a_pos >= A_samples ) {
        a_pos = 0;
      }

      a_sum -= a[a_pos];
      a[a_pos] = (uint16_t)value;
      a_sum += a[a_pos];
    }

    updateResistance(a_sum, r_ntc, temp_c);
  }
}


void handleFrequency() {
  static uint32_t start = 0;
  static uint32_t count = 0;

  count++;

  uint32_t now = millis();
  if( now - start > 1000 ) {
    printf("Measuring analog at %u Hz\n", count);
    start = now;
    count = 0;
  }
}

void play_sound(){
  //_---------------------------------SOUND
  String notes1[] = { "C4", "G3", "G3", "A3", "G3", "SILENCE", "B3", "C4" };
  Melody melody1 = MelodyFactory.load("Nice Melody", 250, notes1, 8); //250 8
  Melody melody2 = MelodyFactory.load("Nice Melody", 250, notes1, 8);
    //------------------------------------/SOUND
  player1.playAsync(melody1);
  player2.playAsync(melody2);
  end = false;
  digitalWrite(HAS_BEEPER_1, LOW);
  digitalWrite(HAS_BEEPER_2, LOW);
}

void setup() {
  analogWriteRange(255);//8 Bit
  analogWriteFreq(32000);
  // start with switch off:
  pinMode(SWITCH_PIN_1, OUTPUT);
  digitalWrite(SWITCH_PIN_1, LOW);
  pinMode(SWITCH_PIN_2, OUTPUT);
  digitalWrite(SWITCH_PIN_2, LOW);
  pinMode(HAS_BEEPER_1, OUTPUT);
  digitalWrite(HAS_BEEPER_1, LOW);
  pinMode(HAS_BEEPER_2, OUTPUT);
  digitalWrite(HAS_BEEPER_2, LOW);
  //pinMode(D3, OUTPUT);
  //digitalWrite(D3, LOW);
  Serial.begin(115200);
  pinMode(ONLINE_LED_PIN, OUTPUT);
  digitalWrite(ONLINE_LED_PIN, LOW);
  _tic_state_old = _tic_state;
  // Initiate network connection (but dont wait for it)
  setup_Wifi();
  // подключаем конструктор и запускаем
  ui.attachBuild(build);
  ui.attach(action);
  ui.start();
  tick_time.attach(1, changeState);
  Serial.println("\nBooted " VERSION);
//A3 A5 B1 B1 ___ B1 B2 B3 B3 ___ B3 B4 B2 B2 ___ B1 A5 A5 B1;
//A3 A5 B1 B1 ___ B1 B2 B3 B3 ___ B3 B4 B2 B2 ___ B1 A5 B1;
//A3 A5 B1 B1 ___ B1 B3 B4 B4 ___ B4 B5 C1 C1 ___ B5 B4 B5 B1;
//B1 B1 B2 B3 ___ B3 B4 B5 B1 ___ B3 B4 B2 B2 ___ B3 B4 B2 B3.

  //String notes1[] = { "C4", "G3", "G3", "A3", "G3", "SILENCE", "B3", "C4" };
  String notes1[] = { "B3", "B5", "B3", "B5", "SILENCE", "B3", "B5", "B3", "B5", "SILENCE"};
  //                    "B3", "B4", "B2", "B2", "SILENCE", "B1", "A5", "A5", "B1", "SILENCE"};
  //                    "A3", "A5", "B1", "B1", "SILENCE", "B1", "B2", "B3", "B3", "SILENCE",
  //                    "B3", "B4", "B2", "B2", "SILENCE", "B1", "A5", "B1", "SILENCE", "SILENCE",
  //                    "A3", "A5", "B1", "B1", "SILENCE", "B1", "B3", "B4", "B4", "SILENCE",
  //                    "B4", "B5", "C1", "C1", "SILENCE", "B5", "B4", "B5", "B1", "SILENCE",
  //                    "B1", "B1", "B2", "B3", "SILENCE", "B3", "B4", "B5", "B1", "SILENCE",
  //                    "B3", "B4", "B2", "B2", "SILENCE", "B3", "B4", "B2", "B3", "SILENCE"};
  
  Melody melody1 = MelodyFactory.load("Nice Melody", 10, notes1, 10); //250 8
  
  Melody melody2 = MelodyFactory.load("Nice Melody", 10, notes1, 10);
  //int notes2[] = { 500, 1000, 0, 2000 };
  //Melody melody2 = MelodyFactory.load("Raw frequencies", 400, notes2, 4);
  Serial.println("Done!");

  Serial.print("Start playing... ");
  player1.playAsync(melody1);
  player2.playAsync(melody2);
  end = false;
  digitalWrite(HAS_BEEPER_1, LOW);
  digitalWrite(HAS_BEEPER_2, LOW);

  _timer = 0;
}
bool holdFlag2;  // флаг удержания второй кнопки

void action() {
  
  if (ui.update()) {
    ui.updateString("Temp_now", _str_temp_c); 
    ui.updateString("Timer_now", _str_timer);
    ui.updateString("Minute_now", _str_secunde);
    
    ui.updateBool("swch1", _Start_oven);
    ui.updateBool("Ten1", _Ten1);
    ui.updateBool("Ten2", _Ten2);
    ui.updateInt("Timer", _time_target);
    ui.updateInt("Temperature", _temp_target);

  }

  if (ui.click("btn1")) {
      _time_target -= 10;
      if (_time_target < 0) {
        _time_target = 0;
      }
  }
  
  if (ui.click("btn2")) {
      _time_target += 10;
      if (_time_target > 300) {
        _time_target = 300;
      }
  }
   if (ui.click("btn3")) {
      _temp_target -= 10;
      if (_temp_target < 0) {
        _temp_target = 0;
      }
  }
  
  if (ui.click("btn4")) {
      _temp_target += 10;
      if (_temp_target > 320) {
        _temp_target = 320;
      }
  }
  if (ui.clickBool("swch1", _Start_oven)) {
      
    }
  if (ui.clickBool("Ten1", _Ten1)) {
     
    }
  if (ui.clickBool("Ten2", _Ten2)) {
     
    }

}

void loop() {
  // handleFrequency();
  ui.tick();
  if (!_Start_oven){
    _timer = 0;
    _secunde = 0;
    digitalWrite(SWITCH_PIN_1, LOW);
    digitalWrite(SWITCH_PIN_2, LOW);
    digitalWrite(ONLINE_LED_PIN, LOW); // turn Lamp off
  }

  handleAnalog(_a_sum, _r_ntc, _temp_c); 
  
  _str_temp_c = String(_temp_c);
  _str_timer = String(_timer);
  _str_secunde = String(_secunde);
  if (_time_target == 0 || _temp_target == 0){
      _Start_oven = false;
    }

  if (_Start_oven){
    
    if (_timer == 0){
    _timer = _time_target;
    }
  handleTemp();
  if (_secunde == 0){
    _timer -= 1;
    _secunde = 59;
    
  }
  if (_tic_state != _tic_state_old){
  _secunde -= 1;
  _tic_state_old = _tic_state;
  
    if (_tic_state_old){digitalWrite(ONLINE_LED_PIN, HIGH);}
    if (!_tic_state_old){digitalWrite(ONLINE_LED_PIN, LOW);}
  }
  }
  //handleWifi();
  if (_Start_oven and _timer == 0){
    _the_end_of_coock = true;
  }
  if (_the_end_of_coock){
    play_sound();
    _the_end_of_coock = false;
    _Start_oven = false;
    digitalWrite(SWITCH_PIN_1, LOW);
    digitalWrite(SWITCH_PIN_2, LOW);
    digitalWrite(ONLINE_LED_PIN, LOW);
  }
  //_secunde += 1;  //proverka
  delay(1);
}