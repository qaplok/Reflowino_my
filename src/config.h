#ifndef CONFIG_H
#define CONFIG_H

// Network stuff, might already be defined by the build tools
#ifndef AP_SSID
  #define AP_SSID      "QWww"
#endif
#ifndef AP_PASS
  #define AP_PASS      "9498498499"
#endif
#ifndef NAME
  #define NAME      "Q_Oven"
#endif
#ifndef PORT
  #define PORT      80
#endif

// Syslog server connection info
#define SYSLOG_SERVER "192.168.0.138"
#define SYSLOG_PORT 514

#define ONLINE_LED_PIN   2

// Switch stuff
#define SWITCH_PIN_1       15
#define SWITCH_PIN_2       13
#define DUTY_CYCLE_MS    1000

// PID stuff
#define PID_K_P          0.6
#define PID_K_I          0.1
#define PID_K_D          0.8

// Analog samples for averaging
#define A_SAMPLES        4000
#define A_MAX            1023

// NTC parameters and voltage divider resistor
#define NTC_B            3999
#define NTC_R_N          100000
#define NTC_T_N          25
#define NTC_R_V          100000              

// Beeper
#define HAS_BEEPER_1        5
#define HAS_BEEPER_2        4


#endif
