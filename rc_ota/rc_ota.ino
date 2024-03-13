/*
MIT License

Copyright (c) 2021 G.Products

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

//
// Should be set to Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS).
//

#include <ArduinoOTA.h>
#include <WiFi.h>
#include "BluetoothSerial.h"
#include "driver/ledc.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

// Port
#define PIN_LED_STATUS (GPIO_NUM_13)
#define PIN_LED_VFREAD (GPIO_NUM_14)

#define INA1 (GPIO_NUM_27)  // IC3
#define INA2 (GPIO_NUM_26)  // IC3
#define INB1 (GPIO_NUM_25)  // IC2
#define INB2 (GPIO_NUM_33)  // IC2
#define INC1 (GPIO_NUM_17)  // IC1
#define INC2 (GPIO_NUM_16)  // IC1

#define INA1_CH (LEDC_CHANNEL_0)
#define INA2_CH (LEDC_CHANNEL_1)
#define INB1_CH (LEDC_CHANNEL_2)
#define INB2_CH (LEDC_CHANNEL_3)
#define INC1_CH (LEDC_CHANNEL_4)
#define INC2_CH (LEDC_CHANNEL_5)
#define NUM_CH (6)

// PWM Setting (motor)
#define LEDC_RESOLUTION (4)
#define LEDC_MAX_DUTY (int)(pow(2, LEDC_RESOLUTION) - 1)
#define LEDC_PWMFREQ (400000)  //400 kHz
#define CMD_TIMEOUT (2000/10)     //2000 ms

// RC Parameters
#define RC_HEADER (0xff)
bool flag_updated;
uint8_t dat;
uint8_t speed_L;
uint8_t speed_R;
uint8_t direction_L;
uint8_t direction_R;

// WiFi Setting for OTA
const char *ssid = "OTA_AP";
const char *pass = "password";
const IPAddress ip(192, 168, 10, 1);
const IPAddress subnet(255, 255, 255, 0);

void pwm_set_duty(uint8_t port, uint8_t duty)  // duty : 0~100%
{
  uint32_t duty_cal = duty;
  if (duty > 100) {
    duty_cal = 100;
  }
  duty_cal = duty_cal * LEDC_MAX_DUTY / 100;

  switch (port) {
    case INA1:
      ledcWrite(INA1_CH, duty_cal);
      break;
    case INA2:
      ledcWrite(INA2_CH, duty_cal);
      break;
    case INB1:
      ledcWrite(INB1_CH, duty_cal);
      break;
    case INB2:
      ledcWrite(INB2_CH, duty_cal);
      break;
    case INC1:
      ledcWrite(INC1_CH, duty_cal);
      break;
    case INC2:
      ledcWrite(INC2_CH, duty_cal);
      break;
    default:
      Serial.println("Error");
      break;
  }
}

void motor_func() {
  switch (direction_L) {
    case 1:
      pwm_set_duty(INA1, 100 - speed_L);
      pwm_set_duty(INA2, 100);
      Serial.println("L1");
      break;
    case 2:
      pwm_set_duty(INA1, 100);
      pwm_set_duty(INA2, 100 - speed_L);
      Serial.println("L2");
      break;
    default:
      pwm_set_duty(INA1, 100);
      pwm_set_duty(INA2, 100);
      Serial.println("L0");
      break;
  }

  switch (direction_R) {
    case 1:
      pwm_set_duty(INB1, 100 - speed_R);
      pwm_set_duty(INB2, 100);
      Serial.println("R1");
      break;
    case 2:
      pwm_set_duty(INB1, 100);
      pwm_set_duty(INB2, 100 - speed_R);
      Serial.println("R2");
      break;
    default:
      pwm_set_duty(INB1, 100);
      pwm_set_duty(INB2, 100);
      Serial.println("R0");
      break;
  }
}

void setup() {
  delay(1000);
  Serial.begin(115200);

  // -------------------------------------------------
  // MUST KEEP OTA CODE
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, pass);
  IPAddress address = WiFi.softAPIP();
  Serial.println(address);
  ArduinoOTA.onStart([]() {})
    .onEnd([]() {})
    .onProgress([](unsigned int progress, unsigned int total) {})
    .onError([](ota_error_t error) {});
  ArduinoOTA.begin();
  // -------------------------------------------------

  // GPIO initialize for LED
  pinMode(PIN_LED_VFREAD, INPUT);
  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_STATUS, LOW);

  // GPIO initialize for Motor
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(INC1, OUTPUT);
  pinMode(INC2, OUTPUT);
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, LOW);
  digitalWrite(INB1, LOW);
  digitalWrite(INB2, LOW);
  digitalWrite(INC1, LOW);
  digitalWrite(INC2, LOW);

  // PWM initialize for Motor
  ledcSetup(INA1_CH, LEDC_PWMFREQ, LEDC_RESOLUTION);
  ledcAttachPin(INA1, INA1_CH);
  ledcSetup(INA2_CH, LEDC_PWMFREQ, LEDC_RESOLUTION);
  ledcAttachPin(INA2, INA2_CH);

  ledcSetup(INB1_CH, LEDC_PWMFREQ, LEDC_RESOLUTION);
  ledcAttachPin(INB1, INB1_CH);
  ledcSetup(INB2_CH, LEDC_PWMFREQ, LEDC_RESOLUTION);
  ledcAttachPin(INB2, INB2_CH);

  ledcSetup(INC1_CH, LEDC_PWMFREQ, LEDC_RESOLUTION);
  ledcAttachPin(INC1, INC1_CH);
  ledcSetup(INC2_CH, LEDC_PWMFREQ, LEDC_RESOLUTION);
  ledcAttachPin(INC2, INC2_CH);

  // Bluetooth UART initialize
  SerialBT.begin("bt_uart");  // Bluetooth device name

  Serial.println("Initialized");
  digitalWrite(PIN_LED_STATUS, HIGH);
}

void rx_func(uint8_t rxdata) {
  // Protocol
  // 0xff + speedL + speedR + directionL + directionR
  static uint8_t mode = 0;
  static uint8_t b_speed_L;      // 0~100%
  static uint8_t b_speed_R;      // 0~100%
  static uint8_t b_direction_L;  // 0:idle, 1:forward, 2:reverse
  static uint8_t b_direction_R;  // 0:idle, 1:forward, 2:reverse

  switch (mode) {
    case 1:
      mode = 2;
      b_speed_L = rxdata;
      break;
    case 2:
      mode = 3;
      b_speed_R = rxdata;
      break;
    case 3:
      mode = 4;
      b_direction_L = rxdata;
      break;
    case 4:
      mode = 0;
      b_direction_R = rxdata;

      // Check if received data is updated.
      flag_updated = false;
      if (speed_L != b_speed_L) {
        flag_updated = true;
      }
      if (speed_R != b_speed_R) {
        flag_updated = true;
      }
      if (direction_L != b_direction_L) {
        flag_updated = true;
      }
      if (direction_R != b_direction_R) {
        flag_updated = true;
      }

      // Update parameters
      if (flag_updated == true) {
        speed_L = b_speed_L;
        speed_R = b_speed_R;
        direction_L = b_direction_L;
        direction_R = b_direction_R;
      }
      break;
    default:
      if (rxdata == RC_HEADER) {
        mode = 1;
      } else {
        mode = 0;
      }
  }
}

uint8_t loop_count = 0;

void loop() {
  // -------------------------------------------------
  // MUST KEEP OTA CODE
  ArduinoOTA.handle();
  // -------------------------------------------------

  if (SerialBT.available()) {
    dat = (uint8_t)SerialBT.read();
    rx_func(dat);

    // Get updated data
    if (flag_updated) {
      flag_updated = false;
      // Motor control
      motor_func();
      digitalWrite(PIN_LED_STATUS, HIGH);
      loop_count = 0;
    }
  }
  delay(10);

  // Showing connection status
  if (++loop_count > CMD_TIMEOUT) {
    // force stop due to command timeout
    loop_count = CMD_TIMEOUT;
    flag_updated = false;
    speed_L = 0;
    speed_R = 0;
    motor_func();
    digitalWrite(PIN_LED_STATUS, HIGH);
  } else {
    if (loop_count > 20) {
      // 200ms
      digitalWrite(PIN_LED_STATUS, LOW);
    }
  }
}
