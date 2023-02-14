#include <Adafruit_ILI9341.h>

#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include <ESP32Time.h>



int servopin = 2;  // select digital pin 9 for servomotor signal line
int myangle;       // initialize angle variable
int pulsewidth;    // initialize width variable
int val;
void vServo(void *pvParameters);
void servopulse(int servopin, int myangle)  // define a servo pulse function
{
  pulsewidth = (myangle * 11) + 500;  // convert angle to 500-2480 pulse width
  digitalWrite(servopin, HIGH);       // set the level of servo pin as “high”
  delayMicroseconds(pulsewidth);      // delay microsecond of pulse width
  digitalWrite(servopin, LOW);        // set the level of servo pin as “low”
  vTaskDelay((20 - pulsewidth / 1000) / portTICK_PERIOD_MS);
}
void setup() {
  pinMode(servopin, OUTPUT);  // set servo pin as “output”
  Serial.begin(9600);         // connect to serial port, set baud rate at “9600”
  Serial.println("servo=o_seral_simple ready");
  xTaskCreatePinnedToCore(vServo, "Servo motor", 1024, NULL, 1, NULL, 1);
}

void vServo(void *pvParameters) {
  int position = 0;
  for (;;) {
    if (position == 0) {
      for (int i = 0; i <= 50; i++)  // giving the servo time to rotate to commanded position
      {
        servopulse(servopin, 0);  // use the pulse function
      }
      position = 1;
    } else {
      for (int i = 0; i <= 50; i++)  // giving the servo time to rotate to commanded position
      {
        servopulse(servopin, 180);  // use the pulse function
      }
      position = 0;
    }
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}

void loop()  // convert number 0 to 9 to corresponding 0-180 degree angle, LED blinks corresponding number of time
{
  vTaskDelete(NULL);
}
