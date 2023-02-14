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
  delay(20 - pulsewidth / 1000);
}
void setup() {
  pinMode(servopin, OUTPUT);  // set servo pin as “output”
  Serial.begin(9600);         // connect to serial port, set baud rate at “9600”
  Serial.println("servo=o_seral_simple ready");
  xTaskCreatePinnedToCore(vServo, "Servo motor", 1024, NULL, 1, NULL, 1);
}

void vServo(void *pvParameters) {
  for (;;) {
    val = Serial.read();  // read serial port value
    if (val >= '0' && val <= '9') {
      val = val - '0';        // convert characteristic quantity to numerical variable
      val = val * (180 / 9);  // convert number to angle
      Serial.print("moving servo to ");
      Serial.print(val, DEC);
      Serial.println();
      for (int i = 0; i <= 50; i++)  // giving the servo time to rotate to commanded position
      {
        servopulse(servopin, val);  // use the pulse function
      }
    }
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}

void loop()  // convert number 0 to 9 to corresponding 0-180 degree angle, LED blinks corresponding number of time
{
  vTaskDelete(NULL);
}
