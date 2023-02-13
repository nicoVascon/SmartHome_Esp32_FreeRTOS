#include <Adafruit_ILI9341.h>

#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
//---------------- PINS -----------------//

#define LM35_Pin 4    //temperature lm35
#define GAS_PIN 15    //analog gas 
#define LIGHT_PIN 26   //ambient light sensor
//-------- ADDITIONAL CONSTANTS ---------//
#define ADC_RESOLUTION 10

//--------------- TASKS -----------------//
void vTemperature(void *pvParameters);
void vAnalogGas(void *pvParameters);
void vAmbientLight(void *pvParameters);

void setup() {
  Serial.begin(9600);//Set Baud Rate to 9600 bps
  xTaskCreatePinnedToCore(vTemperature, "Temperature Measurement Task", 1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vAnalogGas, "Analog Gas Measurement Task", 1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vAmbientLight, "Ambient Light Measurement Task", 1024, NULL, 1, NULL, 1);
  analogReadResolution(ADC_RESOLUTION);
}

void vTemperature(void *pvParameters) {
  int analogTemp;
  float analogTemp_voltage;
  for (;;) {
    analogTemp = analogRead(LM35_Pin);
    analogTemp_voltage=(500*analogTemp) /4096;
    Serial.print("Temp:"); //Display the temperature on Serial monitor
    Serial.println(analogTemp_voltage);
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

void vAnalogGas(void *pvParameters) {
  int val;
  for (;;) {
    val = analogRead(GAS_PIN);
    Serial.println(val);
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

void vAmbientLight(void *pvParameters) {
  int analogLight;
  for (;;) {
    analogLight = analogRead(LIGHT_PIN);
    Serial.println(analogLight);
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}


void loop() {
  vTaskDelete(NULL);
}