#include <Adafruit_ILI9341.h>

#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
//---------------- PINS -----------------//
#define LM35_Pin 4     //temperature lm35
#define LED_PIN 13     //led
#define GAS_PIN 15     //analog gas
#define LIGHT_PIN 26    //ambient light sensor

// Skywritter pins
#define PIN_TRFD 27
#define PIN_RESET 17
//-------- ADDITIONAL CONSTANTS ---------//
#define ADC_RESOLUTION 12

//--------------- TASKS -----------------//
void vTemperature(void *pvParameters);
void vBuzzer(void *pvParameters);
void vAnalogGas(void *pvParameters);
void vAmbientLight(void *pvParameters);


void setup() {
  Serial.begin(9600);  //Set Baud Rate to 9600 bps
  while (!Serial) {}
  Serial.println("Hello World!");
  xTaskCreatePinnedToCore(vTemperature, "Temperature Measurement Task", 1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vAnalogGas, "Analog Gas Measurement Task", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vAmbientLight, "Ambient Light Measurement Task", 2048, NULL, 1, NULL, 1);
  analogReadResolution(ADC_RESOLUTION);
}

void vTemperature(void *pvParameters) {
  int analogTemp;
  float analogTemp_voltage;
  for (;;) {
    analogTemp = analogRead(LM35_Pin);
    analogTemp_voltage = (500 * analogTemp) / 4096;
    Serial.print("Temp:");  //Display the temperature on Serial monitor
    Serial.println(analogTemp_voltage);
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

void vAnalogGas(void *pvParameters) {
  int val;
  for (;;) {
    val = analogRead(GAS_PIN);
    Serial.printf("Gas: %d\n", val);
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

void vAmbientLight(void *pvParameters) {
  int val;
  for (;;) {
    val = analogRead(LIGHT_PIN);
    Serial.printf("Lum: %d\n", val);
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

void loop() {
  vTaskDelete(NULL);
}