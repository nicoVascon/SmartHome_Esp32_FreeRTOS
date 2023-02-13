#include <Adafruit_ILI9341.h>

#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
//---------------- PINS -----------------//

#define LED_PIN 13    //led

//-------- ADDITIONAL CONSTANTS ---------//
#define ADC_RESOLUTION 10
#define FREQ 5000
//--------------- TASKS -----------------//
void vLEDPWM(void *pvParameters);

void setup() {
  xTaskCreatePinnedToCore(vLEDPWM, "PWM for LED Task", 1024, NULL, 1, NULL, 1);
  analogReadResolution(ADC_RESOLUTION);
}

void vLEDPWM(void *pvParameters) {
  int ledChannel = LED_PIN;
  ledcSetup(ledChannel,FREQ,ADC_RESOLUTION);
  ledcAttachPin(LED_PIN,ledChannel);
  for (;;) {
    for (int dutyCycle = 0; dutyCycle <= (pow(2,ADC_RESOLUTION)); dutyCycle++){
      ledcWrite(LED_PIN, dutyCycle);
      vTaskDelay(1 / portTICK_PERIOD_MS);
  }
    for (int dutyCycle = (pow(2,ADC_RESOLUTION)); dutyCycle >= 0; dutyCycle--){
      ledcWrite(LED_PIN, dutyCycle);
      vTaskDelay(1 / portTICK_PERIOD_MS);
}
}
}



void loop() {
  vTaskDelete(NULL);
}