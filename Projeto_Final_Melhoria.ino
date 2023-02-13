#include <Adafruit_ILI9341.h>

#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
//---------------- PINS -----------------//
// Set Adafruit tft pins
#define TFT_DC 33
#define TFT_CS 3
#define TFT_MOSI 23
#define TFT_CLK 18
#define TFT_RST 1
#define TFT_MISO 19

//--------------- TASKS -----------------//
void vLCDTask(void *pvParameters);

void setup() {
  Serial.begin(9600);//Set Baud Rate to 9600 bps
  xTaskCreatePinnedToCore(vLCDTask, "TFT Display", 2048, NULL, 1, NULL, 1);
  Serial.println("hello wolrd");
  analogReadResolution(ADC_RESOLUTION);
}

void vLCDTask(void *pvParameters) {
  //ESP32Time rtc(3600);  // offset in seconds GMT+1
  Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(1);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setCursor(68, 60);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(7);
  tft.println("--:--");
  tft.fillCircle(11, 11, 10, ILI9341_RED);
  tft.setCursor(28, 6);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.println("BLE Not connected");
  tft.setTextColor(ILI9341_WHITE, ILI9341_WHITE);
  tft.setCursor(25, 135);
  tft.setTextSize(2);
  tft.println("Temp");
  tft.setTextColor(ILI9341_WHITE, ILI9341_WHITE);
  tft.setCursor(180, 135);
  tft.setTextSize(2);
  tft.println("Hum");
  for (;;) {

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void loop() {
  vTaskDelete(NULL);
}