#include <Adafruit_ILI9341.h>

#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include <ESP32Time.h>
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

int pos_lcd = 0;

void setup() {
  Serial.begin(9600);//Set Baud Rate to 9600 bps
  xTaskCreatePinnedToCore(vLCDTask, "TFT Display", 1024, NULL, 1, NULL, 1);
}



void vLCDTask(void *pvParameters) {
  ESP32Time rtc(3600);                  // offset in seconds GMT+1
  rtc.setTime(10, 50, 8, 17, 1, 2021);  // 17th Jan 2021 15:24:30
  const int position[3][3] = { { 0, 1, 2 }, { 1, 2, 0 }, { 2, 0, 1 } };
  const char *layout[3] = { "GAS ", "TEMP", "LUM " };
  int values_test[] = { 20, 24, 30 };
  char stringTime[8];
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1000;
  xLastWakeTime = xTaskGetTickCount();
  Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(1);
  tft.setTextSize(2);
  tft.fillRect(5, 90, 70, 60, ILI9341_WHITE);
  tft.fillRect(7, 92, 66, 56, ILI9341_BLACK);
  tft.fillRect(245, 90, 70, 60, ILI9341_WHITE);
  tft.fillRect(247, 92, 66, 56, ILI9341_BLACK);
  tft.fillRect(85, 30, 150, 190, ILI9341_WHITE);
  tft.fillRect(87, 32, 146, 186, ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  Serial.println("Oi Ini");
  for (;;) {
    Serial.println("Oi for......");
    tft.setTextSize(2);
    tft.setCursor(10, 2);
    sprintf(stringTime, "%02d:%02d:%02d", rtc.getHour(true), rtc.getMinute(), rtc.getSecond());
    tft.println(stringTime);
    tft.setCursor(15, 100);
    tft.println(layout[position[pos_lcd][0]]);
    tft.setCursor(255, 100);
    tft.println(layout[position[pos_lcd][2]]);
    tft.setCursor(15, 125);
    tft.println(values_test[position[pos_lcd][0]]);
    tft.setCursor(255, 125);
    tft.println(values_test[position[pos_lcd][2]]);
    tft.setTextSize(5);
    tft.setCursor(95, 40);
    tft.println(layout[position[pos_lcd][1]]);
    tft.setCursor(95, 90);
    tft.println(values_test[position[pos_lcd][1]]);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


void loop() {
  vTaskDelete(NULL);
}