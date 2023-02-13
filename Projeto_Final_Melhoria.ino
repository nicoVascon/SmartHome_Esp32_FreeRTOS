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
  xTaskCreatePinnedToCore(vLCDTask, "TFT Display", 1024, NULL, 1, NULL, 1);
}



void vLCDTask(void *pvParameters) {
  int pos[] = {0,1,2};
  const char* layout[3] = {"GAS ", "TEMP", "LUM "};
  int values_test[] ={20,24,30};
  //ESP32Time rtc(3600);  // offset in seconds GMT+1
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
  tft.setCursor(10, 2);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.println("--:--:--");
  for (;;) {
    tft.setTextSize(2);
    tft.setCursor(15, 100);
    tft.println(layout[pos[0]]);
    tft.setCursor(255, 100);
    tft.println(layout[pos[2]]);
    tft.setCursor(15, 125);
    tft.println(values_test[pos[0]]);
    tft.setCursor(255, 125);
    tft.println(values_test[pos[2]]);
    tft.setTextSize(5);
    tft.setCursor(95, 40);
    tft.println(layout[pos[1]]);
    tft.setCursor(95, 90);
    tft.println(values_test[pos[1]]);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}


void loop() {
  vTaskDelete(NULL);
}