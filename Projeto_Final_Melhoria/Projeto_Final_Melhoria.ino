#include <Adafruit_ILI9341.h>

#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
//---------------- PINS -----------------//
#define BUZZER_Pin 32 //sound buzzer
#define LM35_Pin 4    //temperature lm35
#define LED_PIN 13    //led
#define GAS_PIN 15    //analog gas 
#define LIGHT_PIN 5   //ambient light sensor
// Set Adafruit tft pins
#define TFT_DC 33
#define TFT_CS 3
#define TFT_MOSI 23
#define TFT_CLK 18
#define TFT_RST 1
#define TFT_MISO 19

// Skywritter pins
#define PIN_TRFD 27
#define PIN_RESET 17
//-------- ADDITIONAL CONSTANTS ---------//
#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_D5 587
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_G5 784
#define NOTE_A5 880
#define NOTE_B5 988
#define ADC_RESOLUTION 12

//--------------- TASKS -----------------//
void vTemperature(void *pvParameters);
void vBuzzer(void *pvParameters);
void vAnalogGas(void *pvParameters);
void vAmbientLight(void *pvParameters);
void vLCDTask(void *pvParameters);
voi vLEDPWM(void *pvParameters);

void setup() {
  Serial.begin(9600);//Set Baud Rate to 9600 bps
  xTaskCreatePinnedToCore(vTemperature, "Temperature Measurement Task", 1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vAnalogGas, "Analog Gas Measurement Task", 1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vAmbientLight, "Ambient Light Measurement Task", 1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vLCDTask, "TFT Display", 1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vLEDPWM, "PWM for LED Task", 1024, NULL, 1, NULL, 1);
  //xTaskCreatePinnedToCore(vBuzzer, "Buzzer music", 1024, NULL,1,NULL,1);
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
  int val;
  for (;;) {
    val = analogRead(LIGHT_PIN);
    Serial.println(val);
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
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


void vBuzzer(void *pvParameters){
  /* Play "He's A Pirate" / "Pirates of the Caribbean" Theme Song
 * By Xitang Zhao 2016.06.27
 * Youtube in Action: https://youtu.be/sjPAj1lXgtk
 * or TikTok in Action: https://www.tiktok.com/@tipstorylearn/video/6943019804261502213
  */
// Music notes of the song, 0 is a rest/pulse
int notes[] = {
    NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
    NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
    NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
    NOTE_A4, NOTE_G4, NOTE_A4, 0,

    NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
    NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
    NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
    NOTE_A4, NOTE_G4, NOTE_A4, 0,

    NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
    NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0,
    NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0,
    NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0,

    NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
    NOTE_D5, NOTE_E5, NOTE_A4, 0,
    NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, 0,
    NOTE_C5, NOTE_A4, NOTE_B4, 0,

    NOTE_A4, NOTE_A4,
    //Repeat of first part
    NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
    NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
    NOTE_A4, NOTE_G4, NOTE_A4, 0,

    NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
    NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
    NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
    NOTE_A4, NOTE_G4, NOTE_A4, 0,

    NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
    NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0,
    NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0,
    NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0,

    NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
    NOTE_D5, NOTE_E5, NOTE_A4, 0,
    NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, 0,
    NOTE_C5, NOTE_A4, NOTE_B4, 0,
    //End of Repeat

    NOTE_E5, 0, 0, NOTE_F5, 0, 0,
    NOTE_E5, NOTE_E5, 0, NOTE_G5, 0, NOTE_E5, NOTE_D5, 0, 0,
    NOTE_D5, 0, 0, NOTE_C5, 0, 0,
    NOTE_B4, NOTE_C5, 0, NOTE_B4, 0, NOTE_A4,

    NOTE_E5, 0, 0, NOTE_F5, 0, 0,
    NOTE_E5, NOTE_E5, 0, NOTE_G5, 0, NOTE_E5, NOTE_D5, 0, 0,
    NOTE_D5, 0, 0, NOTE_C5, 0, 0,
    NOTE_B4, NOTE_C5, 0, NOTE_B4, 0, NOTE_A4};
// Durations (in ms) of each music note of the song
// Quarter Note is 250 ms when songSpeed = 1.0
int durations[] = {
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 375, 125,

    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 375, 125,

    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 125, 250, 125,

    125, 125, 250, 125, 125,
    250, 125, 250, 125,
    125, 125, 250, 125, 125,
    125, 125, 375, 375,

    250, 125,
    //Rpeat of First Part
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 375, 125,

    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 375, 125,

    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 125, 250, 125,

    125, 125, 250, 125, 125,
    250, 125, 250, 125,
    125, 125, 250, 125, 125,
    125, 125, 375, 375,
    //End of Repeat

    250, 125, 375, 250, 125, 375,
    125, 125, 125, 125, 125, 125, 125, 125, 375,
    250, 125, 375, 250, 125, 375,
    125, 125, 125, 125, 125, 500,

    250, 125, 375, 250, 125, 375,
    125, 125, 125, 125, 125, 125, 125, 125, 375,
    250, 125, 375, 250, 125, 375,
    125, 125, 125, 125, 125, 500};
for (;;) {
vTaskDelay(250 / portTICK_PERIOD_MS);
}

}

































void loop() {
  vTaskDelete(NULL);
}