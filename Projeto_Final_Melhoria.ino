#include <Adafruit_ILI9341.h>

#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "ESP32Time"

#include <Wire.h>
#include "skywriter.h"

/*-------------Tone Library Structures and Variables-------------*/
typedef enum {
  TONE_START,
  TONE_END,
  TONE_SET_CHANNEL
} tone_cmd_t;

typedef struct {
  tone_cmd_t tone_cmd;
  uint8_t pin;
  unsigned int frequency;
  unsigned long duration;
  uint8_t channel;
} tone_msg_t;
static TaskHandle_t _my_tone_task = NULL;
static QueueHandle_t _tone_queue = NULL;
static uint8_t _channel = 0;
/*--------------------------------------------------*/

/*-------------Buzzer and Melody values-------------*/

// Define pin 10 for buzzer, you can use any other digital pins (Pin 0-13)
const int buzzer = 32;

// Change to 0.5 for a slower version of the song, 1.25 for a faster version
const float songSpeed = 1.0;

// Defining frequency of each music note
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

// Music notes of the song, 0 is a rest/pulse
const int notes[] = {
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
  NOTE_B4, NOTE_C5, 0, NOTE_B4, 0, NOTE_A4
};

// Durations (in ms) of each music note of the song
// Quarter Note is 250 ms when songSpeed = 1.0
const int durations[] = {
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
  125, 125, 125, 125, 125, 500
};
/*--------------------------------------------------*/

#define PIN_TRFD 27
#define PIN_RESET 17

//---------------- Actuators PINS -----------------//
#define LED_PIN 13  //led
//---------------- Sensors PINS -----------------//
#define LM35_Pin 4     //temperature lm35
#define GAS_PIN 15     //analog gas
#define LIGHT_PIN 26    //ambient light sensor

//---------------- ADDITIONAL CONSTANTS -----------------//
#define ADC_RESOLUTION 12
#define FREQ 5000

//---------------- PINS -----------------//
// Set Adafruit tft pins
#define TFT_DC 33
#define TFT_CS 3
#define TFT_MOSI 23
#define TFT_CLK 18
#define TFT_RST 1
#define TFT_MISO 19



/*IRS*/
void my_poll(void);

/* The task functions. */
void vSkywriter_Task(void *pvParameters);
void vGestureManager_Task(void *pvParameters);
void vLEDPWM(void *pvParameters);
void vTemperature(void *pvParameters);
void vAnalogGas(void *pvParameters);
void vAmbientLight(void *pvParameters);
void vLCDTask(void *pvParameters);

/* Declare a variable of type SemaphoreHandle_t.  This is used to reference the
semaphore that is used to synchronize a task with an interrupt. */
SemaphoreHandle_t xBinarySemaphore;
SemaphoreHandle_t xSkywriter_Semaphore;

/*Queues*/
QueueHandle_t xGesturesQueue;

void setup() {
  Serial.begin(9600);

  while (!Serial) {};

  Serial.println("Hello world!");

  /*Sensors Tasks*/
  xTaskCreatePinnedToCore(vLEDPWM, "PWM for LED Task", 1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vTemperature, "Temperature Measurement Task", 1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vAnalogGas, "Analog Gas Measurement Task", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vAmbientLight, "Ambient Light Measurement Task", 2048, NULL, 1, NULL, 1);
  analogReadResolution(ADC_RESOLUTION);

  /*LCD Task*/
  xTaskCreatePinnedToCore(vLCDTask, "TFT Display", 1024, NULL, 1, NULL, 1);

  /* Before a semaphore is used it must be explicitly created.  In this example
  a binary semaphore is created. */
  vSemaphoreCreateBinary(xSkywriter_Semaphore);

  /*Queues Creation*/
  xGesturesQueue = xQueueCreate(5, sizeof(char));

  /* Check the semaphore was created successfully. */
  if (xSkywriter_Semaphore != NULL) {
    /* Create one of the two tasks. */
    xTaskCreatePinnedToCore(vSkywriter_Task,  /* Pointer to the function that implements the task. */
                            "Skywriter Task", /* Text name for the task.  This is to facilitate debugging only. */
                            2048,             /* Stack depth - most small microcontrollers will use much less stack than this. */
                            NULL,             /* We are not using the task parameter. */
                            1,                /* This task will run at priority 1. */
                            NULL,             /* We are not using the task handle. */
                            1);               /* Core where the task should run */

    /* Set up the initial interrupt */
    Skywriter.begin(PIN_TRFD, PIN_RESET);
    Skywriter.onGesture(gesture);

    // pinMode(PIN_TRFD, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(PIN_TRFD), &my_poll, FALLING);
    // attachInterrupt(digitalPinToInterrupt(PIN_TRFD), &my_poll, FALLING);

    /* Create the other task in exactly the same way. */
    xTaskCreatePinnedToCore(vGestureManager_Task, "Gesture Manager", 2048, NULL, 1, NULL, 1);
  }
  /* Create Buzzer Task. */
  xTaskCreatePinnedToCore(vBuzzer_Task, "Buzzer Task", 2048, NULL, 1, NULL, 1);
}

void vLCDTask(void *pvParameters) {
  ESP32Time rtc(3600);  // offset in seconds GMT+1
  rtc.setTime(10, 50, 8, 17, 1, 2021);  // 17th Jan 2021 15:24:30
  int pos[] = {0,1,2};
  const char* layout[3] = {"GAS ", "TEMP", "LUM "};
  int values_test[] ={20,24,30};
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
  for (;;) {
    tft.setTextSize(2);
    tft.setCursor(10, 2);
    sprintf(stringTime, "%02d:%02d:%02d", rtc.getHour(true), rtc.getMinute(),rtc.getSecond());
    tft.println(stringTime);
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
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void vBuzzer_Task(void *pvParameters) {
  const int totalNotes = sizeof(notes) / sizeof(int);
  for (;;) {
    // Loop through each note
    for (int i = 0; i < totalNotes; i++) {
      int currentNote = notes[i];
      float wait = durations[i] / songSpeed;
      // Play tone if currentNote is not 0 frequency, otherwise pause (my_noTone)
      if (currentNote != 0) {
        my_tone(buzzer, notes[i], wait);  // tone(pin, frequency, duration)
      } else {
        my_noTone(buzzer);
      }
      // delay is used to wait for tone to finish playing before moving to next loop
      vTaskDelay(wait / portTICK_PERIOD_MS);
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

void my_poll(void) {
  static signed portBASE_TYPE xHigherPriorityTaskWoken;

  xHigherPriorityTaskWoken = pdFALSE;

  /* 'Give' the semaphore to unblock the task. */
  xSemaphoreGiveFromISR(xSkywriter_Semaphore, (signed portBASE_TYPE *)&xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken == pdTRUE) {
    /* Giving the semaphore unblocked a task, and the priority of the
    unblocked task is higher than the currently running task - force
    a context switch to ensure that the interrupt returns directly to
    the unblocked (higher priority) task.

    NOTE: The syntax for forcing a context switch is different depending
    on the port being used.  Refer to the examples for the port you are
    using for the correct method to use! */
    portYIELD_FROM_ISR();
  }
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

void vLEDPWM(void *pvParameters) {
  int ledChannel = LED_PIN;
  ledcSetup(ledChannel,FREQ,ADC_RESOLUTION);
  ledcAttachPin(LED_PIN,ledChannel);
  for (;;) {
    for (int dutyCycle = 0; dutyCycle <= (pow(2,ADC_RESOLUTION)); dutyCycle += 40){
      ledcWrite(LED_PIN, dutyCycle);
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    for (int dutyCycle = (pow(2,ADC_RESOLUTION)); dutyCycle >= 0; dutyCycle -= 40){
      ledcWrite(LED_PIN, dutyCycle);
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }
}

void vSkywriter_Task(void *pvParameters) {
  Skywriter.begin(PIN_TRFD, PIN_RESET);
  Skywriter.onGesture(gesture);

  xSemaphoreTake(xSkywriter_Semaphore, 0);
  /* As per most tasks, this task is implemented in an infinite loop. */
  for (;;) {
    xSemaphoreTake(xSkywriter_Semaphore, portMAX_DELAY);

    Skywriter.poll();
  }
}
/*-----------------------------------------------------------*/

void vGestureManager_Task(void *pvParameters) {
  char gesture;

  /* As per most tasks, this task is implemented within an infinite loop. */
  for (;;) {
    if (xQueueReceive(xGesturesQueue, &gesture, portMAX_DELAY) != errQUEUE_EMPTY) {
      /* To get here the event must have occurred.  Process the event (in this
      case we just print out a message). */
      Serial.printf("Gesture Manager Task - Gesture: %d\r\n", gesture);
    }
  }
}

void loop() {
  vTaskDelete(NULL);
}

void gesture(unsigned char type) {
  char last_gesture = type;
  xQueueSendToBack(xGesturesQueue, &last_gesture, 0);
}

void handle_xyz(unsigned int x, unsigned int y, unsigned int z) {
  Serial.printf("X: %d; Y: %d; Z: %d\r\n", x, y, z);
}

/* Tone Library Implementation */

static void my_tone_task(void *) {
  tone_msg_t tone_msg;
  while (1) {
    xQueueReceive(_tone_queue, &tone_msg, portMAX_DELAY);
    switch (tone_msg.tone_cmd) {
      case TONE_START:
        log_d("Task received from queue TONE_START: _pin=%d, frequency=%u Hz, duration=%lu ms", tone_msg.pin, tone_msg.frequency, tone_msg.duration);

        log_d("Setup LED controll on channel %d", _channel);
        ledcAttachPin(tone_msg.pin, _channel);
        ledcWriteTone(_channel, tone_msg.frequency);

        if (tone_msg.duration) {
          vTaskDelay(tone_msg.duration / portTICK_PERIOD_MS);
          ledcDetachPin(tone_msg.pin);
          ledcWriteTone(_channel, 0);
        }
        break;

      case TONE_END:
        log_d("Task received from queue TONE_END: pin=%d", tone_msg.pin);
        ledcDetachPin(tone_msg.pin);
        ledcWriteTone(_channel, 0);
        break;

      case TONE_SET_CHANNEL:
        log_d("Task received from queue TONE_SET_CHANNEL: channel=%d", tone_msg.channel);
        _channel = tone_msg.channel;
        break;

      default:;  // do nothing
    }            // switch
  }              // infinite loop
}

static int my_tone_init() {
  if (_tone_queue == NULL) {
    log_v("Creating tone queue");
    _tone_queue = xQueueCreate(128, sizeof(tone_msg_t));
    if (_tone_queue == NULL) {
      log_e("Could not create tone queue");
      return 0;  // ERR
    }
    log_v("Tone queue created");
  }

  if (_my_tone_task == NULL) {
    log_v("Creating tone task");
    xTaskCreate(
      my_tone_task,   // Function to implement the task
      "toneTask",     // Name of the task
      3500,           // Stack size in words
      NULL,           // Task input parameter
      1,              // Priority of the task
      &_my_tone_task  // Task handle.
    );
    if (_my_tone_task == NULL) {
      log_e("Could not create tone task");
      return 0;  // ERR
    }
    log_v("Tone task created");
  }
  Serial.println("Oii");
  return 1;  // OK
}

void my_setToneChannel(uint8_t channel) {
  log_d("channel=%d", channel);
  if (my_tone_init()) {
    tone_msg_t tone_msg = {
      .tone_cmd = TONE_SET_CHANNEL,
      .pin = 0,        // Ignored
      .frequency = 0,  // Ignored
      .duration = 0,   // Ignored
      .channel = channel
    };
    xQueueSend(_tone_queue, &tone_msg, portMAX_DELAY);
  }
}

void my_noTone(uint8_t _pin) {
  log_d("my_noTone was called");
  if (my_tone_init()) {
    tone_msg_t tone_msg = {
      .tone_cmd = TONE_END,
      .pin = _pin,
      .frequency = 0,  // Ignored
      .duration = 0,   // Ignored
      .channel = 0     // Ignored
    };
    xQueueSend(_tone_queue, &tone_msg, portMAX_DELAY);
  }
}

// parameters:
// _pin - pin number which will output the signal
// frequency - PWM frequency in Hz
// duration - time in ms - how long will the signal be outputted.
//   If not provided, or 0 you must manually call my_noTone to end output
void my_tone(uint8_t _pin, unsigned int frequency, unsigned long duration) {
  log_d("_pin=%d, frequency=%u Hz, duration=%lu ms", _pin, frequency, duration);
  if (my_tone_init()) {
    tone_msg_t tone_msg = {
      .tone_cmd = TONE_START,
      .pin = _pin,
      .frequency = frequency,
      .duration = duration,
      .channel = 0  // Ignored
    };
    xQueueSend(_tone_queue, &tone_msg, portMAX_DELAY);
  }
}
