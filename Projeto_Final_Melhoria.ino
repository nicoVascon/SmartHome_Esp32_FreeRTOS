#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"

#include <Wire.h>
#include "skywriter.h"

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
char last_gesture;

/* The task functions. */
void vSkywriter_Task(void *pvParameters);
void vGestureManager_Task(void *pvParameters);
void vLEDPWM(void *pvParameters);
void vTemperature(void *pvParameters);
void vAnalogGas(void *pvParameters);
void vAmbientLight(void *pvParameters);

/* Declare a variable of type SemaphoreHandle_t.  This is used to reference the
semaphore that is used to synchronize a task with an interrupt. */
SemaphoreHandle_t xBinarySemaphore;

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

  /* Before a semaphore is used it must be explicitly created.  In this example
  a binary semaphore is created. */
  vSemaphoreCreateBinary(xBinarySemaphore);

  /*Queues Creation*/
  xGesturesQueue = xQueueCreate(5, sizeof(char));

  /* Check the semaphore was created successfully. */
  if (xBinarySemaphore != NULL) {
    /* Create one of the two tasks. */
    xTaskCreatePinnedToCore(vSkywriter_Task,  /* Pointer to the function that implements the task. */
                            "Skywriter Task", /* Text name for the task.  This is to facilitate debugging only. */
                            2048,             /* Stack depth - most small microcontrollers will use much less stack than this. */
                            NULL,             /* We are not using the task parameter. */
                            1,                /* This task will run at priority 1. */
                            NULL,             /* We are not using the task handle. */
                            1);               /* Core where the task should run */

    /* Create the other task in exactly the same way. */
    xTaskCreatePinnedToCore(vGestureManager_Task, "Gesture Manager", 2048, NULL, 1, NULL, 1);
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

void vSkywriter_Task(void *pvParameters) {
  Skywriter.begin(PIN_TRFD, PIN_RESET);
  Skywriter.onGesture(gesture);
  // Skywriter.onXYZ(handle_xyz);
  /* As per most tasks, this task is implemented in an infinite loop. */
  for (;;) {
    Skywriter.poll();
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}
/*-----------------------------------------------------------*/

void vGestureManager_Task(void *pvParameters) {
  char gesture;

  /* Note that when you create a binary semaphore in FreeRTOS, it is ready
  to be taken, so you may want to take the semaphore after you create it
  so that the task waiting on this semaphore will block until given by
  another task. */
  xSemaphoreTake(xBinarySemaphore, 0);

  /* As per most tasks, this task is implemented within an infinite loop. */
  for (;;) {
    /* Use the semaphore to wait for the event.  The semaphore was created
    before the scheduler was started so before this task ran for the first
    time.  The task blocks indefinitely meaning this function call will only
    return once the semaphore has been successfully obtained - so there is no
    need to check the returned value. */
    xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);

    while (xQueueReceive(xGesturesQueue, &gesture, 0) != errQUEUE_EMPTY) {
      /* To get here the event must have occurred.  Process the event (in this
      case we just print out a message). */
      Serial.printf("Gesture Manager Task - Gesture: %d\r\n", last_gesture);
    }
  }
}

void loop() {
  vTaskDelete(NULL);
}

void gesture(unsigned char type) {
  last_gesture = type;
  xQueueSendToBack(xGesturesQueue, &last_gesture, 0);
  xSemaphoreGive(xBinarySemaphore);
}

void handle_xyz(unsigned int x, unsigned int y, unsigned int z) {
  Serial.printf("X: %d; Y: %d; Z: %d\r\n", x, y, z);
}
