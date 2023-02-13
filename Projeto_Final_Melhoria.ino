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

//---------------- ADDITIONAL CONSTANTS -----------------//
#define ADC_RESOLUTION 10
#define FREQ 5000

/*IRS*/
void my_poll(void);

/* The task functions. */
void vSkywriter_Task(void *pvParameters);
void vGestureManager_Task(void *pvParameters);
void vLEDPWM(void *pvParameters);

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
  analogReadResolution(ADC_RESOLUTION);

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

void vLEDPWM(void *pvParameters) {
  int ledChannel = LED_PIN;
  ledcSetup(ledChannel, FREQ, ADC_RESOLUTION);
  ledcAttachPin(LED_PIN, ledChannel);
  for (;;) {
    for (int dutyCycle = 0; dutyCycle <= (pow(2, ADC_RESOLUTION)); dutyCycle += 2) {
      ledcWrite(LED_PIN, dutyCycle);
      vTaskDelay(2 / portTICK_PERIOD_MS);
    }
    for (int dutyCycle = (pow(2, ADC_RESOLUTION)); dutyCycle >= 0; dutyCycle -= 2) {
      ledcWrite(LED_PIN, dutyCycle);
      vTaskDelay(2 / portTICK_PERIOD_MS);
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
