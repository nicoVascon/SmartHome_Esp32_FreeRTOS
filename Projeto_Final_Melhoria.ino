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

/* The task functions. */
void vSkywriter_Task(void *pvParameters);
void vTask2(void *pvParameters);

/* Declare a variable of type SemaphoreHandle_t.  This is used to reference the
semaphore that is used to synchronize a task with an interrupt. */
SemaphoreHandle_t xBinarySemaphore;

void setup() {
  Serial.begin(9600);

  while (!Serial) {};

  Serial.println("Hello world!");

  /* Before a semaphore is used it must be explicitly created.  In this example
  a binary semaphore is created. */
  vSemaphoreCreateBinary(xBinarySemaphore);

  /* Check the semaphore was created successfully. */
  if (xBinarySemaphore != NULL) {
    /* Create one of the two tasks. */
    xTaskCreatePinnedToCore(vSkywriter_Task,  /* Pointer to the function that implements the task. */
                            "Skywriter Task", /* Text name for the task.  This is to facilitate debugging only. */
                            1024,             /* Stack depth - most small microcontrollers will use much less stack than this. */
                            NULL,             /* We are not using the task parameter. */
                            1,                /* This task will run at priority 1. */
                            NULL,             /* We are not using the task handle. */
                            1);               /* Core where the task should run */

    /* Create the other task in exactly the same way. */
    xTaskCreatePinnedToCore(vTask2, "Task 2", 1024, NULL, 1, NULL, 1);
  }
}

void vSkywriter_Task(void *pvParameters) {
  Skywriter.begin(PIN_TRFD, PIN_RESET);
  Skywriter.onXYZ(handle_xyz);
  /* As per most tasks, this task is implemented in an infinite loop. */
  for (;;) {
    Skywriter.poll();
  }
}
/*-----------------------------------------------------------*/

void vTask2(void *pvParameters) {
  /* Note that when you create a binary semaphore in FreeRTOS, it is ready
  to be taken, so you may want to take the semaphore after you create it
  so that the task waiting on this semaphore will block until given by
  another task. */
  xSemaphoreTake( xBinarySemaphore, 0);

  /* As per most tasks, this task is implemented within an infinite loop. */
  for( ;; )
  {
    /* Use the semaphore to wait for the event.  The semaphore was created
    before the scheduler was started so before this task ran for the first
    time.  The task blocks indefinitely meaning this function call will only
    return once the semaphore has been successfully obtained - so there is no
    need to check the returned value. */
    xSemaphoreTake( xBinarySemaphore, portMAX_DELAY );

    /* To get here the event must have occurred.  Process the event (in this
    case we just print out a message). */
    Serial.print( "EX12: Handler task - Processing event.\r\n" );
  }
}

void loop() {
  vTaskDelete(NULL);
}

void handle_xyz(unsigned int x, unsigned int y, unsigned int z) {
  Serial.printf("X: %d; Y: %d; Z: %d\r\n", x, y, z);
}
