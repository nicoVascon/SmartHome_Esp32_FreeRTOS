void vGestureManager_Task(void *pvParameters) {
  char gesture;

  /* As per most tasks, this task is implemented within an infinite loop. */
  for (;;) {
    if (xQueueReceive(xGesturesQueue, &gesture, portMAX_DELAY) != errQUEUE_EMPTY) {
      /* To get here the event must have occurred.  Process the event (in this
      case we just print out a message). */
      switch (gesture) {
        case 2:
          if (pos_lcd == 0) {
            pos_lcd = 2;
          } else {
            pos_lcd--;
          }
          break;
        case 3:
          if (pos_lcd == 2) {
            pos_lcd = 0;
          } else {
            pos_lcd++;
          }
          break;
        case 4:
          if (pos_servo == 4) {
            break;
          }
          pos_servo++;
          break;
        case 5:
          if (pos_servo == 0) {
            break;
          }
          pos_servo--;
          break;
        default:
          Serial.println("Gesture - Garbage");
      }
    }
  }
}