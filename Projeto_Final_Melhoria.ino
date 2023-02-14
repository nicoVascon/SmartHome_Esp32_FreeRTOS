void vGestureManager_Task(void *pvParameters) {
  char gesture;
  int pos_servo = 0;
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
          if (pos_servo != 4) {
            pos_servo++;
            xQueueSendToBack(xServoQueue, &pos_servo, 0);
          }
          break;
        case 5:
          if (pos_servo != 0) {
            pos_servo--;
            xQueueSendToBack(xServoQueue, &pos_servo, 0);
          }
          break;
        default:
          Serial.println("Gesture - Garbage");
      }
    }
  }
}