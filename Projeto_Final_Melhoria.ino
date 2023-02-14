

//---------------- Actuators PINS -----------------//

#define SERVO_PIN 2 //Servo motor


void vServo(void *pvParameters);


/*----------------Servo-----------------*/
int pos_servo = 0; // initial position of servo


void servopulse(int myangle)  // define a servo pulse function
{
  int pulsewidth = (myangle * 11) + 500;  // convert angle to 500-2480 pulse width
  digitalWrite(SERVO_PIN, HIGH);       // set the level of servo pin as “high”
  delayMicroseconds(pulsewidth);      // delay microsecond of pulse width
  digitalWrite(SERVO_PIN, LOW);        // set the level of servo pin as “low”
  vTaskDelay((20 - pulsewidth / 1000) / portTICK_PERIOD_MS);
}
/*-----------------------------*/

void setup() {
  /*Servo Task*/
  xTaskCreatePinnedToCore(vServo, "Servo motor", 1024, NULL, 2, NULL, 1);
}

void vServo(void *pvParameters) {
  pinMode(SERVO_PIN, OUTPUT);  // set servo pin as “output”
  for (;;) {
    for (int i = 0; i <= 50; i++)  // giving the servo time to rotate to commanded position
    {
      servopulse(180*pos_servo/4);  // use the pulse function
    }
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}

