
int __min  = -0.5;
int __max =  1.5;
void setup() {
  
  Serial.begin(1200);
  while(!Serial);

  // put your setup code here, to run once:
  pinMode(16, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("btn:");
  Serial.println(digitalRead(16));
  Serial.print("min:");
  Serial.println(__min);
  Serial.print("max:");
  Serial.println(__max);
}
