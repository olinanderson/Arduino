volatile unsigned int count = 0;
unsigned long next = 1000;
unsigned int temp = 0;

void setup()
{
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(3), magnet_detect, RISING);
}

void loop()
{
  if (millis()> next) {
    temp = count;
    Serial.println(temp*12);
    count -= temp;
    next += 1000;
  }
}

void magnet_detect()
{
   count++;
}
