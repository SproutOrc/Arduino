#include <Arduino.h>
int analogPin = 3;     // potentiometer wiper (middle terminal) connected to analog pin 3
                       // outside leads to ground and +5V
int val = 0;           // variable to store the value read


void setup()
{
  analogReference(DEFAULT);
   Serial.begin(115200);          //  setup serial
}
void loop()
{
  val = analogRead(analogPin);    // read the input pin
  val >>= 2;
  Serial.write(val);             // debug value
}
