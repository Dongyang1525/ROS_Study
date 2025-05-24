// ESP32study - chapter 1 ADC current and voltage detection


#include <Arduino.h>

void setup()
{
  Serial.begin(115200);
  pinMode(34,INPUT);
  analogSetAttenuation(ADC_11db);
  pinMode(2,OUTPUT);

}

void loop()
{
  digitalWrite(2,LOW);
  delay(300);
  digitalWrite(2,HIGH);
  delay(300);

  int analogValue = analogRead(34);
  int analogVolts = analogReadMilliVolts(34);

  float realVolts = 5.02*((float)analogVolts * 1e-3);

  //Serial.printf("1\n");
  Serial.printf("ADC analog Value: %d\n",analogValue);
  Serial.printf("ADC millivolts: %d\n",analogVolts );
  Serial.printf("Real Volts: %.2f\n",realVolts);
  
  delay(100);
}







