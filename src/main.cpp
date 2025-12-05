#include <Arduino.h>
#include <SoftwareSerial.h>
#include <HerkulexServo.h>
#include <HERKULEX.h>
#include "SERVOS.h"


void setup() {
 // Ancien ID
  Serial.begin(115200);
  init_serial_1_for_herkulex(); // Fonction init de "HERKULEX.h"
  delay(500);
  //HerkulexServo old_servo(herkulexBus, 0x04); // Déclaration de l'ancien servo
  //HerkulexServo new_servo(herkulexBus, 0x0A); // Déclaration du nouveau servo
  //change_id(NEW_SERVO, old_servo, new_servo); // Change l'ID du servo de OLD_SERVO à NEW_SERVO
  //detect_id(1);

}

void loop() 
{
  serrer();
  serrer2();
  delay(2000);
  tourner();
  tourner2();
  delay(2000);
  desserrer();
  desserrer2();
  delay(2000);
}