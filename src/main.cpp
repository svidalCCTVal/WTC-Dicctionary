#include <Arduino.h>
#include "Servo.h"

char F_cmd[4]= "FFF";
char R_cmd[4]= "RRR";
char V_cmd[4]= "VVV";
char T_cmd[4]= "TTT";

void setup() {
  Serial.begin(9600);
}

void loop() {
  // reply only when you receive data:
  if (Serial.available() > 0) {
    String IncomingString = Serial.readStringUntil('\n');
    char input_cmd[4];
    IncomingString.toCharArray(input_cmd,sizeof(input_cmd));
    char input_data[64] = {0}; 
    IncomingString.substring(3).toCharArray(input_data, sizeof(input_data));

    //FFF
    if (!strcmp(input_cmd,F_cmd))
    {
      Serial.println("FFF yes");
      Serial.println(input_data);
    } //RRR
    else if (!strcmp(input_cmd,R_cmd))
    {
      Serial.println("FFF yes");
    } /* //VVV
    else if (!strcmp(input_cmd,F_cmd))
    {
      Serial.println("FFF yes");
    } //TTT
    else if (!strcmp(input_cmd,F_cmd))
    {
      Serial.println("FFF yes");
    } //III
    else if (!strcmp(input_cmd,F_cmd))
    {
      Serial.println("FFF yes");
    }//PPP
    else if (!strcmp(input_cmd,F_cmd))
    {
      Serial.println("FFF yes");
    }//AAA
    else if (!strcmp(input_cmd,F_cmd))
    {
      Serial.println("FFF yes");
    }
    */
  }
}
