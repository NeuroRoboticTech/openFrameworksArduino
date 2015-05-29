#include "CommanderHS.h"

CommanderData commanderData;
CommanderHS command = CommanderHS(&Serial1);
  
void checkCommander() {
  
  if(command.ReadMsgs() > 0) {
      //Serial.print("Commander has messages: ");
      //Serial.println(command.ReadMsgs());
    
    //if(command.buttons&BUT_RT)
    //{
      //debugSerial.listen();
     // debugSerial.println("RT");
      //debugSerial.listen();
    //}
    
    if(command.buttons != commanderData.buttons) {
      commanderData.buttons = command.buttons;
      commanderData.dataChanged = true;
    }

    if(command.walkV != commanderData.walkV) {
      commanderData.walkV = command.walkV;
      commanderData.dataChanged = true;
    }

    if(command.walkH != commanderData.walkH) {
      commanderData.walkH = command.walkH;
      commanderData.dataChanged = true;
    }

    if(command.walkV != commanderData.walkV) {
      commanderData.walkV = command.walkV;
      commanderData.dataChanged = true;
    }

    if(command.lookH != commanderData.lookH) {
      commanderData.lookH = command.lookH;
      commanderData.dataChanged = true;
    }

    if(command.lookV != commanderData.lookV) {
      commanderData.lookV = command.lookV;
      commanderData.dataChanged = true;
    }

    //If the commander data has changed then fill out the
    //custom sysex byte array and send it.    
    if(commanderData.dataChanged == true) {
      Serial.print("Commander ");
      Serial.print(", WalkV: "); Serial.print(commanderData.walkV); 
      Serial.print(", WalkH: "); Serial.print(commanderData.walkH); 
      Serial.print(", LookV: "); Serial.print(commanderData.lookV); 
      Serial.print(", LookH: "); Serial.print(commanderData.lookH); 
      Serial.print(", Buttons: "); Serial.print(commanderData.buttons); 
      Serial.println ("");
      
      commanderData.dataChanged = false;
    } 
  }  
}
  
  
void setup() {
  Serial.begin(57600);
  while(!Serial);
  Serial.println("Starting setup");
  
  command.begin(38400);
}

void loop() {
  checkCommander();
  delay(10);
}



