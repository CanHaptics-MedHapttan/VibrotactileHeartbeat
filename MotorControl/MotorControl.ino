#include <stdio.h>
#include <math.h>

const int switchPin = 2;
const int motorPin = 9;
int switchState = 0;

int previousFrame = 0;
int currentFrame = 0;
float heartbeatValue = 0;
float tick = 0;
int threshhold = 2;

void setup() {
  Serial.begin(9600);
  pinMode(motorPin, OUTPUT);
  pinMode(switchPin, INPUT);
}

void AdvanceECG(){
  tick += 0.1; 
  heartbeatValue = (sin(((tick)*4)) + sin(tick*16)/4) * 3 * (-(floor(sin(tick * 2)) + 0.1)) * (1 - floor(sin(fmod(tick/1.5, 2))));
}

void loop() {
  switchState = digitalRead(switchPin);

  currentFrame++;
  if(currentFrame - previousFrame > 2){
    previousFrame = currentFrame;      
    AdvanceECG(); 
  }

  Serial.println(heartbeatValue);
  if((heartbeatValue > threshhold || heartbeatValue < -threshhold)){
    digitalWrite(motorPin, HIGH);
  }
  else {
    digitalWrite(motorPin, LOW);
  }
}
