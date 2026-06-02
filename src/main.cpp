#include <Arduino.h>
#include <SharpIR.h>
#include <PID_v1.h>
#include <Servo.h>

#define SERVO_PIN 9
#define SENSOR_PIN 4
int sensorModel = 20150;
double Setpoint, Input, Output;

SharpIR sensorDistancia(sensorModel, SENSOR_PIN);
Servo myservo;

double Kp=2, Ki=5, Kd=1;
PID processPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);
  myservo.attach(SERVO_PIN);

  Setpoint = 100;

  processPID.SetOutputLimits(-30, 30);
  processPID.SetSampleTime(20);

  processPID.SetMode(AUTOMATIC);

  myservo.write(90);
  Serial.println("Sistema iniciado");
}

void loop() {
  int distancia = sensorDistancia.getDistance();

  Serial.print("Distância: ");
  Serial.println(distancia);

  processPID.Compute();

  int servoAngle = (int)(90 + Output);
  servoAngle = constrain(servoAngle, 60, 120);

  myservo.write(servoAngle);

  Serial.print("Posicao: ");
  Serial.print(Input);

  Serial.print(" cm | Setpoint: ");
  Serial.print(Setpoint);

  Serial.print(" | PID: ");
  Serial.print(Output);

  Serial.print(" | Servo: ");
  Serial.println(servoAngle);
}
