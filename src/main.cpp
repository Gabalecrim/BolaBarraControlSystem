#include <Arduino.h>
#include <SharpIR.h>
#include <PID_v1.h>
#include <Servo.h>

#define SERVO_PIN 9
#define SENSOR_PIN 4
int sensorModel = 20150;
double Setpoint, Distancia, Output;

SharpIR sensorDistancia(sensorModel, SENSOR_PIN);
Servo myservo;

double Kp=1, Ki=0, Kd=0;
PID processPID(&Distancia, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

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
  Distancia = sensorDistancia.getDistance();

  processPID.Compute();

  int servoAngle = (int)(90 + Output);
  servoAngle = constrain(servoAngle, 60, 120);

  myservo.write(servoAngle);

  Serial.print("Posicao: ");
  Serial.print(Distancia);

  Serial.print(" cm | Setpoint: ");
  Serial.print(Setpoint);

  Serial.print(" | PID: ");
  Serial.print(Output);

  Serial.print(" | Servo: ");
  Serial.println(servoAngle);
}
