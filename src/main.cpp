#include <Arduino.h>
#include <SharpIR.h>
#include <PID_v1.h>
#include <Servo.h>

#define SERVO_PIN 14
#define SENSOR_PIN 35

SharpIR sensorDistancia(SharpIR::GP2Y0A41SK0F, SENSOR_PIN);

Servo myservo;

double Setpoint, Distancia, Output;

double Kp = 3.0, Ki = 0.0, Kd = 1.0;

PID processPID( &Distancia, &Output, &Setpoint, Kp, Ki, Kd, DIRECT );

double distanciaFiltrada = 0.0;
const double alpha = 0.2;

const uint32_t SAMPLE_TIME_MS = 20;
uint32_t lastUpdate = 0;

void setup() {
  Serial.begin(115200);
  myservo.attach(SERVO_PIN);

  Setpoint = 15.0;

  distanciaFiltrada = sensorDistancia.getDistance();
  Distancia = distanciaFiltrada;

  processPID.SetOutputLimits(-30, 30);
  processPID.SetSampleTime(SAMPLE_TIME_MS);
  processPID.SetMode(AUTOMATIC);

  myservo.write(90);

  Serial.println(" Sistema iniciado");
}

void loop() {

  if (millis() - lastUpdate >= SAMPLE_TIME_MS) {
    lastUpdate = millis();

    double leitura = sensorDistancia.getDistance();

    distanciaFiltrada = (1.0 - alpha) * distanciaFiltrada + alpha * leitura;

    Distancia = distanciaFiltrada;

    processPID.Compute();

    int servoAngle = (int)(90 + Output);

    servoAngle = constrain( servoAngle, 60, 120 );

    myservo.write(servoAngle);

    double erro = Setpoint - Distancia;

    Serial.print("Leitura: ");
    Serial.print(leitura);

    Serial.print(" cm | Filtrada: ");
    Serial.print(Distancia, 2);

    Serial.print(" cm | Setpoint: ");
    Serial.print(Setpoint);

    Serial.print(" | Erro: ");
    Serial.print(erro, 2);

    Serial.print(" | PID: ");
    Serial.print(Output, 2);

    Serial.print(" | Servo: ");
    Serial.println(servoAngle);
  }
}