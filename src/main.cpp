#include <Arduino.h>
#include <PID_v1.h>
#include <Servo.h>
#include "config/config.h"
#include "serial_console.h"
#include "wifi_manager.h"
#include "mqtt_client.h"
#include "sensor.h"

double Setpoint, Distancia, Output;
double Kp = 1, Ki = 0, Kd = 0;
PID processPID(&Distancia, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

double distanciaFiltrada = 0.0;
uint32_t lastUpdate = 0;

int status = WL_IDLE_STATUS;
unsigned long lastMsg = 0;

char msg[MSG_BUFFER_SIZE];

Servo myservo;

void setup() {
  Serial.begin(115200);
  pinMode(SENSOR_PIN, INPUT);
  myservo.attach(SERVO_PIN);

  client.setServer(MQTT_SERVER, 1900);
  client.setCallback(callback);

  setup_wifi();
  printWifiStatus();

  Setpoint = 12.5;

  processPID.SetOutputLimits(-60, 60);
  processPID.SetSampleTime(SAMPLE_TIME_MS);
  processPID.SetMode(AUTOMATIC);

  myservo.write(90);

  Serial.println(" Sistema iniciado");
  printPIDStatus( Kp, Ki, Kd, Setpoint );
}

void loop() {
  processSerialCommands( Kp, Ki, Kd, Setpoint, processPID );
  
  if (!client.connected()) reconnect();
  client.loop();

  if (millis() - lastUpdate >= SAMPLE_TIME_MS) {
    lastUpdate = millis();

    double leitura = distanciaInterpolada(analogRead(SENSOR_PIN));

    distanciaFiltrada = (1.0 - alpha) * distanciaFiltrada + alpha * leitura;
    Distancia = distanciaFiltrada;
    client.publish("Distancia", String(Distancia).c_str());

    processPID.Compute();

    int servoAngle = (int)(90 + Output);
    servoAngle = constrain(servoAngle, 30, 170);
    myservo.write(servoAngle);
    client.publish("servoAngle", String(servoAngle).c_str());

    double erro = Setpoint - Distancia;

    Serial.print("Distancia: ");
    Serial.print(Distancia, 2);

    Serial.print(" cm | kp : ");
    Serial.print(Kp, 2);

    Serial.print(" | ki : ");
    Serial.print(Ki, 2);

    Serial.print(" | kd : ");
    Serial.print(Kd, 2);

    Serial.print(" | PID: ");
    Serial.print(Output, 2);

    Serial.print(" | Servo: ");
    Serial.println(servoAngle);
  }
}