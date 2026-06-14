#include <Arduino.h>
#include <PID_v1.h>
#include <Servo.h>
#include "config/config.h"
#include "serial_console.h"
#include "wifi_manager.h"
#include "mqtt_client.h"
#include "sensor.h"

double Setpoint, Distancia, Output;
double Kp = 2.50, Ki = 0.2, Kd = 3;
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

  processPID.SetOutputLimits(-400, 400);
  processPID.SetSampleTime(SAMPLE_TIME_MS);
  processPID.SetMode(AUTOMATIC);

  double leitura = distanciaInterpolada(analogRead(SENSOR_PIN));
  double Distancia = leitura;
  myservo.writeMicroseconds(1295);

  angAtual = 1295;
  acelAtual = 0;
  velAtual = 0;
  pos = 1295;
  angDestino = 1295;
  ultimoScan = millis();

  Serial.println(" Sistema iniciado");
  printPIDStatus( Kp, Ki, Kd, Setpoint );
}

void loop() {
  processSerialCommands( Kp, Ki, Kd, Setpoint, processPID );
  
  if (!client.connected()) reconnect();
  client.loop();

  if (millis() - ultimoUpdatePID >= SAMPLE_TIME_MS)
  {
    ultimoUpdatePID = millis();

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

  // Controle de posicao: transforma o erro de PWM em uma velocidade alvo.
  float erroPos = angDestino - pos;
  float velDesejada = erroPos * ganhoPos;
  velDesejada = constrain(velDesejada, -velMax, velMax);

  // Controle de velocidade: converte o erro de velocidade em aceleracao alvo.
  float acelDesejada = (velDesejada - velAtual) * ganhoVel;

  // Limitacao de aceleracao: mantem a aceleracao dentro do envelope configurado.
  acelDesejada = constrain(acelDesejada, -acelMax, acelMax);

  // Limitacao de jerk: suaviza a transicao da aceleracao a cada ciclo.
  float deltaAcel = acelDesejada - acelAtual;
  deltaAcel = constrain(deltaAcel, -jerkMax * dt, jerkMax * dt);
  acelAtual += deltaAcel;
  acelAtual = constrain(acelAtual, -acelMax, acelMax);

  velAtual = constrain(velAtual, -velMax, velMax);
  velAtual += acelAtual * dt;
  velAtual = constrain(velAtual, -velMax, velMax);

  pos += velAtual * dt;
  pos = constrain(pos, 1100.0f, 1900.0f);

  float servoPWM = constrain(pos, 1100.0f, 1900.0f);
  angAtual = servoPWM;

  myservo.writeMicroseconds(servoPWM);
  client.publish("servoAngle", String(servoPWM).c_str());

  Serial.print("pos: ");
  Serial.print(pos, 2);

  Serial.print("| Distancia: ");
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
  Serial.println(servoPWM);
}
