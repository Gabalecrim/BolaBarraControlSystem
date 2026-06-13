#include <Arduino.h>
#include <PID_v1.h>
#include <Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define SERVO_PIN 14
#define SENSOR_PIN 35

#define SECRET_SSID "gab"
#define SECRET_PASS "12345678"
#define MSG_BUFFER_SIZE	(50)
const char* mqtt_server = "192.168.137.1";

float angAtual;
float acelAtual;
float velAtual;
float velMax = 150;
float acelMax = 900;
float jerkMax = 1667;
const float ganhoPos = 2.0f;
const float ganhoVel = 6.0f;

float pos = 0;
float angDestino = 0;

struct CalibrationPoint {
  int adc;
  float distancia_mm;
};

CalibrationPoint tabela[] = {
    {3400, 3},
    {3000, 4},
    {2500, 5},
    {2200, 6},
    {1900, 7},
    {1650, 8},
    {1450, 9},
    {1290, 10},
    {1100, 11},
    {1000, 12},
    {920, 13},
    {880, 14},
    {790, 15},
    {720, 16},
    {630, 17},
    {590, 18},
    {450, 19},
    {400, 20},
    {360, 21},
    {300, 22},
    {220, 23},
    {200, 24},
    {150, 25}
};

const int NUM_PONTOS = sizeof(tabela) / sizeof(tabela[0]);
#define SECRET_SSID "gab"
#define SECRET_PASS "12345678"
#define MSG_BUFFER_SIZE	(50)
const char* mqtt_server = "192.168.137.1";

#include "config/config.h"
#include "serial_console.h"
#include "wifi_manager.h"
#include "mqtt_client.h"
#include "sensor.h"

double Setpoint, Distancia, Output;
double Kp = 2.50, Ki = 0.2, Kd = 3;
PID processPID(&Distancia, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

double distanciaFiltrada = 0.0;
const double alpha = 0.80;

const uint32_t SAMPLE_TIME_MS = 10;
uint32_t ultimoUpdatePID = 0;
uint32_t ultimoScan = 0;

float distanciaInterpolada(double adc)
{
    if (adc >= tabela[0].adc)
        return tabela[0].distancia_mm;

    if (adc <= tabela[NUM_PONTOS - 1].adc)
        return tabela[NUM_PONTOS - 1].distancia_mm;

    for (int i = 0; i < NUM_PONTOS - 1; i++)
    {
        if (adc <= tabela[i].adc && adc >= tabela[i + 1].adc)
        {
            float x1 = tabela[i].adc;
            float x2 = tabela[i + 1].adc;
            float y1 = tabela[i].distancia_mm;
            float y2 = tabela[i + 1].distancia_mm;

            return y1 + (adc - x1) * (y2 - y1) / (x2 - x1);
        }
    }

    return 0;
}

void printPIDStatus()
{
  Serial.println("\n===== PID =====");

  Serial.print("Kp: ");
  Serial.println(Kp);

  Serial.print("Ki: ");
  Serial.println(Ki);

  Serial.print("Kd: ");
  Serial.println(Kd);

  Serial.print("Setpoint: ");
  Serial.println(Setpoint);

  Serial.println("===============");
}

void processSerialCommands()
{
  if (!Serial.available())
    return;

  String command = Serial.readStringUntil('\n');
  command.trim();

  if (command.startsWith("kp="))
  {
    Kp = command.substring(3).toFloat();
    processPID.SetTunings(Kp, Ki, Kd);

    Serial.print("Novo Kp: ");
    Serial.println(Kp);
  }
  else if (command.startsWith("ki="))
  {
    Ki = command.substring(3).toFloat();
    processPID.SetTunings(Kp, Ki, Kd);

    Serial.print("Novo Ki: ");
    Serial.println(Ki);
  }
  else if (command.startsWith("kd="))
  {
    Kd = command.substring(3).toFloat();
    processPID.SetTunings(Kp, Ki, Kd);

    Serial.print("Novo Kd: ");
    Serial.println(Kd);
  }
  else if (command.startsWith("sp="))
  {
    Setpoint = command.substring(3).toFloat();

    Serial.print("Novo Setpoint: ");
    Serial.println(Setpoint);
  }
  else if (command == "status")
  {
    printPIDStatus();
  }
}
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

  Setpoint = 15.0;

  distanciaFiltrada = sensorDistancia.getDistance();
  Distancia = distanciaFiltrada;

  processPID.SetOutputLimits(-400, 400);
  processPID.SetSampleTime(SAMPLE_TIME_MS);
  processPID.SetMode(AUTOMATIC);

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

void loop()
{
  processSerialCommands();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (millis() - ultimoUpdatePID >= SAMPLE_TIME_MS)
  {
    ultimoUpdatePID = millis();

    double leitura = distanciaInterpolada(analogRead(SENSOR_PIN));

    distanciaFiltrada = (1.0 - alpha) * distanciaFiltrada + alpha * leitura;

    Distancia = distanciaFiltrada;
    client.publish("Distancia", String(Distancia).c_str());

    processPID.Compute();

    angDestino = 1295 + Output;
  }

  float dt = (millis() - ultimoScan) / 1000.0f;
  ultimoScan = millis();
    client.publish("servoAngle", String(servoAngle).c_str());

  if (dt <= 0.0f)
  {
    dt = SAMPLE_TIME_MS / 1000.0f;
  }
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

  // if ((pos <= 1100.0f && velAtual < 0.0f) || (pos >= 1900.0f && velAtual > 0.0f))
  // {
  //   velAtual = 0.0f;
  //   if ((pos <= 1100.0f && acelAtual < 0.0f) || (pos >= 1900.0f && acelAtual > 0.0f))
  //   {
  //     acelAtual = 0.0f;
  //   }
  // }

  float servoPWM = constrain(pos, 1100.0f, 1900.0f);
  angAtual = servoPWM;

  myservo.writeMicroseconds(servoPWM);

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
