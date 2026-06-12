#include <Arduino.h>
#include <PID_v1.h>
#include <Servo.h>

#define SERVO_PIN 14
#define SENSOR_PIN 35

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

Servo myservo;

double Setpoint, Distancia, Output;
double Kp = 1, Ki = 0, Kd = 0;
PID processPID(&Distancia, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

double distanciaFiltrada = 0.0;
const double alpha = 0.95;

const uint32_t SAMPLE_TIME_MS = 10;
uint32_t lastUpdate = 0;

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
    return 0;

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

void setup() {
  Serial.begin(115200);
  pinMode(SENSOR_PIN, INPUT);
  myservo.attach(SERVO_PIN);

  Setpoint = 12.5;
  Distancia = distanciaFiltrada;

  processPID.SetOutputLimits(-60, 60);
  processPID.SetSampleTime(SAMPLE_TIME_MS);
  processPID.SetMode(AUTOMATIC);

  myservo.write(90);

  Serial.println(" Sistema iniciado");
  printPIDStatus();
}

void loop() {
  processSerialCommands();

  if (millis() - lastUpdate >= SAMPLE_TIME_MS) {
    lastUpdate = millis();

    double leitura = distanciaInterpolada(analogRead(SENSOR_PIN));

    distanciaFiltrada = (1.0 - alpha) * distanciaFiltrada + alpha * leitura;
    Distancia = distanciaFiltrada;

    processPID.Compute();

    int servoAngle = (int)(90 + Output);
    servoAngle = constrain(servoAngle, 30, 170);
    myservo.write(servoAngle);

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