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


double Setpoint, Distancia, Output;
double Kp = 3.0, Ki = 0.0, Kd = 1.0;

double distanciaFiltrada = 0.0;
const double alpha = 0.95;
const uint32_t SAMPLE_TIME_MS = 16;
uint32_t lastUpdate = 0;

float distanciaInterpolada(double adc)
{
  // Saturação superior
  if (adc >= tabela[0].adc)
      return tabela[0].distancia_mm;

  // Saturação inferior
  if (adc <= tabela[NUM_PONTOS - 1].adc)
      return tabela[NUM_PONTOS - 1].distancia_mm;

  for (int i = 0; i < NUM_PONTOS - 1; i++)
  {
    if (adc <= tabela[i].adc &&
        adc >= tabela[i + 1].adc)
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
int status = WL_IDLE_STATUS;
unsigned long lastMsg = 0;

char msg[MSG_BUFFER_SIZE];

SharpIR sensorDistancia(SharpIR::GP2Y0A41SK0F, SENSOR_PIN);
PID processPID( &Distancia, &Output, &Setpoint, Kp, Ki, Kd, DIRECT );
Servo myservo;

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(SECRET_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(SECRET_SSID, SECRET_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // todo: Implement message receved treatement

}

void reconnect() {
  // todo: Refactor this function to not brake pid when not connected
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    if (client.connect("espClient", "esp32", "123456")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(SENSOR_PIN, INPUT);
  myservo.attach(SERVO_PIN);

  client.setServer(mqtt_server, 1900);
  client.setCallback(callback);

  setup_wifi();
  printWifiStatus();

  Setpoint = 15.0;

  Distancia = distanciaFiltrada;

  processPID.SetOutputLimits(-30, 30);
  processPID.SetSampleTime(SAMPLE_TIME_MS);
  processPID.SetMode(AUTOMATIC);

  myservo.write(90);

  Serial.println(" Sistema iniciado");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (millis() - lastUpdate >= SAMPLE_TIME_MS) {
    lastUpdate = millis();

    double leitura = distanciaInterpolada(analogRead(SENSOR_PIN));

    distanciaFiltrada = (1.0 - alpha) * distanciaFiltrada + alpha * leitura;

    Distancia = distanciaFiltrada;
    client.publish("Distancia", String(Distancia).c_str());

    processPID.Compute();

    int servoAngle = (int)(90 + Output);

    servoAngle = constrain( servoAngle, 60, 120 );

    myservo.write(servoAngle);
    client.publish("servoAngle", String(servoAngle).c_str());

    double erro = Setpoint - Distancia;
    
    Serial.print("Leitura: ");
    Serial.print(leitura);
    
    Serial.print(" cm | Filtrada: ");
    Serial.println(Distancia, 2);
    delay(200);

    // Serial.print(" cm | Setpoint: ");
    // Serial.print(Setpoint);

    // Serial.print(" | Erro: ");
    // Serial.print(erro, 2);

    // Serial.print(" | PID: ");
    // Serial.print(Output, 2);

    // Serial.print(" | Servo: ");
    // Serial.println(servoAngle);
  }
}