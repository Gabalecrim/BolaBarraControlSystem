#include <Arduino.h>
#include <SharpIR.h>
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


double Setpoint, Distancia, Output;
double Kp = 3.0, Ki = 0.0, Kd = 1.0;

double distanciaFiltrada = 0.0;
const double alpha = 0.2;
const uint32_t SAMPLE_TIME_MS = 16;
uint32_t lastUpdate = 0;
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
  myservo.attach(SERVO_PIN);

  client.setServer(mqtt_server, 1900);
  client.setCallback(callback);

  setup_wifi();
  printWifiStatus();

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
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (millis() - lastUpdate >= SAMPLE_TIME_MS) {
    lastUpdate = millis();

    double leitura = sensorDistancia.getDistance();

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