#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "mqtt_client.h"
#include "wifi_manager.h"

namespace {
  constexpr char MQTT_TOPIC_CONFIG[] = "pid/config";
  constexpr char MQTT_TOPIC_STATUS[] = "pid/status";
  constexpr size_t MQTT_JSON_DOC_SIZE = 512;
  constexpr size_t MQTT_JSON_BUFFER_SIZE = 256;

  void printCurrentConfig() {
    Serial.println("Configuracao final:");
    Serial.print("  kp: ");
    Serial.println(Kp, 4);
    Serial.print("  ki: ");
    Serial.println(Ki, 4);
    Serial.print("  kd: ");
    Serial.println(Kd, 4);
    Serial.print("  setpoint: ");
    Serial.println(Setpoint, 4);
    Serial.print("  alpha: ");
    Serial.println(ALPHA, 4);
    Serial.print("  sampleTime: ");
    Serial.println(SAMPLE_TIME_MS, 4);
    Serial.print("  velMax: ");
    Serial.println(velMax, 4);
    Serial.print("  acelMax: ");
    Serial.println(acelMax, 4);
    Serial.print("  jerkMax: ");
    Serial.println(jerkMax, 4);
    Serial.print("  ganhoPos: ");
    Serial.println(ganhoPos, 4);
    Serial.print("  ganhoVel: ");
    Serial.println(ganhoVel, 4);
  }

  template <typename T>
  bool updateIfPresent(JsonDocument& doc, const char* key, T& target) {
    if (!doc.containsKey(key)) {
      return false;
    }

    T newValue = doc[key].as<T>();
    target = newValue;

    Serial.print("Parametro atualizado: ");
    Serial.print(key);
    Serial.print(" = ");
    Serial.println(target, 4);

    return true;
  }

  void handleConfigMessage(const char* payload) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
      Serial.print("Erro ao desserializar JSON de configuracao: ");
      Serial.println(error.c_str());
      return;
    }

    bool pidUpdated = false;
    bool sampleTimeUpdated = false;
    bool anyUpdated = false;

    pidUpdated |= updateIfPresent(doc, "kp", Kp);
    pidUpdated |= updateIfPresent(doc, "ki", Ki);
    pidUpdated |= updateIfPresent(doc, "kd", Kd);
    anyUpdated |= pidUpdated;

    anyUpdated |= updateIfPresent(doc, "setpoint", Setpoint);
    anyUpdated |= updateIfPresent(doc, "alpha", ALPHA);
    sampleTimeUpdated = updateIfPresent(doc, "sampleTime", SAMPLE_TIME_MS);
    anyUpdated |= sampleTimeUpdated;
    anyUpdated |= updateIfPresent(doc, "velMax", velMax);
    anyUpdated |= updateIfPresent(doc, "acelMax", acelMax);
    anyUpdated |= updateIfPresent(doc, "jerkMax", jerkMax);
    anyUpdated |= updateIfPresent(doc, "ganhoPos", ganhoPos);
    anyUpdated |= updateIfPresent(doc, "ganhoVel", ganhoVel);

    if (!anyUpdated) {
      Serial.println("JSON recebido sem campos reconhecidos. Nenhuma alteracao aplicada.");
      return;
    }

    if (pidUpdated) {
      processPID.SetTunings(Kp, Ki, Kd);
      Serial.println("Tunings do PID atualizados.");
    }

    if (sampleTimeUpdated) {
      processPID.SetSampleTime((int)SAMPLE_TIME_MS);
      Serial.println("Sample time do PID atualizado.");
    }

    printCurrentConfig();
    publishConfig();
  }
}

PubSubClient client(espClient);

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensagem MQTT recebida no topico [");
  Serial.print(topic);
  Serial.println("]");

  if (strcmp(topic, MQTT_TOPIC_CONFIG) != 0) {
    Serial.println("Topico sem tratamento configurado. Mensagem ignorada.");
    return;
  }

  if (length >= MQTT_JSON_DOC_SIZE) {
    Serial.println("Payload de configuracao maior que o limite suportado. Mensagem ignorada.");
    return;
  }

  char payloadBuffer[MQTT_JSON_DOC_SIZE];
  memcpy(payloadBuffer, payload, length);
  payloadBuffer[length] = '\0';

  Serial.print("Payload recebido: ");
  Serial.println(payloadBuffer);

  handleConfigMessage(payloadBuffer);
}

void publishConfig() {
  JsonDocument doc;
  doc["kp"] = Kp;
  doc["ki"] = Ki;
  doc["kd"] = Kd;
  doc["setpoint"] = Setpoint;
  doc["alpha"] = ALPHA;
  doc["sampleTime"] = SAMPLE_TIME_MS;
  doc["velMax"] = velMax;
  doc["acelMax"] = acelMax;
  doc["jerkMax"] = jerkMax;
  doc["ganhoPos"] = ganhoPos;
  doc["ganhoVel"] = ganhoVel;

  char buffer[MQTT_JSON_BUFFER_SIZE];
  size_t requiredSize = measureJson(doc) + 1;

  if (requiredSize > sizeof(buffer)) {
    Serial.println("Buffer insuficiente para publicar configuracao MQTT.");
    return;
  }

  size_t payloadSize = serializeJson(doc, buffer, sizeof(buffer));

  if (payloadSize == 0) {
    Serial.println("Falha ao serializar configuracao MQTT.");
    return;
  }

  if (!client.publish(MQTT_TOPIC_STATUS, buffer, payloadSize)) {
    Serial.println("Falha ao publicar configuracao atual em pid/status.");
    return;
  }

  Serial.print("Configuracao publicada em ");
  Serial.print(MQTT_TOPIC_STATUS);
  Serial.print(": ");
  Serial.println(buffer);
}

void reconnect() {
  // TODO: Refactor this function to not brake pid when not connected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    if (client.connect("espClient", "esp32", "123456")) {
      Serial.println("connected");

      if (client.subscribe(MQTT_TOPIC_CONFIG)) {
        Serial.print("Inscrito no topico ");
        Serial.println(MQTT_TOPIC_CONFIG);
      } else {
        Serial.print("Falha ao inscrever no topico ");
        Serial.println(MQTT_TOPIC_CONFIG);
      }

      publishConfig();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
