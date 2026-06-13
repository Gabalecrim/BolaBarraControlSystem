#include <Arduino.h>
#include <PID_v1.h>
#include "serial_service.h"

void serialSetup() {
  return;
}

void printPIDStatus( double Kp, double Ki, double Kd, double Setpoint ) 
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

void processSerialCommands(
  double& Kp,
  double& Ki,
  double& Kd,
  double& Setpoint,
  PID& processPID
) 
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
  printPIDStatus( Kp, Ki, Kd, Setpoint );
  }
}