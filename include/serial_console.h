#pragma once

void serialSetup();

void printPIDStatus(
  double Kp,
  double Ki,
  double Kd,
  double Setpoint
);

void processSerialCommands(
  double& Kp,
  double& Ki,
  double& Kd,
  double& Setpoint,
  PID& processPID
);