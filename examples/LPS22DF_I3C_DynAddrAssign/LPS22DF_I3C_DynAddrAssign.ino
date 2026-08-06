/*
   @file    LPS22DF_I3C_DynAddrAssign.ino
   @author  STMicroelectronics
   @brief   Example to use the LPS22DF pressure sensor with I3C dynamic address assignment
 *******************************************************************************
   Copyright (c) 2026, STMicroelectronics
   All rights reserved.

   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause

 *******************************************************************************
*/

#include "LPS22DFSensor.h"

LPS22DFSensor sensor(&I3C);

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(1000);

  Serial.println("=== LPS22DF DAA ===");

  if (!I3C.begin(I3C_SDA, I3C_SCL, 1000000U)) {
    Serial.println("begin() failed");
    while (1) {}
  }

  I3CDiscoveredDevice devices[8]{};
  size_t found = 0;

  if (I3C.discover(devices, 8, &found)) {
    Serial.println("discover() failed");
    while (1) {}
  }

  uint8_t lpsDynAddr = 0U;

  for (size_t i = 0; i < found; ++i) {
    Serial.println(devices[i].pid,HEX);
    if (devices[i].pid == LPS22DF_I3C_PID_H) {
      lpsDynAddr = devices[i].dynAddr;
      break;
    }
  }

  if (lpsDynAddr == 0U) {
    Serial.println("Sensor not found");
    while (1) {}
  }

  if (sensor.set_address(lpsDynAddr) != LPS22DF_OK) {
    Serial.println("set_address() failed");
    while (1) {}
  }

  if (sensor.begin() != LPS22DF_OK) {
    Serial.println("sensor.begin() failed");
    while (1) {}
  }

  if (!I3C.setClock(12500000)) {
    Serial.println("setClock() failed");
    while (1) {}
  }

  if (sensor.Enable() != LPS22DF_OK) {
    Serial.println("sensor.Enable() failed");
    while (1) {}
  }

  Serial.println("LPS22DF ready");
}

void loop() {
  float p = 0.0f;
  float t = 0.0f;

  if (sensor.GetPressure(&p) == LPS22DF_OK && sensor.GetTemperature(&t) == LPS22DF_OK) {
    Serial.print("P = ");
    Serial.print(p, 2);
    Serial.print(" hPa   T = ");
    Serial.print(t, 1);
    Serial.println(" C");
  } else {
    Serial.println("Read failed");
  }

  delay(500);
}
