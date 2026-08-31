/*
   @file    LPS22DF_DataLog_Terminal_I3C_ENTDAA.ino
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

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}
  delay(1000);

  Serial.println("=== LPS22DF DAA ===");

  if (!I3C.begin(I3C_SDA, I3C_SCL, 1000000U)) {
    Serial.println("begin() failed");
    while (1) {}
  }

  if (!I3C.resetDynamicAddresses()) {
    Serial.println("resetDynamicAddresses() failed");
    while (1) {}
  }

  I3CDiscoveredDevice devices[8] {};
  size_t found = 0;

  if (I3C.discover(devices, 8, &found)) {
    Serial.println("discover() failed");
    while (1) {}
  }

  uint8_t lpsDynAddr = 0U;

  for (size_t i = 0; i < found; ++i) {
    Serial.println(devices[i].pid, HEX);
    if (devices[i].pid == LPS22DF_I3C_PID_H) {
      lpsDynAddr = devices[i].dynAddr;
      Serial.print("lpsDynAddr=");
      Serial.println(lpsDynAddr, HEX);
      break;
    }
  }

  if (lpsDynAddr == 0U) {
    Serial.println("Sensor not found");
    while (1) {}
  }

  if (sensor.begin(lpsDynAddr) != LPS22DF_OK) {
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

void loop()
{
  float pressure = 0.0f;
  float temperature = 0.0f;

  if (sensor.GetPressure(&pressure) == LPS22DF_OK && sensor.GetTemperature(&temperature) == LPS22DF_OK) {
    Serial.print("Pressure[hPa]:");
    Serial.print(pressure, 2);
    Serial.print(", Temperature[C]:");
    Serial.println(temperature, 2);
  } else {
    Serial.println("Read failed");
  }

  delay(500);
}
