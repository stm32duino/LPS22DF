/*
   @file    LPS22DF_I3C_Basic.ino
   @author  STMicroelectronics
   @brief   Example to use the LPS22DF pressure sensor with I3C and SETDASA command
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

LPS22DFSensor sensor(&I3C, LPS22DF_I3C_ADD_H, 0x30);

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(1000);

  Serial.println("=== LPS22DF SETDASA ===");

  if (!I3C.begin(I3C_SDA, I3C_SCL, 1000000U)) {
    Serial.println("begin() failed");
    while (1) {}
  }

  if (!I3C.resetDynamicAddresses()) {
    Serial.println("resetDynamicAddresses() failed");
    while (1) {}
  }

  if (!I3C.assignDynamicAddress(sensor.getStaticAddress(), sensor.getDynAddress())) {
    Serial.println("assignDynamicAddress() failed");
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
