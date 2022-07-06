/*
   @file    LPS22DF_Example.ino
   @author  Giuseppe Roberti <giuseppe.roberti@ieee.org>
   @brief   Example to use the LPS22DF 260-1260 hPa absolute digital
            output barometer
 *******************************************************************************
   Copyright (c) 2022, STMicroelectronics
   All rights reserved.
   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause
 *******************************************************************************
*/

#include <LPS22DFSensor.h>

LPS22DFSensor sensor (&Wire);
float pressure, temperature;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  sensor.begin();
  sensor.Enable();
}

void loop() {
  sensor.GetPressure(&pressure);
  sensor.GetTemperature(&temperature);

  Serial.print("Pressure[hPa]:");
  Serial.print(pressure, 2);
  Serial.print(", Temperature[C]:");
  Serial.println(temperature, 2);

  blink(LED_BUILTIN);
}

inline void blink(int pin) {
  digitalWrite(pin, HIGH);
  delay(25);
  digitalWrite(pin, LOW);
  delay(975);
}
