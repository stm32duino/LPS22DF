/**
 ******************************************************************************
 * @file    LPS22DFSensor.cpp
 * @author  SRA
 * @version V1.0.0
 * @date    July 2022
 * @brief   Implementation of a LPS22DF pressure sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2022 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/

#include "LPS22DFSensor.h"


/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
LPS22DFSensor::LPS22DFSensor(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
{
  reg_ctx.write_reg = LPS22DF_io_write;
  reg_ctx.read_reg = LPS22DF_io_read;
  reg_ctx.handle = (void *)this;
  dev_spi = NULL;
  enabled = 0L;
}

/** Constructor
 * @param spi object of an helper class which handles the SPI peripheral
 * @param cs_pin the chip select pin
 * @param spi_speed the SPI speed
 */
LPS22DFSensor::LPS22DFSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed) : dev_spi(spi), cs_pin(cs_pin), spi_speed(spi_speed)
{
  reg_ctx.write_reg = LPS22DF_io_write;
  reg_ctx.read_reg = LPS22DF_io_read;
  reg_ctx.handle = (void *)this;
  dev_i2c = NULL;
  address = 0L;
  enabled = 0L;
}

/**
 * @brief  Configure the sensor in order to be used
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DFSensor::begin()
{
  lps22df_md_t md;
  lps22df_bus_mode_t bus_mode;

  if (dev_spi) {
    // Configure CS pin
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
  }

  /* Set bdu and if_inc recommended for driver usage */
  if (lps22df_init_set(&reg_ctx, LPS22DF_DRV_RDY) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  /* Select bus interface */
  if (bus_type == LPS22DF_SPI_3WIRES_BUS) { /* SPI 3-Wires */
    bus_mode.interface = lps22df_bus_mode_t::LPS22DF_SPI_3W;
  } else if (bus_type == LPS22DF_SPI_4WIRES_BUS) { /* SPI 3-Wires */
    bus_mode.interface = lps22df_bus_mode_t::LPS22DF_SPI_4W;
  } else {
    bus_mode.interface = lps22df_bus_mode_t::LPS22DF_SEL_BY_HW;
  }

  bus_mode.filter = lps22df_bus_mode_t::LPS22DF_AUTO;
  if (lps22df_bus_mode_set(&reg_ctx, &bus_mode) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  /* Set Output Data Rate in Power Down */
  md.odr = lps22df_md_t::LPS22DF_ONE_SHOT;
  md.avg = lps22df_md_t::LPS22DF_4_AVG;
  md.lpf = lps22df_md_t::LPS22DF_LPF_ODR_DIV_4;

  /* Power down the device */
  if (lps22df_mode_set(&reg_ctx, &md) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  if (lps22df_mode_get(&reg_ctx, &last_md) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  last_md.odr = lps22df_md_t::LPS22DF_25Hz;
  enabled = 0L;

  initialized = 1L;

  return LPS22DF_OK;
}

/**
 * @brief  Disable the sensor and relative resources
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DFSensor::end()
{
  if (initialized == 1U && Disable() != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  /* Reset CS configuration */
  if (dev_spi) {
    // Configure CS pin
    pinMode(cs_pin, INPUT);
  }

  initialized = 0L;

  return LPS22DF_OK;
}

/**
 * @brief  Get WHO_AM_I value
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DFSensor::ReadID(uint8_t *Id)
{
  if (lps22df_id_get(&reg_ctx, (lps22df_id_t *)Id) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  return LPS22DF_OK;
}

/**
 * @brief  Enable the LPS22DF pressure sensor
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DFSensor::Enable()
{
  /* Check if the component is already enabled */
  if (enabled == 1U) {
    return LPS22DF_OK;
  }

  /* Output data rate selection. */
  if (lps22df_mode_set(&reg_ctx, &last_md) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  enabled = 1U;

  return LPS22DF_OK;
}

/**
 * @brief  Disable the LPS22DF pressure sensor
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DFSensor::Disable()
{
  /* Check if the component is already disabled */
  if (enabled == 0U) {
    return LPS22DF_OK;
  }

  lps22df_md_t val;

  /* Get current output data rate. */
  if (lps22df_mode_get(&reg_ctx, &val) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  memcpy(&last_md, &val, sizeof(lps22df_md_t));

  val.odr = lps22df_md_t::LPS22DF_ONE_SHOT;

  /* Output data rate selection - power down. */
  if (lps22df_mode_set(&reg_ctx, &val) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  enabled = 0L;

  return LPS22DF_OK;
}

/**
 * @brief  Get output data rate
 * @param  Odr the output data rate value
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DFSensor::GetOutputDataRate(float *Odr)
{
  LPS22DFStatusTypeDef ret = LPS22DF_OK;
  lps22df_md_t val;

  if (lps22df_mode_get(&reg_ctx, &val) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  switch (val.odr) {
    case lps22df_md_t::LPS22DF_ONE_SHOT:
      *Odr = 0.0f;
      break;

    case lps22df_md_t::LPS22DF_1Hz:
      *Odr = 1.0f;
      break;

    case lps22df_md_t::LPS22DF_4Hz:
      *Odr = 4.0f;
      break;

    case lps22df_md_t::LPS22DF_10Hz:
      *Odr = 10.0f;
      break;

    case lps22df_md_t::LPS22DF_25Hz:
      *Odr = 25.0f;
      break;

    case lps22df_md_t::LPS22DF_50Hz:
      *Odr = 50.0f;
      break;

    case lps22df_md_t::LPS22DF_75Hz:
      *Odr = 75.0f;
      break;

    case lps22df_md_t::LPS22DF_100Hz:
      *Odr = 100.0f;
      break;

    case lps22df_md_t::LPS22DF_200Hz:
      *Odr = 200.0f;
      break;

    default:
      ret = LPS22DF_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LPS22DF pressure sensor output data rate
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DFSensor::SetOutputDataRate(float Odr)
{
  /* Check if the component is enabled */
  if (enabled == 1U) {
    return SetOutputDataRate_When_Enabled(Odr);
  } else {
    return SetOutputDataRate_When_Disabled(Odr);
  }
}

/**
 * @brief  Set output data rate when enabled
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DFSensor::SetOutputDataRate_When_Enabled(float Odr)
{
  lps22df_md_t new_val;

  if (lps22df_mode_get(&reg_ctx, &new_val) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  new_val.odr = (Odr <=   1.0f) ? lps22df_md_t::LPS22DF_1Hz
                : (Odr <=   4.0f) ? lps22df_md_t::LPS22DF_4Hz
                : (Odr <=  10.0f) ? lps22df_md_t::LPS22DF_10Hz
                : (Odr <=  25.0f) ? lps22df_md_t::LPS22DF_25Hz
                : (Odr <=  50.0f) ? lps22df_md_t::LPS22DF_50Hz
                : (Odr <=  75.0f) ? lps22df_md_t::LPS22DF_75Hz
                : (Odr <= 100.0f) ? lps22df_md_t::LPS22DF_100Hz
                :                   lps22df_md_t::LPS22DF_200Hz;

  if (lps22df_mode_set(&reg_ctx, &new_val) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  if (lps22df_mode_get(&reg_ctx, &last_md) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  return LPS22DF_OK;
}

/**
 * @brief  Set output data rate when disabled
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DFSensor::SetOutputDataRate_When_Disabled(float Odr)
{
  last_md.odr = (Odr <=   1.0f) ? lps22df_md_t::LPS22DF_1Hz
                : (Odr <=   4.0f) ? lps22df_md_t::LPS22DF_4Hz
                : (Odr <=  10.0f) ? lps22df_md_t::LPS22DF_10Hz
                : (Odr <=  25.0f) ? lps22df_md_t::LPS22DF_25Hz
                : (Odr <=  50.0f) ? lps22df_md_t::LPS22DF_50Hz
                : (Odr <=  75.0f) ? lps22df_md_t::LPS22DF_75Hz
                : (Odr <= 100.0f) ? lps22df_md_t::LPS22DF_100Hz
                :                   lps22df_md_t::LPS22DF_200Hz;

  return LPS22DF_OK;
}

/**
 * @brief  Get the LPS22DF pressure value
 * @param  Value pointer where the pressure value is written
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DFSensor::GetPressure(float *Value)
{
  lps22df_data_t data;

  if (lps22df_data_get(&reg_ctx, &data) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  *Value = data.pressure.hpa;

  return LPS22DF_OK;
}

/**
 * @brief  Get the LPS22DF pressure data ready bit value
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DFSensor::Get_PRESS_DRDY_Status(uint8_t *Status)
{
  lps22df_status_t reg;

  if (lps22df_read_reg(&reg_ctx, LPS22DF_STATUS, (uint8_t *) &reg, 1) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  *Status = reg.p_da;

  return LPS22DF_OK;
}

/**
 * @brief  Get the LPS22DF temperature value
 * @param  Value pointer where the temperature value is written
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DFSensor::GetTemperature(float *Value)
{
  lps22df_data_t data;

  if (lps22df_data_get(&reg_ctx, &data) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  *Value = data.heat.deg_c;

  return LPS22DF_OK;
}

/**
 * @brief  Get the LPS22DF temperature data ready bit value
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DFSensor::Get_TEMP_DRDY_Status(uint8_t *Status)
{
  lps22df_status_t reg;

  if (lps22df_read_reg(&reg_ctx, LPS22DF_STATUS, (uint8_t *) &reg, 1) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  *Status = reg.t_da;

  return LPS22DF_OK;
}

/**
 * @brief  Get the LPS22DF register value
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DFSensor::Read_Reg(uint8_t Reg, uint8_t *Data)
{
  if (lps22df_read_reg(&reg_ctx, Reg, Data, 1) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  return LPS22DF_OK;
}

/**
 * @brief  Set the LPS22DF register value
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DFSensor::Write_Reg(uint8_t Reg, uint8_t Data)
{
  if (lps22df_write_reg(&reg_ctx, Reg, &Data, 1) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  return LPS22DF_OK;
}

/**
 * @brief  Set the LPS22DF One Shot Mode
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DFSensor::Set_One_Shot()
{
  lps22df_md_t md;

  if (lps22df_mode_get(&reg_ctx, &md) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  /* Start One Shot Measurement */
  if (lps22df_trigger_sw(&reg_ctx, &md) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  return LPS22DF_OK;
}

/**
 * @brief  Get the LPS22DF One Shot Status
 * @param  Status pointer to the one shot status (1 means measurements available, 0 means measurements not available yet)
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DFSensor::Get_One_Shot_Status(uint8_t *Status)
{
  uint8_t p_da;
  uint8_t t_da;

  /* Get DataReady for pressure */
  if (Get_PRESS_DRDY_Status(&p_da) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  /* Get DataReady for temperature */
  if (Get_TEMP_DRDY_Status(&t_da) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  if (p_da && t_da) {
    *Status = 1L;
  } else {
    *Status = 0L;
  }

  return LPS22DF_OK;
}

int32_t LPS22DF_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  return ((LPS22DFSensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}

int32_t LPS22DF_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  return ((LPS22DFSensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}
