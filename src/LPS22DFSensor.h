/**
 ******************************************************************************
 * @file    LPS22DFSensor.h
 * @author  SRA
 * @version V1.0.0
 * @date    July 2022
 * @brief   Abstract Class of a LPS22DF pressure sensor.
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


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __LPS22DFSensor_H__
#define __LPS22DFSensor_H__


/* Includes ------------------------------------------------------------------*/

#include "Wire.h"
#include "SPI.h"
#include "lps22df_reg.h"


/* Defines -------------------------------------------------------------------*/

#define LPS22DF_I2C_BUS          0U
#define LPS22DF_SPI_4WIRES_BUS   1U
#define LPS22DF_SPI_3WIRES_BUS   2U


/* Typedefs ------------------------------------------------------------------*/

typedef enum {
  LPS22DF_OK = 0,
  LPS22DF_ERROR = -1
} LPS22DFStatusTypeDef;


/* Class Declaration ---------------------------------------------------------*/

/**
 * Abstract class of a LPS22DF pressure sensor.
 */
class LPS22DFSensor {
  public:
    LPS22DFSensor(TwoWire *i2c, uint8_t address = LPS22DF_I2C_ADD_H);
    LPS22DFSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed = 2000000);

    LPS22DFStatusTypeDef begin();
    LPS22DFStatusTypeDef end();
    LPS22DFStatusTypeDef ReadID(uint8_t *Id);
    LPS22DFStatusTypeDef Enable();
    LPS22DFStatusTypeDef Disable();
    LPS22DFStatusTypeDef GetOutputDataRate(float *Odr);
    LPS22DFStatusTypeDef SetOutputDataRate(float Odr);

    LPS22DFStatusTypeDef GetPressure(float *Value);
    LPS22DFStatusTypeDef Get_PRESS_DRDY_Status(uint8_t *Status);

    LPS22DFStatusTypeDef GetTemperature(float *Value);
    LPS22DFStatusTypeDef Get_TEMP_DRDY_Status(uint8_t *Status);

    LPS22DFStatusTypeDef Read_Reg(uint8_t Reg, uint8_t *Data);
    LPS22DFStatusTypeDef Write_Reg(uint8_t Reg, uint8_t Data);

    LPS22DFStatusTypeDef Set_One_Shot();
    LPS22DFStatusTypeDef Get_One_Shot_Status(uint8_t *Status);

    /**
     * @brief Utility function to read data.
     * @param  pBuffer: pointer to data to be read.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  NumByteToRead: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Read(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
    {
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));

        digitalWrite(cs_pin, LOW);

        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr | 0x80);
        /* Read the data */
        for (uint16_t i = 0; i < NumByteToRead; i++) {
          *(pBuffer + i) = dev_spi->transfer(0x00);
        }

        digitalWrite(cs_pin, HIGH);

        dev_spi->endTransaction();

        return 0;
      }

      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
        dev_i2c->write(RegisterAddr);
        dev_i2c->endTransmission(false);

        dev_i2c->requestFrom(((uint8_t)(((address) >> 1) & 0x7F)), (uint8_t) NumByteToRead);

        int i = 0;
        while (dev_i2c->available()) {
          pBuffer[i] = dev_i2c->read();
          i++;
        }

        return 0;
      }

      return 1;
    }

    /**
     * @brief Utility function to write data.
     * @param  pBuffer: pointer to data to be written.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  NumByteToWrite: number of bytes to write.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Write(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
    {
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));

        digitalWrite(cs_pin, LOW);

        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr);
        /* Write the data */
        for (uint16_t i = 0; i < NumByteToWrite; i++) {
          dev_spi->transfer(pBuffer[i]);
        }

        digitalWrite(cs_pin, HIGH);

        dev_spi->endTransaction();

        return 0;
      }

      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));

        dev_i2c->write(RegisterAddr);
        for (uint16_t i = 0 ; i < NumByteToWrite ; i++) {
          dev_i2c->write(pBuffer[i]);
        }

        dev_i2c->endTransmission(true);

        return 0;
      }

      return 1;
    }

  private:
    LPS22DFStatusTypeDef SetOutputDataRate_When_Enabled(float Odr);
    LPS22DFStatusTypeDef SetOutputDataRate_When_Disabled(float Odr);

    /* Helper classes. */
    TwoWire  *dev_i2c;
    SPIClass *dev_spi;

    uint32_t     bus_type; /*0 means I2C, 1 means SPI 4-Wires, 2 means SPI-3-Wires */
    uint8_t      initialized;
    uint8_t      enabled;
    lps22df_md_t last_md;

    /* Configuration */
    uint8_t  address;
    int      cs_pin;
    uint32_t spi_speed;

    lps22df_ctx_t reg_ctx;
};

#ifdef __cplusplus
extern "C" {
#endif
int32_t LPS22DF_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
int32_t LPS22DF_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);
#ifdef __cplusplus
}
#endif

#endif /* __LPS22DFSensor_H__ */
