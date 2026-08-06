# LPS22DF

Arduino library to support the LPS22DF 260-1260 hPa absolute digital output barometer

## API

This sensor uses I2C, SPI or I3C to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensors:  

    TwoWire dev_i2c(I2C_SDA, I2C_SCL);  
    dev_i2c.begin();

For SPI it is then required to create a SPI interface before accessing to the sensors:  

    SPIClass dev_spi(SPI_MOSI, SPI_MISO, SPI_SCK);  
    dev_spi.begin();

An instance can be created and enabled when the I2C bus is used following the procedure below:  

    LPS22DFSensor PressTemp(&dev_i2c);
    PressTemp.begin();
    PressTemp.Enable();

An instance can be created and enabled when the SPI bus is used following the procedure below:  

    LPS22DFSensor PressTemp(&dev_spi, CS_PIN);
    PressTemp.begin();
    PressTemp.Enable();

An instance can be created and enabled when the I3C bus is used with SETDASA (static-to-dynamic address assignment):  

    LPS22DFSensor PressTemp(&I3C, LPS22DF_I3C_ADD_H, 0x30);
    I3C.begin(I3C_SDA, I3C_SCL, 1000000U);
    I3C.resetDynamicAddresses();
    I3C.assignDynamicAddress(PressTemp.getStaticAddress(), PressTemp.getDynAddress());
    PressTemp.begin();
    I3C.setClock(12500000);
    PressTemp.Enable();

An instance can be created and enabled when the I3C bus is used with ENTDAA (dynamic address discovery):  

    LPS22DFSensor PressTemp(&I3C);
    I3C.begin(I3C_SDA, I3C_SCL, 1000000U);
    I3C.discover(devices, 8, &found);
    // find dynAddr by matching LPS22DF_I3C_PID_H in discovered devices
    PressTemp.set_address(dynAddr);
    PressTemp.begin();
    I3C.setClock(12500000);
    PressTemp.Enable();

The access to the sensor values is done as explained below:  

  Read pressure and temperature.  

    float pressure;
    float temperature;
    PressTemp.GetPressure(&pressure);  
    PressTemp.GetTemperature(&temperature);

## Documentation

You can find the source files at  
https://github.com/stm32duino/LPS22DF

The LPS22DF datasheet is available at  
https://www.st.com/content/st_com/en/products/mems-and-sensors/pressure-sensors/lps22df.html
