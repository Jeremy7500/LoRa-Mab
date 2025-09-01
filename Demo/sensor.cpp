#include "sensor.h"
#include "mbed.h"
#include "Eeprom.h"

//Read buffer
const int bufferSize = 4;
uint8_t ReadBuffer[bufferSize];

Sensor::Sensor(PinName mosi, PinName miso, PinName sclk, PinName nss): SensorNss( nss )
{
    SensorSpi = new SPI( mosi, miso, sclk );
    SensorSpi->frequency(100000);
    SensorNss = 1;
}

Sensor::~Sensor()
{
    delete SensorSpi;
}

/* DEPRECATED FOR HTSENS ! */
void Sensor::WakeUp(void)
{
    SensorNss = 0;
    
    // Enable Patch
    Write_Register(WRITE_REGISTER_OPCODE, TEST_ANA_SEL, 0x80, 0x00);
    
    // Confirming the wake-up
    int i = 0;
    for(i=0;i<10;i++)
    {
        Write_Register(WRITE_REGISTER_OPCODE, RF_PWR_CTRL, 0x20, 0x89);  
    } 
    
    // Disabling the patch
    Write_Register(WRITE_REGISTER_OPCODE, TEST_ANA_SEL, 0x00,0x00); 
    
    // Configuring the sensor
    Write_Register(WRITE_REGISTER_OPCODE, SENS_CTRL_0, 0x0B,0x13); 
    Write_Register(WRITE_REGISTER_OPCODE, SENS_CTRL_1, 0x3A,0x49); 
    
    SensorNss = 1;
    
}

/* DEPRECATED FOR HTSENS ! */
void Sensor::Write_Register(uint8_t opcode, uint8_t address, uint8_t value1, uint8_t value2)
{
    SensorNss = 0;
    SensorSpi->write( opcode );
    SensorSpi->write( address );
    SensorSpi->write( value1 );
    SensorSpi->write( value2 );
    SensorNss = 1;
}

/* OK FOR HTSENS */
uint16_t Sensor::Read_Register(uint16_t address)
{
    uint16_t regVal;
    
    SensorSpi->lock();
    SensorNss = 0;
    SensorSpi->write(0x00); // Read opcode
    SensorSpi->write((uint8_t)(address & 0xff)); //lsb
    SensorSpi->write((uint8_t)(address >> 8));  //msb
    SensorSpi->write(0x00); // Dummy
    SensorSpi->write(0x00); // Dummy
    regVal = (uint16_t)(SensorSpi->write(0x00)) | \
             (uint16_t)(SensorSpi->write(0x00) << 8); 
    SensorNss = 1;
    SensorSpi->unlock();
    
    return regVal;
}

/* DEPRECATED FOR HTSENS ! */
void Sensor::Read_Pression(uint8_t address, uint8_t *buffer, uint8_t position)
{
    if (address == 0xFF)
    // dummy read
    {
        SensorNss = 0;
        
        uint8_t pong[] = "TEST";
        buffer[position] = pong[0];
        buffer[position + 1] = pong[1];
        buffer[position + 2] = pong[2];
        buffer[position + 3] = pong[3];
        
        SensorNss = 1;
    }
    
    else
    {
        SensorNss = 0;
        //Read_Register(READ_PAGE_OPCODE, address, buffer, position);
        SensorNss = 1; 
    }
}    
