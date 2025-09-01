#ifndef __SENSOR_H_
#define __SENSOR_H_

#include "mbed.h"

/* DEPRECATED FOR HTSENS ! */
#define READ_REGISTER_OPCODE     0x84
#define WRITE_REGISTER_OPCODE    0x74
#define READ_PAGE_OPCODE         0x88

/* DEPRECATED FOR HTSENS ! */
#define RF_PWR_CTRL     0xA0
#define CLK_CTRL        0x80       
#define SENS_CTRL_0     0x81
#define SENS_CTRL_1     0x82
#define TEST_ANA_SEL    0x83
#define PWR_STAT        0x90
#define PWR_TEST_STAT   0x91


class Sensor
{
public:
    /*!
     * \brief Constructor for HTSENS with SPI support
     *
     * Represents the physical connectivity 
     */
    Sensor(PinName mosi, PinName miso, PinName sclk, PinName nss);
    ~Sensor();
    
    /*!
     * \brief Wake-up the sensor 
     *
     * 
     */  
     void WakeUp();  
             
     
    /*!
     * \brief Write a single byte of data to the sensor memory
     *
     * \param [in]  opcode        The opcode to write in the right section
     * \param [in]  address       The address of the register to write in
     * \param [in]  value1        The MSB of the data to be written in sensor register
     * \param [in]  value2        The LSB of the data to be written in sensor register
     */
    void Write_Register(uint8_t opcode, uint8_t address, uint8_t value1, uint8_t value2  );

    /*!
     * \brief Read data from the sensor memory
     *
     * \param [in]  opcode        The opcode to read in the right section
     * \param [in]  address       The address of the register to read in
     * \param [out] buffer        The buffer that will contain the register value
     * \param [in]  position      The position where to start to write into the buffer 
     *
     */
    uint16_t Read_Register(uint16_t address);
    
        /*!
     * \brief Read Pression ata from the sensor memory
     *
     * \param [in]  address       The address of the page to read in
     * \param [out] buffer        The buffer that holds data read from sensor
     * \param [in]  position      The position where to start to write into the buffer 
     *
     */
    void Read_Pression( uint8_t address, uint8_t *buffer, uint8_t position );

protected:

    SPI *SensorSpi;                                  //!< The SPI object used to communicate with the HTSens
    DigitalOut SensorNss;                            //!< The pin connected to HTSens chip select (active low)

};

#endif // __SENSOR_H_
