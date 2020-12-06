/*
 * MAX31856.h
 *
 *  Created on: Oct 31, 2020
 *      Author: arielarias
 */

#ifndef SRC_MAX31856_H_
#define SRC_MAX31856_H_

#include "stm32f1xx_hal.h"
#include <stdlib.h>

// MAX31856 Registers
// Register 0x00: CR0
#define CR0_AUTOMATIC_CONVERSION                0x80
#define CR0_ONE_SHOT                            0x40
#define CR0_OPEN_CIRCUIT_FAULT_TYPE_K           0x10    // Type-K is 10 to 20 Ohms
#define CR0_COLD_JUNCTION_DISABLED              0x08
#define CR0_FAULT_INTERRUPT_MODE                0x04
#define CR0_FAULT_CLEAR                         0x02
#define CR0_NOISE_FILTER_50HZ                   0x01
// Register 0x02: MASK
#define MASK_COLD_JUNCTION_HIGH_FAULT           0x20
#define MASK_COLD_JUNCTION_LOW_FAULT            0x10
#define MASK_THERMOCOUPLE_HIGH_FAULT            0x08
#define MASK_THERMOCOUPLE_LOW_FAULT             0x04
#define MASK_VOLTAGE_UNDER_OVER_FAULT           0x02
#define MASK_THERMOCOUPLE_OPEN_FAULT            0x01
// Register 0x0F: SR
#define SR_FAULT_COLD_JUNCTION_OUT_OF_RANGE     0x80
#define SR_FAULT_THERMOCOUPLE_OUT_OF_RANGE      0x40
#define SR_FAULT_COLD_JUNCTION_HIGH             0x20
#define SR_FAULT_COLD_JUNCTION_LOW              0x10
#define SR_FAULT_THERMOCOUPLE_HIGH              0x08
#define SR_FAULT_THERMOCOUPLE_LOW               0x04
#define SR_FAULT_UNDER_OVER_VOLTAGE             0x02
#define SR_FAULT_OPEN                           0x01

#define MAX31856_CR0_REG 0x00         ///< Config 0 register
#define MAX31856_CR0_AUTOCONVERT 0x80 ///< Config 0 Auto convert flag
#define MAX31856_CR0_1SHOT 0x40       ///< Config 0 one shot convert flag
#define MAX31856_CR0_OCFAULT1 0x20    ///< Config 0 open circuit fault 1 flag
#define MAX31856_CR0_OCFAULT0 0x10    ///< Config 0 open circuit fault 0 flag
#define MAX31856_CR0_CJ 0x08          ///< Config 0 cold junction disable flag
#define MAX31856_CR0_FAULT 0x04       ///< Config 0 fault mode flag
#define MAX31856_CR0_FAULTCLR 0x02    ///< Config 0 fault clear flag

#define MAX31856_CR1_REG 0x01  ///< Config 1 register
#define MAX31856_MASK_REG 0x02 ///< Fault Mask register
#define MAX31856_CJHF_REG 0x03 ///< Cold junction High temp fault register
#define MAX31856_CJLF_REG 0x04 ///< Cold junction Low temp fault register
#define MAX31856_LTHFTH_REG                                                    \
  0x05 ///< Linearized Temperature High Fault Threshold Register, MSB
#define MAX31856_LTHFTL_REG                                                    \
  0x06 ///< Linearized Temperature High Fault Threshold Register, LSB
#define MAX31856_LTLFTH_REG                                                    \
  0x07 ///< Linearized Temperature Low Fault Threshold Register, MSB
#define MAX31856_LTLFTL_REG                                                    \
  0x08 ///< Linearized Temperature Low Fault Threshold Register, LSB
#define MAX31856_CJTO_REG 0x09  ///< Cold-Junction Temperature Offset Register
#define MAX31856_CJTH_REG 0x0A  ///< Cold-Junction Temperature Register, MSB
#define MAX31856_CJTL_REG 0x0B  ///< Cold-Junction Temperature Register, LSB
#define MAX31856_LTCBH_REG 0x0C ///< Linearized TC Temperature, Byte 2
#define MAX31856_LTCBM_REG 0x0D ///< Linearized TC Temperature, Byte 1
#define MAX31856_LTCBL_REG 0x0E ///< Linearized TC Temperature, Byte 0
#define MAX31856_SR_REG 0x0F    ///< Fault Status Register

#define MAX31856_FAULT_CJRANGE                                                 \
  0x80 ///< Fault status Cold Junction Out-of-Range flag
#define MAX31856_FAULT_TCRANGE                                                 \
  0x40 ///< Fault status Thermocouple Out-of-Range flag
#define MAX31856_FAULT_CJHIGH                                                  \
  0x20 ///< Fault status Cold-Junction High Fault flag
#define MAX31856_FAULT_CJLOW 0x10 ///< Fault status Cold-Junction Low Fault flag
#define MAX31856_FAULT_TCHIGH                                                  \
  0x08 ///< Fault status Thermocouple Temperature High Fault flag
#define MAX31856_FAULT_TCLOW                                                   \
  0x04 ///< Fault status Thermocouple Temperature Low Fault flag
#define MAX31856_FAULT_OVUV                                                    \
  0x02 ///< Fault status Overvoltage or Undervoltage Input Fault flag
#define MAX31856_FAULT_OPEN                                                    \
  0x01 ///< Fault status Thermocouple Open-Circuit Fault flag

/** Noise filtering options enum. Use with setNoiseFilter() */
typedef enum
{
    NOISE_FILTER_60HZ,
    NOISE_FILTER_50HZ
} noise_filter_t;

/** Multiple types of thermocouples supported */
typedef enum
{
    TC_TYPE_B = 0b0000,
    TC_TYPE_E = 0b0001,
    TC_TYPE_J = 0b0010,
    TC_TYPE_K = 0b0011,
    TC_TYPE_N = 0b0100,
    TC_TYPE_R = 0b0101,
    TC_TYPE_S = 0b0110,
    TC_TYPE_T = 0b0111,
    V_MODE_G8 = 0b1000,
    V_MODE_G32 = 0b1100,
} thermocoupletype_t;

typedef enum
{
    AVG_1_SAMPLE   = 0x00,
    AVG_2_SAMPLES  = 0x10,
    AVG_4_SAMPLES  = 0x20,
    AVG_8_SAMPLES  = 0x30,
    AVG_16_SAMPLES = 0x40,
} averageSample_t;

/** Temperature conversion mode */
typedef enum
{
    CONV_ONESHOT,
    CONV_ONESHOT_NOWAIT,
    CONV_CONTINUOUS
} conversion_mode_t;

class MAX31856
{

private:
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *port_cs;
    uint16_t pin_cs;
    //Adafruit_SPIDevice spi_dev;
    bool initialized = false;

    conversion_mode_t conversionMode;

    void readRegisterN( uint8_t addr, uint8_t * buffer, uint16_t n );

    uint8_t readRegister8( uint8_t addr );
    uint16_t readRegister16( uint8_t addr );
    uint32_t readRegister24( uint8_t addr );

    void writeRegister8( uint8_t addr, uint8_t reg );

public:

    virtual ~MAX31856();

    MAX31856( SPI_HandleTypeDef * spi, GPIO_TypeDef * GPIOx_cs,
            uint16_t Pin_cs );

    bool begin( thermocoupletype_t tipeThermo, averageSample_t averageSamples,
            conversion_mode_t convMode );

    void setConversionMode( conversion_mode_t mode );
    conversion_mode_t getConversionMode( void );

    void setThermocoupleType( thermocoupletype_t type );
    thermocoupletype_t getThermocoupleType( void );

    uint8_t readFault( void );

    void triggerOneShot( void );
    bool conversionComplete( void );

    float readCJTemperature( void );
    float readThermocoupleTemperature( void );

    void setTempFaultThreshholds( float flow, float fhigh );
    void setColdJunctionFaultThreshholds( int8_t low, int8_t high );
    void setNoiseFilter( noise_filter_t noiseFilter );

};

#endif /* SRC_MAX31856_H_ */
