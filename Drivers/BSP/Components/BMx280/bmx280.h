/**
  ******************************************************************************
  * @file    bmx280.h
  * @brief   This file contains all the constants parameters for the BME280 or
  * 		 BMP280 sensor
  ******************************************************************************
  * @attention
  * Usage:
  *		Select bus driver communication by un-comment related selection
  *
  ******************************************************************************
  */
#ifndef __BMX280_H__
#define __BMX280_H__

/* Select communication method */
//#define HAL_SPI
//#define LL_SPI
#define HAL_I2C
//#define LL_I2C

/*---------------------------------------------------------------------------*/
/* Includes*/
/*---------------------------------------------------------------------------*/
#include "main.h"

#if defined(HAL_SPI) || defined(LL_SPI)
#include "spi.h"

#if defined(HAL_SPI)
#define SPIx				hspi1
#else
#define SPIx				SPI1
#endif


#define SPI_DATA_INDEX		(1)
#define SPI_ADDRESS_INDEX	(2)
#define SPI_BUFFER_LEN		(28)
#define SPI_READ			(0x80)
#define SPI_WRITE			(0x7F)
#endif

#if defined(HAL_I2C) || defined(LL_I2C)
#include "i2c.h"

#if defined(HAL_I2C)
#define I2Cx				hi2c2
#else
#define I2Cx				I2C2
#endif

#define	I2C_BUFFER_LEN		(28)
/* Master I2C Address(SDO LOW) = (0x76)
 * Slave I2C Address(SDO HIGH) = (0x77)
 */
#define I2C_ADDRESS			(0x76)
#endif

/****************************************************/
/**\name	CHIP ID DEFINITIONS  */
/***************************************************/
#define BMP280		(0x58)
#define BME280		(0x60)

/**
 * BMx280 registers
 */
#define REG_ID          (0xD0)
#define REG_CTRL_HUM    (0xF2)
#define REG_STATUS		(0xF3)
#define REG_CTRL        (0xF4)
#define REG_CONFIG      (0xF5)
#define REG_DATA		(0xF7)
#define REG_RESET		(0xE0)
#define CMD_SOFT_RESET	(0xB6)

/****************************************************/
/**\name	DEFINITIONS FOR ARRAY SIZE OF DATA   */
/***************************************************/
#define	CALIB_DATA_SIZE					(26)
#define HUMIDITY_CALIB_DATA_SIZE		(7)

/****************************************************/
/**\name	BUS READ AND WRITE FUNCTION POINTERS */
/***************************************************/
#define BUS_WRITE_FUNC(device_addr, register_addr,\
		register_data, wr_len) bus_write(device_addr, register_addr,\
		register_data, wr_len)

#define BUS_READ_FUNC(device_addr, register_addr,\
		register_data, rd_len) bus_read(device_addr, register_addr,\
		register_data, rd_len)

#define BUS_DELAY_FUNC(millis) bus_delay(millis)

#define WR_FUNC_PTR\
		uint8_t (*bus_write)(uint8_t, uint8_t,\
		uint8_t *, uint8_t)

#define RD_FUNC_PTR\
		uint8_t (*bus_read)(uint8_t, uint8_t,\
		uint8_t *, uint8_t)

#define DL_FUNC_PTR\
		void (*bus_delay)(uint16_t)

/**************************************************************/
/**\name	Macro DEFINITIONS                         */
/**************************************************************/
#define SET_BITS(reg_data, MSK, POS, data) \
	((reg_data & ~(MSK)) | ((data << POS) & MSK))

#define SET_BITS_POS_0(reg_data, MSK, POS, data) \
    ((reg_data & ~(MSK)) | (data & MSK))

#define GET_BITS(reg_data, MSK, POS) \
	((reg_data & MSK) >> POS)

#define GET_BITS_POS_0(reg_data, MSK) \
	(reg_data & MSK)

#define CONCAT_BYTES(msb, lsb) \
	(((uint16_t)msb << 8) | (uint16_t)lsb)
/**************************************************************/
/**\name	STRUCTURE DEFINITIONS                         */
/**************************************************************/

/**
 * Mode of BMx280 module operation.
 * Forced - Measurement is initiated by user and goes back to sleep
 * Normal - Continues measurement with standby time
 * Sleep  - No measurement but the registers are available for reading
 */

#define MODE_SLEEP		(0x00)
#define MODE_FORCED		(0x01)
#define MODE_NORMAL		(0x03)

/**\name Filter coefficient selection macros */
#define FILTER_COEFF_OFF	(0x00)
#define FILTER_COEFF_2		(0x01)
#define FILTER_COEFF_4		(0x02)
#define FILTER_COEFF_8		(0x03)
#define FILTER_COEFF_16		(0x04)

/**
 * Pressure oversampling settings
 */
#define NO_OVERSAMPLING		(0x00)
#define OVERSAMPLING_1X		(0x01)
#define OVERSAMPLING_2X		(0x02)
#define OVERSAMPLING_4X		(0x03)
#define OVERSAMPLING_8X		(0x04)
#define OVERSAMPLING_16X	(0x05)

/**
 * Stand by time between measurements in normal mode
 */
#define STANDBY_0_5_MS		(0x00)	/* stand by time 0.5ms */
#define STANDBY_62_5_MS		(0x01)	/* stand by time 62.5ms */
#define STANDBY_125_MS		(0x02)	/* stand by time 125ms */
#define STANDBY_250_MS		(0x03)	/* stand by time 250ms */
#define STANDBY_500_MS		(0x04)	/* stand by time 500ms */
#define STANDBY_1000_MS		(0x05) 	/* stand by time 1s */
#define STANDBY_10_MS		(0x06) 	/* stand by time 2s BMP280, 10ms BME280 */
#define STANDBY_20_MS		(0x07)  /* stand by time 4s BMP280, 20ms BME280 */

/*!
 * @brief Calibration data
 */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
    int32_t	 dig_TF;
} dev_calib_t;

/*!
 * @brief bme280 sensor parameters structure which comprises of mode,
 * oversampling and filter parameters.
 */
typedef struct {
    uint8_t osr_p;
    uint8_t osr_t;
    uint8_t osr_h;
    uint8_t filter;
    uint8_t standby;
}dev_params_t;

typedef struct {
	uint32_t humidity;
	uint32_t pressure;
	uint32_t temperature;
}dev_udata_t;

typedef struct {
	float humidity;
	float pressure;
	float temperature;
}dev_data_t;

/*!
 * @brief This structure holds BME280 initialization parameters
 */
typedef struct {
	uint16_t addr;
	uint8_t  id;
	uint16_t max_delay;
	dev_calib_t calibs;
    dev_params_t params;
    dev_data_t data;
    WR_FUNC_PTR;
    RD_FUNC_PTR;
    DL_FUNC_PTR;
    dev_data_t udata;
    uint8_t burst[8];
} Dev_HandleTypedef;


/**************************************************************/
/**\name	EXTERNAL FUNCTIONS                                */
/**************************************************************/

uint8_t BMX280_Init(Dev_HandleTypedef *dev);
uint8_t set_sensor_settings(Dev_HandleTypedef *dev);
uint8_t set_sensor_mode(Dev_HandleTypedef *dev, uint8_t sensor_mode);
uint8_t get_sensor_data(Dev_HandleTypedef *dev);
#endif
