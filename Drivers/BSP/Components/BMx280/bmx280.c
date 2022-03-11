/**
  ******************************************************************************
  * @file    bmx280.c
  * @brief   This file includes the HAL/LL driver for BME280/BMP280 sensor,
  * 		 with I2C/SPI communication
  ******************************************************************************
  */
#include "bmx280.h"

#ifdef HAL_SPI
/**
  * @brief  The function is used as SPI bus read
  * @retval	status indicator 0:success 1:error
  * @param  DevAddr		Target device address
  * @param  MemAddr  	Internal memory address
  * @param  pData 		Pointer to data buffer
  * @param  Len  		Amount of data to be read
  */
uint8_t SPI_bus_read(uint8_t DevAddr, uint8_t RegAddr, uint8_t *RegData,
		uint8_t len)
{
	uint8_t rslt 		= 0;
	uint8_t TxBuf[28] 	= {0,};
	uint8_t RxBuf[28] 	= {0,};
	uint8_t pos;

	/*	For the SPI mode only 7 bits of register addresses are used.
	The MSB of register address is declared the bit what functionality it is
	read/write (read as 1/write as BME280_INIT_VALUE)*/
	/*read routine is initiated register address is mask with 0x80*/
	TxBuf[0] = RegAddr | SPI_READ;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	rslt = HAL_SPI_TransmitReceive(&SPIx, (uint8_t *)(&TxBuf),
			(uint8_t *)(&RxBuf), len+1, 5);
	while (SPIx.State == HAL_SPI_STATE_BUSY) {};
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(5);

    if (rslt != HAL_OK)
    {
    	/* The BME280 API calls for 0 return value as a success,
    	 * and 1 returned as failure */
    	return 1;
    }

	for (pos = 0; pos < len; pos++) {
		*(RegData + pos) = RxBuf[pos + SPI_DATA_INDEX];
	}

	return rslt;
}

/**
  * @brief  The function is used as SPI bus write
  * @retval	status indicator 0:success 1:error
  * @param  DevAddr		Target device address
  * @param  MemAddr  	Internal memory address
  * @param  pData 		Pointer to data buffer
  * @param  Len  		Amount of data to be write
  */
uint8_t SPI_bus_write(uint8_t DevAddr, uint8_t RegAddr, uint8_t *RegData,
		uint8_t len)
{
	uint8_t rslt = 0;
	uint8_t TxBuf[28 * SPI_ADDRESS_INDEX];
	uint8_t pos = 0;
	uint8_t index = 0;
	for (pos = 0; pos < len; pos++) {
		/* the operation of (RegAddr++)&0x7F done as per the
		SPI communication protocol specified in the data sheet*/
		index = pos * SPI_ADDRESS_INDEX;
		TxBuf[index] = (RegAddr++) & SPI_WRITE;
		TxBuf[index + SPI_DATA_INDEX] = *(RegData + pos);
	}

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	rslt = HAL_SPI_Transmit(&SPIx, (uint8_t*)(&TxBuf), len*2, 100);
	while (SPIx.State == HAL_SPI_STATE_BUSY) {};
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

    if (rslt != HAL_OK)
    {
    	/* The BME280 API calls for 0 return value as a success,
    	 * and 1 returned as failure */
    	return 1;
    }

	return rslt;
}

/**
  * @brief  The function is used as delay
  * @param  millis		Target delay in ms
  */
void Delay(uint16_t millis)
{
	HAL_Delay(millis);
}

void SPI_Routine(Dev_HandleTypedef *dev)
{
	/*--------------------------------------------------------------------------*
	 *  By using bme280 the following structure parameter can be accessed
	 *	Bus write function pointer: BME280_WR_FUNC_PTR
	 *	Bus read function pointer: BME280_RD_FUNC_PTR
	 *	Delay function pointer: delay_msec
	 *--------------------------------------------------------------------------*/
	dev->bus_write	= SPI_bus_write;
	dev->bus_read	= SPI_bus_read;
	dev->bus_delay	= Delay;
	/*--------------------------------------------------------------------------*
	 *  Drop the chip select pin to low - this tells the BME280 to
	 *  use SPI mode. It will not respond to I2C commands until you
	 *  reset the power to it.
	 */
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}
#endif

#ifdef LL_SPI
/**
  * @brief  The function is used as SPI bus read
  * @retval	status indicator 0:success 1:error
  * @param  DevAddr		Target device address
  * @param  MemAddr  	Internal memory address
  * @param  pData 		Pointer to data buffer
  * @param  Len  		Amount of data to be read
  */
uint8_t SPI_bus_read(uint8_t DevAddr, uint8_t RegAddr, uint8_t *RegData,
		uint8_t len)
{
	uint8_t rslt		= 0;
	uint8_t pos			= 0;
	uint8_t TxCnt		= len + 1;
	uint8_t RxCnt		= len + 1;
	uint8_t TxBuf[28] 	= {0,};


	/* For the SPI mode only 7 bits of register addresses are used.
	The MSB of register address is declared the bit what functionality it is
	read/write (read as 1/write as BME280_INIT_VALUE)*/
	/*read routine is initiated register address is mask with 0x80*/
	TxBuf[0] = RegAddr | SPI_READ;
	__IO uint8_t (*TxPtr) = (uint8_t *)(&TxBuf);

	LL_SPI_SetTransferSize(SPIx, len + 1);
	LL_SPI_Enable(SPIx);
	LL_SPI_StartMasterTransfer(SPIx);
	LL_GPIO_ResetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);

	while ((TxCnt > 0UL) || (RxCnt > 0UL))
	{
		/* check TXP flag */
		if (LL_SPI_IsActiveFlag_TXP(SPIx) && (TxCnt > 0UL))
		{
			LL_SPI_TransmitData8(SPIx, *(TxPtr));
			TxPtr++;
			TxCnt--;
		}

		/* check RXP flag */
		if (LL_SPI_IsActiveFlag_RXP(SPIx) && (RxCnt > 0UL))
		{
			*(RegData + pos) = LL_SPI_ReceiveData8(SPIx);
			if (RxCnt != (len + 1))
			{
				pos++;
			}
			RxCnt--;
		}
	}

	while (LL_SPI_IsActiveFlag_EOT(SPIx) == 0) {};
	LL_SPI_ClearFlag_EOT(SPIx);
	LL_SPI_ClearFlag_TXTF(SPIx);
	LL_SPI_SuspendMasterTransfer(SPIx);
	LL_SPI_Disable(SPIx);
	LL_GPIO_SetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);

	return rslt;
}

/**
  * @brief  The function is used as SPI bus write
  * @retval	status indicator 0:success 1:error
  * @param  DevAddr		Target device address
  * @param  MemAddr  	Internal memory address
  * @param  pData 		Pointer to data buffer
  * @param  Len  		Amount of data to be write
  */
uint8_t SPI_bus_write(uint8_t DevAddr, uint8_t RegAddr, uint8_t *RegData,
		uint8_t len)
{
	uint8_t rslt = 0;
	uint8_t array[28 * SPI_ADDRESS_INDEX];
	uint8_t TxCnt = len + 1;
	uint8_t pos = 0;
	uint8_t index = 0;
	for (pos = 0; pos < len; pos++) {
		/* the operation of (RegAddr++)&0x7F done as per the
		SPI communication protocol specified in the data sheet*/
		index = pos * SPI_ADDRESS_INDEX;
		array[index] = (RegAddr++) & SPI_WRITE;
		array[index + SPI_DATA_INDEX] = *(RegData + pos);
	}

	__IO uint8_t (*TxPtr) = (uint8_t *)(&array);

	LL_SPI_SetTransferSize(SPIx, len * 2);
	LL_SPI_Enable(SPIx);
	LL_SPI_StartMasterTransfer(SPIx);
	LL_GPIO_ResetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);

	while (TxCnt > 0)
	{
		/* check TXP flag */
		if (LL_SPI_IsActiveFlag_TXP(SPIx))
		{
			LL_SPI_TransmitData8(SPIx, *(TxPtr));
			TxPtr++;
			TxCnt--;
		}

	}

	while (LL_SPI_IsActiveFlag_EOT(SPIx) == 0) {};
	LL_SPI_ClearFlag_EOT(SPIx);
	LL_SPI_ClearFlag_TXTF(SPIx);
	LL_SPI_SuspendMasterTransfer(SPIx);
	LL_SPI_Disable(SPIx);
	LL_GPIO_SetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);

	return rslt;
}

/**
  * @brief  The function is used as delay
  * @param  millis		Target delay in ms
  */
void Delay(uint16_t millis)
{
	LL_mDelay(millis);
}

void SPI_Routine(Dev_HandleTypedef *dev)
{
	/*--------------------------------------------------------------------------*
	 *  By using bme280 the following structure parameter can be accessed
	 *	Bus write function pointer: BME280_WR_FUNC_PTR
	 *	Bus read function pointer: BME280_RD_FUNC_PTR
	 *	Delay function pointer: delay_msec
	 *--------------------------------------------------------------------------*/
	dev->bus_write	= SPI_bus_write;
	dev->bus_read	= SPI_bus_read;
	dev->bus_delay	= Delay;
	/*--------------------------------------------------------------------------*
	 *  Drop the chip select pin to low - this tells the BME280 to
	 *  use SPI mode. It will not respond to I2C commands until you
	 *  reset the power to it.
	 */
	LL_GPIO_ResetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);
	LL_mDelay(10);
	LL_GPIO_SetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);
}
#endif

#ifdef HAL_I2C
/**
  * @brief  The function is used as I2C bus read
  * @retval	status indicator 0:success 1:error
  * @param  DevAddr		Target device address
  * @param  MemAddr  	Internal memory address
  * @param  pData 		Pointer to data buffer
  * @param  Len  		Amount of data to be read
  */
uint8_t I2C_bus_read(uint8_t DevAddr, uint8_t RegAddr, uint8_t *RegData,
		uint8_t len) {
	uint8_t rslt = 0;
	uint8_t array[I2C_BUFFER_LEN] = {0,};
	uint8_t pos;
	array[0] = RegAddr;

	while (HAL_I2C_IsDeviceReady(&I2Cx, (uint8_t)(DevAddr<<1), 3, 100) != 0) {};

    rslt = HAL_I2C_Mem_Read(&I2Cx, (uint8_t)(DevAddr<<1), (uint8_t)RegAddr,
    		I2C_MEMADD_SIZE_8BIT, (uint8_t*)(&array), len, 500);

    if (rslt != 0)
    {
    	/* The BME280 API calls for 0 return value as a success,
    	 * and 1 returned as failure */
    	return 1;
    }
	for (pos = 0; pos < len; pos++) {
		*(RegData + pos) = array[pos];
	}

	return rslt;
}

/**
  * @brief  The function is used as I2C bus write
  * @retval	status indicator 0:success 1:error
  * @param  DevAddr		Target device address
  * @param  MemAddr  	Internal memory address
  * @param  pData 		Pointer to data buffer
  * @param  Len  		Amount of data to be write
  */
uint8_t I2C_bus_write(uint8_t DevAddr, uint8_t RegAddr, uint8_t *RegData,
		uint8_t len) {
	uint8_t rslt = 0;

	while (HAL_I2C_IsDeviceReady(&I2Cx, (uint8_t)(DevAddr<<1), 3, 100) != 0) {};

    rslt = HAL_I2C_Mem_Write(&I2Cx, (uint8_t)(DevAddr<<1), (uint8_t)RegAddr,
    		I2C_MEMADD_SIZE_8BIT, (uint8_t*)(&RegData), len, 500);
	if (rslt != 0)
    {
        /* The BME280 API calls for 0 return value as a success,
         * and 1 returned as failure */
    	return 1;
    }
	return rslt;
}

/**
  * @brief  The function is used as delay
  * @param  millis		Target delay in ms
  */
void Delay(uint16_t millis)
{
	HAL_Delay(millis);
}

void I2C_Routine(Dev_HandleTypedef *dev)
{
	dev->bus_write	= I2C_bus_write;
	dev->bus_read	= I2C_bus_read;
	dev->bus_delay	= Delay;
	dev->addr		= I2C_ADDRESS;
}
#endif

#ifdef LL_I2C
/**
  * @brief  The function is used as I2C bus read
  * @retval	status indicator 0:success 1:error
  * @param  DevAddr		Target device address
  * @param  MemAddr  	Internal memory address
  * @param  pData 		Pointer to data buffer
  * @param  Len  		Amount of data to be read
  */
uint8_t I2C_bus_read(uint8_t DevAddr, uint8_t RegAddr, uint8_t *RegData,
		uint8_t len) {
	uint8_t rslt = 0;
    uint16_t XferCount = len;
    uint16_t XferSize = 0;

	/* Wait for I2C bus is free */
	while (LL_I2C_IsActiveFlag_BUSY(I2Cx)) {};

	if (!LL_I2C_IsEnabled(I2Cx))
	{
		LL_I2C_Enable(I2Cx);
	}

    /* Send Slave Address and Memory Address */
    LL_I2C_HandleTransfer(I2Cx, (uint8_t)(DevAddr<<1), LL_I2C_ADDRSLAVE_7BIT, 1,
    		LL_I2C_MODE_SOFTEND,
			LL_I2C_GENERATE_START_WRITE);

    while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {};

	 /* Send Memory Address */
    LL_I2C_TransmitData8(I2Cx, ((uint8_t)(uint16_t)(RegAddr & (uint16_t)0x00FF)));
    while (!LL_I2C_IsActiveFlag_TC(I2Cx)) {};

    /* Send Slave Address */
    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE */
    if (XferCount > 255U)
    {
        XferSize = 255U;
        LL_I2C_HandleTransfer(I2Cx, (uint8_t)(DevAddr<<1), LL_I2C_ADDRSLAVE_7BIT,
        		(uint8_t)XferSize, I2C_CR2_RELOAD ,
				LL_I2C_GENERATE_START_READ);
    }
    else
    {
        XferSize = XferCount;
        LL_I2C_HandleTransfer(I2Cx, (uint8_t)(DevAddr<<1), LL_I2C_ADDRSLAVE_7BIT,
        		(uint8_t)XferSize, I2C_CR2_AUTOEND ,
				LL_I2C_GENERATE_START_READ);
    }

    do
    {
        /* Wait until RXNE flag is set */
        while (!LL_I2C_IsActiveFlag_RXNE(I2Cx)) {};

        /* Read data from RXDR */
        *RegData = LL_I2C_ReceiveData8(I2Cx);

        /* Increment Buffer pointer */
        RegData++;
        XferCount--;
        XferSize--;

        if ((XferCount != 0U) && (XferSize == 0U))
        {
            /* Wait until TCR flag is set */
            while (!LL_I2C_IsActiveFlag_TCR(I2Cx)) {};

            if (XferCount > 255U)
            {
                XferSize = 255U;

                LL_I2C_HandleTransfer(I2Cx, (uint8_t)(DevAddr<<1),
                		LL_I2C_ADDRSLAVE_7BIT, (uint8_t)XferSize, I2C_CR2_RELOAD,
						LL_I2C_GENERATE_NOSTARTSTOP);
            }
            else
            {
                XferSize = XferCount;
                LL_I2C_HandleTransfer(I2Cx, (uint8_t)(DevAddr<<1),
                		LL_I2C_ADDRSLAVE_7BIT, (uint8_t)XferSize, I2C_CR2_AUTOEND,
						LL_I2C_GENERATE_NOSTARTSTOP);
            }
        }
    } while (XferCount > 0U);

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically
     * generated.
     * Wait until STOPF flag is reset */
    while (!LL_I2C_IsActiveFlag_STOP(I2Cx)) {};

	/* Clear NACKF Flag */
    LL_I2C_ClearFlag_NACK(I2Cx);

    /* Clear STOP Flag */
    LL_I2C_ClearFlag_STOP(I2Cx);

    /* Clear Configuration Register 2 */
    I2Cx->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R |
    		I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN));

	return rslt;
}

/**
  * @brief  The function is used as I2C bus write
  * @retval	status indicator 0:success 1:error
  * @param  DevAddr		Target device address
  * @param  MemAddr  	Internal memory address
  * @param  pData 		Pointer to data buffer
  * @param  Len  		Amount of data to be write
  */
uint8_t I2C_bus_write(uint8_t DevAddr, uint8_t RegAddr, uint8_t *RegData,
		uint8_t len) {
	uint8_t rslt = 0;
	uint16_t XferCount = len;
	uint8_t *TxBuffer = RegData;
	uint16_t XferSize = 0;

	/* Wait for I2C bus is free */
	while (LL_I2C_IsActiveFlag_BUSY(I2Cx)) {};

	if (!LL_I2C_IsEnabled(I2Cx))
	{
		LL_I2C_Enable(I2Cx);
	}

    /* Send Slave Address and Memory Address */
    LL_I2C_HandleTransfer(I2Cx, (uint8_t)(DevAddr<<1), LL_I2C_ADDRSLAVE_7BIT, 1,
    		I2C_CR2_RELOAD, LL_I2C_GENERATE_START_WRITE);

    while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)){};

	 /* Send Memory Address */
    LL_I2C_TransmitData8(I2Cx, ((uint8_t)(uint16_t)(RegAddr & (uint16_t)0x00FF)));
	while (!LL_I2C_IsActiveFlag_TCR(I2Cx)) {};

    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE */
    if (XferCount > 255U)
    {
        XferSize = 255U;
        LL_I2C_HandleTransfer(I2Cx, (uint8_t)(DevAddr<<1), LL_I2C_ADDRSLAVE_7BIT,
        		(uint8_t)XferSize, I2C_CR2_RELOAD , LL_I2C_GENERATE_NOSTARTSTOP);
    }
    else
    {
        XferSize = XferCount;
        LL_I2C_HandleTransfer(I2Cx, (uint8_t)(DevAddr<<1), LL_I2C_ADDRSLAVE_7BIT,
        		(uint8_t)XferSize, I2C_CR2_AUTOEND , LL_I2C_GENERATE_NOSTARTSTOP);
    }

    do
    {
        /* Wait until TXIS flag is set */
    	while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {};

        /* Write data to TXDR */
        LL_I2C_TransmitData8(I2Cx, *TxBuffer);

        /* Increment Buffer pointer */
        TxBuffer++;

        XferCount--;
        XferSize--;

        if ((XferCount != 0U) && (XferSize == 0U))
        {
            /* Wait until TCR flag is set */
        	while (!LL_I2C_IsActiveFlag_TCR(I2Cx)) {};

            if (XferCount > 255U)
            {
                XferSize = 255U;
                LL_I2C_HandleTransfer(I2Cx, (uint8_t)(DevAddr<<1), LL_I2C_ADDRSLAVE_7BIT,
                		(uint8_t)XferSize, I2C_CR2_RELOAD , LL_I2C_GENERATE_NOSTARTSTOP);
            }
            else
            {
                XferSize = XferCount;
                LL_I2C_HandleTransfer(I2Cx, (uint8_t)(DevAddr<<1), LL_I2C_ADDRSLAVE_7BIT,
                		(uint8_t)XferSize, I2C_CR2_AUTOEND , LL_I2C_GENERATE_NOSTARTSTOP);
            }
        }
    } while (XferCount > 0U);

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is reset */
    while (!LL_I2C_IsActiveFlag_STOP(I2Cx)) {};

    /* Clear NACKF Flag */
    LL_I2C_ClearFlag_NACK(I2Cx);

    /* Clear STOP Flag */
    LL_I2C_ClearFlag_STOP(I2Cx);

    /* Clear Configuration Register 2 */
    I2Cx->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R |
    		I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN));

	return rslt;
}

/**
  * @brief  The function is used as delay
  * @param  millis		Target delay in ms
  */
void Delay(uint16_t millis)
{
	LL_mDelay(millis);
}

void I2C_Routine(Dev_HandleTypedef *dev)
{
	dev->bus_write	= I2C_bus_write;
	dev->bus_read	= I2C_bus_read;
	dev->bus_delay	= Delay;
	dev->addr		= I2C_ADDRESS;
}

#endif

/*!
 * @brief This API performs the soft reset of the sensor, All the registry
 * will be erase after this process.
 */
static uint8_t soft_reset(Dev_HandleTypedef *dev)
{
	uint8_t rslt = 0;
	uint8_t CMD = CMD_SOFT_RESET;

	/* Write the soft reset command in the sensor */
	if (dev->bus_write(dev->addr, REG_RESET, &CMD, 1) != 0)
	{
		return 1;
	}

	// Wait until finished copying over the NVP data.
	while (1)
	{
		uint8_t status;
		rslt = dev->bus_read(dev->addr, REG_STATUS, &status, 1);

		if ((status & 1) == 0) break;

		dev->bus_delay(2);
	}

	return rslt;
}

/*!
 * @brief This internal API reads the calibration data from the sensor, parse
 * it and store in the device structure.
 */
static uint8_t get_calib_data(Dev_HandleTypedef *dev)
{
	uint8_t rslt = 0;
	uint8_t d_calib[CALIB_DATA_SIZE];
	uint8_t RegAddr = (0x88); // BME280_TEMP_PRESS_CALIB_DATA_ADDR

	rslt = dev->bus_read(dev->addr, RegAddr, d_calib, CALIB_DATA_SIZE);

	if (rslt == 0)
	{
		dev->calibs.dig_T1 = CONCAT_BYTES(d_calib[1], d_calib[0]);
		dev->calibs.dig_T2 = (int16_t)CONCAT_BYTES(d_calib[3], d_calib[2]);
		dev->calibs.dig_T3 = (int16_t)CONCAT_BYTES(d_calib[5], d_calib[4]);

		dev->calibs.dig_P1 = CONCAT_BYTES(d_calib[7], d_calib[6]);
		dev->calibs.dig_P2 = (int16_t)CONCAT_BYTES(d_calib[9], d_calib[8]);
		dev->calibs.dig_P3 = (int16_t)CONCAT_BYTES(d_calib[11], d_calib[10]);
		dev->calibs.dig_P4 = (int16_t)CONCAT_BYTES(d_calib[13], d_calib[12]);
		dev->calibs.dig_P5 = (int16_t)CONCAT_BYTES(d_calib[15], d_calib[14]);
		dev->calibs.dig_P6 = (int16_t)CONCAT_BYTES(d_calib[17], d_calib[16]);
		dev->calibs.dig_P7 = (int16_t)CONCAT_BYTES(d_calib[19], d_calib[18]);
		dev->calibs.dig_P8 = (int16_t)CONCAT_BYTES(d_calib[21], d_calib[20]);
		dev->calibs.dig_P9 = (int16_t)CONCAT_BYTES(d_calib[23], d_calib[22]);
		dev->calibs.dig_H1 = d_calib[25];

		if (dev->id == BME280)
		{
			RegAddr = (0xE1);	// BME280_HUMIDITY_CALIB_DATA_ADDR
			rslt = dev->bus_read(dev->addr, RegAddr, d_calib,
					HUMIDITY_CALIB_DATA_SIZE);

			if (rslt == 0)
			{
			    int16_t dig_h4_lsb;
			    int16_t dig_h4_msb;
			    int16_t dig_h5_lsb;
			    int16_t dig_h5_msb;

				dev->calibs.dig_H2 = (int16_t)CONCAT_BYTES(d_calib[1], d_calib[0]);
				dev->calibs.dig_H3 = d_calib[2];
			    dig_h4_msb = (int16_t)(int8_t)d_calib[3] * 16;
			    dig_h4_lsb = (int16_t)(d_calib[4] & 0x0F);
			    dev->calibs.dig_H4 = dig_h4_msb | dig_h4_lsb;
			    dig_h5_msb = (int16_t)(int8_t)d_calib[5] * 16;
			    dig_h5_lsb = (int16_t)(d_calib[4] >> 4);
			    dev->calibs.dig_H5 = dig_h5_msb | dig_h5_lsb;
			    dev->calibs.dig_H6 = (int8_t)d_calib[6];

			}else{
				return 1;
			}
		}else{
			return 1;
		}
	}else{
		return 1;
	}

	return rslt;
}

/*!
 * @brief This function called to initiate sensor with all parameter
 * \dev: device
 * \return: communication result
 */
uint8_t BMX280_Init(Dev_HandleTypedef *dev)
{
#if defined(HAL_SPI) || defined(LL_SPI)
	SPI_Routine(dev);
#else
	I2C_Routine(dev);
#endif

	/* read Chip Id */
	if (dev->bus_read(dev->addr, REG_ID, &dev->id, 1) != 0)
	{
		return 1;
	}

	/* Check for the correct chip id */
	if (dev->id != BMP280 && dev->id != BME280)
	{
		return 1;
	}

	/* Reset the sensor */
	if (soft_reset(dev) != 0)
	{
		return 1;
	}

	/* Read the calibration data */
	if (get_calib_data(dev) != 0)
	{
		return 1;
	}

	return 0;
}

/*!
 * @brief This API gets the power mode of the sensor.
 */
static uint8_t get_sensor_mode(Dev_HandleTypedef *dev, uint8_t *current_mode)
{
	uint8_t RegData;

	/* Read the power mode register */
	if (dev->bus_read(dev->addr, REG_CTRL, &RegData, 1) != 0)
	{
		return 1;
	}

	/* Assign the power mode in the device structure */
	*current_mode = RegData & (0x03);

	return 0;
}

/*!
 * @brief This API sets the oversampling, filter and standby duration
 * (normal mode) settings in the sensor.
 */
uint8_t set_sensor_settings(Dev_HandleTypedef *dev)
{
	uint8_t rslt = 0;
	uint8_t current_mode;

	/* Check sensor mode, sleep sensor if not in sleep mode */
	if (get_sensor_mode(dev, &current_mode) == 0 && current_mode != MODE_SLEEP)
	{
		/* Soft_reset to put sensor into sleep mode */
		if (soft_reset(dev) != 0)
		{
			return 1;
		}
	}

	/* sets the oversampling settings for pressure, temperature and
	 * humidity in the sensor.
	 * */
	if (dev->id == BME280)
	{
		uint8_t ctrl_hum = dev->params.osr_h & (0x07);
		rslt += dev->bus_write(dev->addr, REG_CTRL_HUM, &ctrl_hum, 1);
	}

	uint8_t ctrl = 0;
	ctrl = SET_BITS(ctrl, (0x1C), (0x02), dev->params.osr_p);
	ctrl = SET_BITS(ctrl, (0xE0), (0x05), dev->params.osr_t);
	rslt += dev->bus_write(dev->addr, REG_CTRL, &ctrl, 1);

	/* Set filter and/or standby settings */
	uint8_t config = 0;
	config = SET_BITS(config, (0x1C), (0x02), dev->params.filter);
	config = SET_BITS(config, (0xE0), (0x05), dev->params.standby);
	rslt += dev->bus_write(dev->addr, REG_CONFIG, &config, 1);

	/* Calculate max measurement time so that data can be read*/
	float Th = (dev->params.osr_h == 0) ? 0 : ((2.3* dev->params.osr_h) + 0.575);
	float Tp = (dev->params.osr_p == 0) ? 0 : ((2.3* dev->params.osr_p) + 0.575);
	float Tt = (dev->params.osr_t == 0) ? 0 : (2.3* dev->params.osr_t);
	dev->max_delay = (uint16_t)(1.25 + Tt + Tp + Th + 0.5);

	return rslt;
}

/*!
 * @brief This API sets the power mode of the sensor.
 */
uint8_t set_sensor_mode(Dev_HandleTypedef *dev, uint8_t sensor_mode)
{
	uint8_t RegData;

	/* Read the power mode register */
	if (dev->bus_read(dev->addr, REG_CTRL, &RegData, 1) != 0)
	{
		return 1;
	}

	/* Set the power mode */
	RegData = SET_BITS_POS_0(RegData, (0x03), (0x00), sensor_mode);

	/* Write the power mode in the register */
	if (dev->bus_write(dev->addr, REG_CTRL, &RegData, 1) != 0)
	{
		return 1;
	}

	return 0;
}

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in integer data type.
 */
static int32_t compensate_temperature(Dev_HandleTypedef *dev)
{
    int32_t var1;
    int32_t var2;
    int32_t temperature;
    int32_t temperature_min = -4000;
    int32_t temperature_max = 8500;

    var1 = (int32_t)((dev->udata.temperature / 8) - ((int32_t)dev->calibs.dig_T1 * 2));
    var1 = (var1 * ((int32_t)dev->calibs.dig_T2)) / 2048;
    var2 = (int32_t)((dev->udata.temperature / 16) - ((int32_t)dev->calibs.dig_T1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)dev->calibs.dig_T3)) / 16384;
    dev->calibs.dig_TF = var1 + var2;
    temperature = (dev->calibs.dig_TF * 5 + 128) / 256;

    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type.
 */
static uint32_t compensate_pressure(Dev_HandleTypedef *dev)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    uint32_t var5;
    uint32_t pressure;
    uint32_t pressure_min = 30000;
    uint32_t pressure_max = 110000;

    var1 = (((int32_t)dev->calibs.dig_TF) / 2) - (int32_t)64000;
    var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)dev->calibs.dig_P6);
    var2 = var2 + ((var1 * ((int32_t)dev->calibs.dig_P5)) * 2);
    var2 = (var2 / 4) + (((int32_t)dev->calibs.dig_P4) * 65536);
    var3 = (dev->calibs.dig_P3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
    var4 = (((int32_t)dev->calibs.dig_P2) * var1) / 2;
    var1 = (var3 + var4) / 262144;
    var1 = (((32768 + var1)) * ((int32_t)dev->calibs.dig_P1)) / 32768;

    /* avoid exception caused by division by zero */
    if (var1)
    {
        var5 = (uint32_t)((uint32_t)1048576) - dev->udata.pressure;
        pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;

        if (pressure < 0x80000000)
        {
            pressure = (pressure << 1) / ((uint32_t)var1);
        }
        else
        {
            pressure = (pressure / (uint32_t)var1) * 2;
        }

        var1 = (((int32_t)dev->calibs.dig_P9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
        var2 = (((int32_t)(pressure / 4)) * ((int32_t)dev->calibs.dig_P8)) / 8192;
        pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + dev->calibs.dig_P7) / 16));

        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else
    {
        pressure = pressure_min;
    }

    return pressure;
}

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in integer data type.
 */
static uint32_t compensate_humidity(Dev_HandleTypedef *dev)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    uint32_t humidity;
    uint32_t humidity_max = 102400;

    var1 = dev->calibs.dig_TF - ((int32_t)76800);
    var2 = (int32_t)(dev->udata.humidity * 16384);
    var3 = (int32_t)(((int32_t)dev->calibs.dig_H4) * 1048576);
    var4 = ((int32_t)dev->calibs.dig_H5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)dev->calibs.dig_H6)) / 1024;
    var3 = (var1 * ((int32_t)dev->calibs.dig_H3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)dev->calibs.dig_H2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)dev->calibs.dig_H1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    humidity = (uint32_t)(var5 / 4096);

    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }

    return humidity;
}

/*!
 *  @brief This API is used to parse the pressure, temperature and
 *  humidity data and store it in the bme280_uncomp_data structure instance.
 */
static void parse_sensor_data(Dev_HandleTypedef *dev, const uint8_t *RegData)
{
	/* Variables to store the sensor data */
	uint32_t data_xlsb;
	uint32_t data_lsb;
	uint32_t data_msb;

	/* Store the parsed register values for pressure data */
	data_msb = (uint32_t)RegData[0] << 12;
	data_lsb = (uint32_t)RegData[1] << 4;
	data_xlsb = (uint32_t)RegData[2] >> 4;
	dev->udata.pressure = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for temperature data */
    data_msb = (uint32_t)RegData[3] << 12;
    data_lsb = (uint32_t)RegData[4] << 4;
    data_xlsb = (uint32_t)RegData[5] >> 4;
    dev->udata.temperature = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for humidity data */
    data_msb = (uint32_t)RegData[6] << 8;
    data_lsb = (uint32_t)RegData[7];
    dev->udata.humidity = data_msb | data_lsb;

    dev->data.temperature = compensate_temperature(dev);
    dev->data.pressure = compensate_pressure(dev);
    dev->data.humidity = compensate_humidity(dev);
}

/*!
 * @brief This function called to get temperature/pressure/humididty
 * reading from the sensor
 */
uint8_t get_sensor_data(Dev_HandleTypedef *dev)
{
	/* Read the pressure and temperature data from the sensor */
	if (dev->bus_read(dev->addr, REG_DATA, dev->burst, 8) != 0)
	{
		return 1;
	}

	parse_sensor_data(dev, dev->burst);

	return 0;
}
