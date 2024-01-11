#include "main.h"
#include "i2c_slave.h"
#include "stdlib.h"

#define Rx_Size 3

extern I2C_HandleTypeDef hi2c1;

uint8_t Rx_Buffer[Rx_Size];
int16_t angle = 0;
uint8_t connected_check = 0;

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    if (TransferDirection == I2C_DIRECTION_TRANSMIT)
    {
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, Rx_Buffer, Rx_Size, I2C_FIRST_FRAME);
    }
    else
    {
        Error_Handler();
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    angle = (Rx_Buffer[1] << 8) | Rx_Buffer[2];
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    connected_check = 1;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_EnableListen_IT(hi2c);
}

void Init_I2C_Slave()
{
    if (HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

int16_t Return_Data()
{
    return angle;
}

uint8_t Return_Connected()
{
    return connected_check;
}