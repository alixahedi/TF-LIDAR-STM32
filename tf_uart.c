// src/tf_uart.c
#include "tf_lidar.h"
#include <string.h>

#define UART_TIMEOUT_MS 100U

/**
 * @brief  Receive one raw frame over UART (blocking).
 * @param  lidar: Pointer to TF LIDAR handle
 * @param  frame: Buffer to store received bytes
 * @retval true on success, false on HAL error or timeout
 */
static bool tf_uart_read_frame(tf_lidar_t *lidar, uint8_t *frame)
{
    return HAL_UART_Receive(lidar->iface.huart,
                            frame,
                            lidar->model->uart_frame_length,
                            UART_TIMEOUT_MS) == HAL_OK;
}

/**
 * @brief  Initialize UART transport.
 * @see    tf_lidar_init_uart in tf_lidar.h
 */
bool tf_lidar_init_uart(tf_lidar_t *lidar,
                        const tf_lidar_model_t *model,
                        UART_HandleTypeDef *huart)
{
    lidar->model     = model;
    lidar->transport = TF_TRANSPORT_UART;
    lidar->iface.huart = huart;

    huart->Init.BaudRate   = model->uart_baud;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits   = UART_STOPBITS_1;
    huart->Init.Parity     = UART_PARITY_NONE;
    huart->Init.Mode       = UART_MODE_RX;
    return HAL_UART_Init(huart) == HAL_OK;
}

/**
 * @brief  Read a measurement via UART.
 * @see    tf_lidar_read in tf_lidar.h
 */
bool tf_lidar_read_uart(tf_lidar_t *lidar, float *distance_m, float *signal)
{
    uint8_t frame[32];
    if (!tf_uart_read_frame(lidar, frame))
        return false;

    uint16_t raw_d = ((uint16_t)frame[lidar->model->offset_distance] << 8)
                   | frame[lidar->model->offset_distance + 1];
    uint16_t raw_s = ((uint16_t)frame[lidar->model->offset_signal] << 8)
                   | frame[lidar->model->offset_signal + 1];

    *distance_m = raw_d * lidar->model->distance_scale;
    *signal     = raw_s * lidar->model->signal_scale;
    return true;
}
