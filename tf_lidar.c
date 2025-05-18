// src/tf_lidar.c
#include "tf_lidar.h"

/**
 * @brief  Read a measurement from the sensor over the active transport.
 * @see    tf_lidar_read in tf_lidar.h
 */
bool tf_lidar_read(tf_lidar_t *lidar, float *distance_m, float *signal)
{
    switch (lidar->transport) {
    case TF_TRANSPORT_UART:
        return tf_lidar_read_uart(lidar, distance_m, signal);
    case TF_TRANSPORT_CAN:
        return tf_lidar_read_can(lidar, distance_m, signal);
    case TF_TRANSPORT_FDCAN:
        return tf_lidar_read_fdcan(lidar, distance_m, signal);
    default:
        return false;
    }
}

/**
 * @brief  Switch active transport to UART.
 * @see    tf_lidar_switch_to_uart in tf_lidar.h
 */
bool tf_lidar_switch_to_uart(tf_lidar_t *lidar, UART_HandleTypeDef *huart)
{
    return tf_lidar_init_uart(lidar, lidar->model, huart);
}

/**
 * @brief  Switch active transport to classic CAN.
 * @see    tf_lidar_switch_to_can in tf_lidar.h
 */
bool tf_lidar_switch_to_can(tf_lidar_t *lidar, CAN_HandleTypeDef *hcan)
{
    return tf_lidar_init_can(lidar, lidar->model, hcan);
}

/**
 * @brief  Switch active transport to CAN FD.
 * @see    tf_lidar_switch_to_fdcan in tf_lidar.h
 */
bool tf_lidar_switch_to_fdcan(tf_lidar_t *lidar, FDCAN_HandleTypeDef *hfdcan)
{
    return tf_lidar_init_fdcan(lidar, lidar->model, hfdcan);
}
