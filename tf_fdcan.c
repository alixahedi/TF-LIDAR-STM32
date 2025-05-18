// src/tf_fdcan.c
#include "tf_lidar.h"
#include <string.h>

/**
 * @brief  Initialize CAN FD transport.
 * @see    tf_lidar_init_fdcan in tf_lidar.h
 */
bool tf_lidar_init_fdcan(tf_lidar_t *lidar,
                         const tf_lidar_model_t *model,
                         FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};

    lidar->model       = model;
    lidar->transport   = TF_TRANSPORT_FDCAN;
    lidar->iface.hfdcan = hfdcan;

    if (HAL_FDCAN_Init(hfdcan) != HAL_OK)
        return false;

    // Configure standard ID filter for TX ID
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = model->can_id_tx;
    filter.FilterID2    = 0x7FF;
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
        return false;

    HAL_FDCAN_Start(hfdcan);
    HAL_FDCAN_ActivateNotification(hfdcan,
                                   FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    return true;
}

/**
 * @brief  Read a measurement via CAN FD.
 * @see    tf_lidar_read in tf_lidar.h
 */
bool tf_lidar_read_fdcan(tf_lidar_t *lidar, float *distance_m, float *signal)
{
    FDCAN_RxHeaderTypeDef hdr;
    uint8_t data[64];

    if (HAL_FDCAN_GetRxMessage(lidar->iface.hfdcan,
                               FDCAN_RX_FIFO0, &hdr, data) != HAL_OK)
        return false;
    // hdr.StdId is already filtered by hardware

    uint16_t raw_d = ((uint16_t)data[lidar->model->offset_distance] << 8)
                   | data[lidar->model->offset_distance + 1];
    uint16_t raw_s = ((uint16_t)data[lidar->model->offset_signal] << 8)
                   | data[lidar->model->offset_signal + 1];

    *distance_m = raw_d * lidar->model->distance_scale;
    *signal     = raw_s * lidar->model->signal_scale;
    return true;
}
