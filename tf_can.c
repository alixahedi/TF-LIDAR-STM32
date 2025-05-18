// src/tf_can.c
#include "tf_lidar.h"

/**
 * @brief  Initialize classic CAN transport.
 * @see    tf_lidar_init_can in tf_lidar.h
 */
bool tf_lidar_init_can(tf_lidar_t *lidar,
                       const tf_lidar_model_t *model,
                       CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filt = {0};

    lidar->model      = model;
    lidar->transport  = TF_TRANSPORT_CAN;
    lidar->iface.hcan = hcan;

    // Basic CAN init (user must call HAL_CAN_Init after setting Prescaler, etc.)
    if (HAL_CAN_Init(hcan) != HAL_OK)
        return false;

    // Configure 32-bit mask filter to accept only TX ID
    filt.FilterActivation     = ENABLE;
    filt.FilterBank           = 0;
    filt.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filt.FilterMode           = CAN_FILTERMODE_IDMASK;
    filt.FilterScale          = CAN_FILTERSCALE_32BIT;
    filt.FilterIdHigh         = (model->can_id_tx << 5) & 0xFFFF;
    filt.FilterMaskIdHigh     = 0xFFE0;
    if (HAL_CAN_ConfigFilter(hcan, &filt) != HAL_OK)
        return false;

    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    return true;
}

/**
 * @brief  Read a measurement via classic CAN.
 * @see    tf_lidar_read in tf_lidar.h
 */
bool tf_lidar_read_can(tf_lidar_t *lidar, float *distance_m, float *signal)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t data[8];

    if (HAL_CAN_GetRxMessage(lidar->iface.hcan,
                             CAN_RX_FIFO0, &hdr, data) != HAL_OK)
        return false;
    if (hdr.StdId != lidar->model->can_id_rx)
        return false;

    uint16_t raw_d = ((uint16_t)data[lidar->model->offset_distance] << 8)
                   | data[lidar->model->offset_distance + 1];
    uint16_t raw_s = ((uint16_t)data[lidar->model->offset_signal] << 8)
                   | data[lidar->model->offset_signal + 1];

    *distance_m = raw_d * lidar->model->distance_scale;
    *signal     = raw_s * lidar->model->signal_scale;
    return true;
}
