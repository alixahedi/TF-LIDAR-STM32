// include/tf_config.h
#ifndef TF_CONFIG_H
#define TF_CONFIG_H

#include <stdint.h>

#if __has_include("stm32f1xx_hal.h")
#  include "stm32f1xx_hal.h"
#elif __has_include("stm32f4xx_hal.h")
#  include "stm32f4xx_hal.h"
#elif __has_include("stm32g4xx_hal.h")
#  include "stm32g4xx_hal.h"
#endif

/**
 * @brief  Model-specific parameters for a TF-series LiDAR.
 */
typedef struct {
    const char    *model_name;        /**< Name of the model */
    uint32_t       uart_baud;         /**< UART baud rate */
    uint8_t        uart_frame_length; /**< Number of bytes per UART frame */
    uint32_t       can_baud;          /**< CAN nominal baud rate */
    uint32_t       can_id_rx;         /**< CAN standard ID to receive data */
    uint32_t       can_id_tx;         /**< CAN standard ID to send commands */
    uint8_t        offset_distance;   /**< Byte offset of distance in frame */
    uint8_t        offset_signal;     /**< Byte offset of signal strength in frame */
    float          distance_scale;    /**< Scale factor for raw distance */
    float          signal_scale;      /**< Scale factor for raw signal */
} tf_lidar_model_t;

#endif /* TF_CONFIG_H */
