// include/tf_lidar.h
#ifndef TF_LIDAR_H
#define TF_LIDAR_H

#include "tf_config.h"

#if __has_include("stm32f1xx_hal_uart.h")
#  include "stm32f1xx_hal_uart.h"
#elif __has_include("stm32f4xx_hal_uart.h")
#  include "stm32f4xx_hal_uart.h"
#endif

#if __has_include("stm32f1xx_hal_can.h")
#  include "stm32f1xx_hal_can.h"
#elif __has_include("stm32f4xx_hal_can.h")
#  include "stm32f4xx_hal_can.h"
#endif

#if __has_include("stm32f1xx_hal_fdcan.h")
#  include "stm32f1xx_hal_fdcan.h"
#elif __has_include("stm32g4xx_hal_fdcan.h")
#  include "stm32g4xx_hal_fdcan.h"
#endif

/**
 * @brief  Available transports for TF-series LiDAR.
 */
typedef enum {
    TF_TRANSPORT_UART = 0, /**< Use UART interface */
    TF_TRANSPORT_CAN,      /**< Use classic CAN interface */
    TF_TRANSPORT_FDCAN     /**< Use CAN FD interface */
} tf_transport_t;

/**
 * @brief  Handle for a TF-series LiDAR instance.
 */
typedef struct {
    const tf_lidar_model_t *model;    /**< Pointer to model parameters */
    tf_transport_t          transport;/**< Active transport */
    union {
        UART_HandleTypeDef  *huart;   /**< UART handle */
        CAN_HandleTypeDef   *hcan;    /**< CAN handle */
        FDCAN_HandleTypeDef *hfdcan;  /**< FDCAN handle */
    } iface;
    uint8_t rx_buf[64];               /**< Reception buffer */
} tf_lidar_t;

/**
 * @brief  Initialize TF LIDAR over UART.
 * @param  lidar: Pointer to TF LIDAR handle
 * @param  model: Pointer to model configuration
 * @param  huart: UART handle, pointer to initialized UART instance
 * @retval true on success, false on failure
 */
bool tf_lidar_init_uart(tf_lidar_t *lidar,
                        const tf_lidar_model_t *model,
                        UART_HandleTypeDef *huart);

/**
 * @brief  Initialize TF LIDAR over classic CAN.
 * @param  lidar: Pointer to TF LIDAR handle
 * @param  model: Pointer to model configuration
 * @param  hcan:  CAN handle, pointer to initialized CAN instance
 * @retval true on success, false on failure
 */
bool tf_lidar_init_can(tf_lidar_t *lidar,
                       const tf_lidar_model_t *model,
                       CAN_HandleTypeDef *hcan);

/**
 * @brief  Initialize TF LIDAR over CAN FD.
 * @param  lidar:  Pointer to TF LIDAR handle
 * @param  model:  Pointer to model configuration
 * @param  hfdcan: FDCAN handle, pointer to initialized FDCAN instance
 * @retval true on success, false on failure
 */
bool tf_lidar_init_fdcan(tf_lidar_t *lidar,
                         const tf_lidar_model_t *model,
                         FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief  Read a single measurement from the sensor.
 * @param  lidar:      Pointer to TF LIDAR handle
 * @param  distance_m: Pointer to float to receive distance in meters
 * @param  signal:     Pointer to float to receive signal strength
 * @retval true on success, false on failure or timeout
 */
bool tf_lidar_read(tf_lidar_t *lidar, float *distance_m, float *signal);

/**
 * @brief  Switch active transport to UART.
 * @param  lidar: Pointer to TF LIDAR handle
 * @param  huart: UART handle
 * @retval true on success, false on failure
 */
bool tf_lidar_switch_to_uart(tf_lidar_t *lidar, UART_HandleTypeDef *huart);

/**
 * @brief  Switch active transport to classic CAN.
 * @param  lidar: Pointer to TF LIDAR handle
 * @param  hcan:  CAN handle
 * @retval true on success, false on failure
 */
bool tf_lidar_switch_to_can(tf_lidar_t *lidar, CAN_HandleTypeDef *hcan);

/**
 * @brief  Switch active transport to CAN FD.
 * @param  lidar:  Pointer to TF LIDAR handle
 * @param  hfdcan: FDCAN handle
 * @retval true on success, false on failure
 */
bool tf_lidar_switch_to_fdcan(tf_lidar_t *lidar, FDCAN_HandleTypeDef *hfdcan);

#endif /* TF_LIDAR_H */
