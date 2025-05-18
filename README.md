````markdown
# TF-LIDAR-STM32

Modular STM32 library for TF-series LiDAR sensors over UART, CAN and FDCAN.

## Features

- **Multi-Model Support**  
  Plug-and-play support for any TF-series LiDAR by adding a model config.
- **Multiple Transports**  
  UART, classic CAN and CAN FD (FDCAN) with dynamic switching at runtime.
- **Generic API**  
  Single API surface (`tf_lidar_init_*`, `tf_lidar_read`, `tf_lidar_switch_to_*`).
- **Config-Driven**  
  Per-model parameters in `tf_models/…_config.c` — no core code changes.
- **STM32 HAL**  
  Uses STM32 HAL drivers for UART, CAN and FDCAN.

## Table of Contents

- [Getting Started](#getting-started)  
  - [Prerequisites](#prerequisites)  
  - [Installation](#installation)  
- [Usage](#usage)  
  - [Initialization](#initialization)  
  - [Reading Data](#reading-data)  
  - [Switching Transport](#switching-transport)  
  - [Adding a New Model](#adding-a-new-model)  
- [Example](#example)  
- [API Reference](#api-reference)  
- [Contributing](#contributing)  
- [License](#license)  

## Getting Started

### Prerequisites

- **Hardware**: STM32 MCU (any series with HAL UART/CAN/FDCAN), TF-series LiDAR sensor(s).  
- **Software**: STM32CubeIDE or equivalent toolchain.  
- **Libraries**: STM32 HAL drivers for UART, CAN and FDCAN.  

### Installation

1. **Clone the repository**  
   ```bash
   git clone https://github.com/yourusername/TF-LIDAR-STM32.git
   cd TF-LIDAR-STM32
````

2. **Add to your project**
   Copy the `include/` and `src/` folders into your STM32 project.
3. **Include headers**

   ```c
   #include "tf_lidar.h"
   #include "tf_models/tf350_config.c"  // or your chosen model
   ```
4. **Configure and init your transports**

   * Make sure UART, CAN and/or FDCAN peripherals are initialized (CubeMX or manual).

## Usage

### Initialization

```c
tf_lidar_t lidar;

// UART
tf_lidar_init_uart(&lidar, &tf350_model, &huart1);

// Classic CAN
tf_lidar_init_can(&lidar, &tf350_model, &hcan1);

// CAN FD
tf_lidar_init_fdcan(&lidar, &tf350_model, &hfdcan1);
```

### Reading Data

```c
float distance_m, signal;

if (tf_lidar_read(&lidar, &distance_m, &signal)) {
    printf("Distance: %.2f m, Signal: %.0f\r\n", distance_m, signal);
}
```

### Switching Transport

```c
// Switch from UART to CAN FD at runtime
tf_lidar_switch_to_fdcan(&lidar, &hfdcan1);
```

### Adding a New Model

1. Create `src/tf_models/yourmodel_config.c` with a `tf_lidar_model_t` struct:

   ```c
   #include "tf_config.h"

   const tf_lidar_model_t yourmodel_model = {
       .model_name        = "YourModel",
       .uart_baud         = 115200,
       .uart_frame_length = 9,
       .can_baud          = 1000000,
       .can_id_rx         = 0x3010,
       .can_id_tx         = 0x0103,
       .offset_distance   = 2,
       .offset_signal     = 4,
       .distance_scale    = 0.01f,
       .signal_scale      = 1.0f
   };
   ```
2. Include your config in your build (e.g. CMakeLists or Makefile).
3. Use `&yourmodel_model` when calling `tf_lidar_init_*`.

## Example

```c
#include "main.h"
#include "tf_lidar.h"
#include "tf_models/tf350_config.c"

UART_HandleTypeDef   huart1;
CAN_HandleTypeDef    hcan1;
FDCAN_HandleTypeDef  hfdcan1;
tf_lidar_t           lidar;

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_USART1_UART_Init();
    MX_CAN1_Init();
    MX_FDCAN1_Init();

    // Start on UART
    tf_lidar_init_uart(&lidar, &tf350_model, &huart1);

    while (1) {
        float dist, sig;
        if (tf_lidar_read(&lidar, &dist, &sig)) {
            printf("UART → Distance: %.2f m, Signal: %.0f\r\n", dist, sig);
        }
        HAL_Delay(200);

        // Switch to CAN FD mid-loop (example)
        tf_lidar_switch_to_fdcan(&lidar, &hfdcan1);
        if (tf_lidar_read(&lidar, &dist, &sig)) {
            printf("FDCAN → Distance: %.2f m, Signal: %.0f\r\n", dist, sig);
        }
        HAL_Delay(200);
    }
}
```

## API Reference

### Core Types

```c
typedef enum {
    TF_TRANSPORT_UART,
    TF_TRANSPORT_CAN,
    TF_TRANSPORT_FDCAN
} tf_transport_t;

typedef struct {
    const tf_lidar_model_t *model;
    tf_transport_t          transport;
    union {
        UART_HandleTypeDef  *huart;
        CAN_HandleTypeDef   *hcan;
        FDCAN_HandleTypeDef *hfdcan;
    } iface;
    uint8_t rx_buf[64];
} tf_lidar_t;
```

### Initialization & Transport

```c
bool tf_lidar_init_uart(tf_lidar_t *lidar,
                        const tf_lidar_model_t *model,
                        UART_HandleTypeDef *huart);

bool tf_lidar_init_can(tf_lidar_t *lidar,
                       const tf_lidar_model_t *model,
                       CAN_HandleTypeDef *hcan);

bool tf_lidar_init_fdcan(tf_lidar_t *lidar,
                         const tf_lidar_model_t *model,
                         FDCAN_HandleTypeDef *hfdcan);

bool tf_lidar_switch_to_uart(tf_lidar_t *lidar,
                             UART_HandleTypeDef *huart);

bool tf_lidar_switch_to_can(tf_lidar_t *lidar,
                            CAN_HandleTypeDef *hcan);

bool tf_lidar_switch_to_fdcan(tf_lidar_t *lidar,
                              FDCAN_HandleTypeDef *hfdcan);
```

### Data Acquisition

```c
/**
 * @brief  Read distance and signal strength.
 * @param  lidar      TF LIDAR handle
 * @param  distance_m Out: distance in meters
 * @param  signal     Out: signal strength
 * @retval true on success
 */
bool tf_lidar_read(tf_lidar_t *lidar, float *distance_m, float *signal);
```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/NewModel`)
3. Commit your changes (`git commit -m "Add support for NEWMODEL"`)
4. Push to your branch (`git push origin feature/NewModel`)
5. Open a Pull Request

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

```
```

## 📞 Contact
For questions or suggestions, feel free to reach out via email:
📧 Alixahedi@gmail.com

<br/>
<a href="https://twitter.com/alixahedi">
<img align="left" alt="Alix | Twitter" width="22px" src="https://github.com/alixahedi/alixahedi/blob/main/assests/img/social/Twitter.png" />
</a>
<a href="https://www.linkedin.com/in/ali-zahedi-b5a360158//">
<img align="left" alt="Ali's LinkedIN" width="22px" src="https://github.com/alixahedi/alixahedi/blob/main/assests/img/social/Linkedin.png" />
</a>
<a href="https://www.facebook.com/Alixahedi/">
<img align="left" alt="Ali's FaceBook" width="22px" src="https://github.com/alixahedi/alixahedi/blob/main/assests/img/social/fb.png" />
</a>
<a href="https://www.instagram.com/Alixahedi">
<img align="left" alt="Ali's Instagram" width="22px" src="https://github.com/alixahedi/alixahedi/blob/main/assests/img/social/insta.png" />
</a>
<br/>
