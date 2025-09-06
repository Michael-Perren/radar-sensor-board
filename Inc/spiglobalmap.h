#include "stm32h5xx_hal.h"
#include <stdbool.h>
#include <stddef.h>

typedef enum {
  SPI_CB_TX_COMPLETE,
  SPI_CB_RX_COMPLETE,
  SPI_CB_TXRX_COMPLETE,
  SPI_CB_ERROR,
} spi_cb_type_t;

/* ISR-safe callback signature */
typedef void (*spi_user_cb_t)(SPI_HandleTypeDef *hspi, void *user_ctx);

bool spi_cb_register (SPI_HandleTypeDef *hspi, spi_cb_type_t type,
                      spi_user_cb_t cb, void *user_ctx);
bool spi_cb_unregister(SPI_HandleTypeDef *hspi, spi_cb_type_t type,
                      spi_user_cb_t cb, void *user_ctx);
void spi_cb_dispatch  (SPI_HandleTypeDef *hspi, spi_cb_type_t type);