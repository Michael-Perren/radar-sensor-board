#include "spiglobalmap.h"
#ifndef SPI_CB_MAX
#define SPI_CB_MAX 16  /* tune for your app */
#endif

typedef struct {
  SPI_HandleTypeDef *hspi;
  spi_cb_type_t      type;
  spi_user_cb_t      cb;
  void              *user_ctx;
} spi_cb_entry_t;

static spi_cb_entry_t g_tbl[SPI_CB_MAX];
static size_t         g_cnt;

/* Register one listener for (handle,event) */
bool spi_cb_register(SPI_HandleTypeDef *hspi, spi_cb_type_t type,
                     spi_user_cb_t cb, void *user_ctx)
{
  /* Optional: protect if called at runtime; for init-time calls you can skip */
  // uint32_t primask = __get_PRIMASK(); __disable_irq();
  for (size_t i = 0; i < g_cnt; ++i) {
    if (g_tbl[i].hspi == hspi && g_tbl[i].type == type &&
        g_tbl[i].cb   == cb) {
      g_tbl[i].user_ctx = user_ctx; /* update */
      // if (!primask) __enable_irq();
      return true;
    }
  }
  if (g_cnt >= SPI_CB_MAX) { /* overflow */ /* if (!primask) __enable_irq(); */ return false; }
  g_tbl[g_cnt++] = (spi_cb_entry_t){ .hspi = hspi, .type = type, .cb = cb, .user_ctx = user_ctx };
  // if (!primask) __enable_irq();
  return true;
}

bool spi_cb_unregister(SPI_HandleTypeDef *hspi, spi_cb_type_t type,
                       spi_user_cb_t cb, void *user_ctx)
{
  // uint32_t primask = __get_PRIMASK(); __disable_irq();
  for (size_t i = 0; i < g_cnt; ++i) {
    if (g_tbl[i].hspi == hspi && g_tbl[i].type == type &&
        g_tbl[i].cb   == cb && g_tbl[i].user_ctx == user_ctx) {
      g_tbl[i] = g_tbl[g_cnt - 1]; /* compact by swap-with-last */
      --g_cnt;
      // if (!primask) __enable_irq();
      return true;
    }
  }
  // if (!primask) __enable_irq();
  return false;
}

/* Called from HAL ISR/weak callback; MUST be ISR-safe */
void spi_cb_dispatch(SPI_HandleTypeDef *hspi, spi_cb_type_t type)
{
  /* No allocation, no blocking; just iterate & invoke */
  for (size_t i = 0; i < g_cnt; ++i) {
    if (g_tbl[i].hspi == hspi && g_tbl[i].type == type) {
      g_tbl[i].cb(hspi, g_tbl[i].user_ctx);
    }
  }
}