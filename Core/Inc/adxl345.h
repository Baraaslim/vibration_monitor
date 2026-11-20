#ifndef ADXL345_H
#define ADXL345_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Driver handle: tells the driver which SPI bus and CS pin to use */
typedef struct
{
  SPI_HandleTypeDef *hspi;      /* SPI bus, e.g. &hspi2        */
  GPIO_TypeDef      *cs_port;   /* CS port, e.g. GPIOB         */
  uint16_t           cs_pin;    /* CS pin,  e.g. GPIO_PIN_12   */
} adxl345_t;

/* Public API */
bool    adxl345_init(adxl345_t *dev);
uint8_t adxl345_read_reg(adxl345_t *dev, uint8_t reg);
void    adxl345_write_reg(adxl345_t *dev, uint8_t reg, uint8_t value);
void    adxl345_read_xyz(adxl345_t *dev, int16_t *x, int16_t *y, int16_t *z);

#endif /* ADXL345_H */