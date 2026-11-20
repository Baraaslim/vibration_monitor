#include "adxl345.h"

/* ---- Register addresses (datasheet page 13) ---- */
#define ADXL345_REG_DEVID        0x00
#define ADXL345_REG_BW_RATE      0x2C
#define ADXL345_REG_POWER_CTL    0x2D
#define ADXL345_REG_DATA_FORMAT  0x31
#define ADXL345_REG_DATAX0       0x32

/* ---- Command byte bits (page 9) ---- */
#define ADXL345_READ             0x80   /* bit 7: R/W = 1 */
#define ADXL345_MULTIBYTE        0x40   /* bit 6: MB  = 1 */

/* ---- Values ---- */
#define ADXL345_DEVID_VALUE      0xE5   /* page 14 */
#define ADXL345_RATE_3200HZ      0x0F   /* page 7, Table 5 */
#define ADXL345_FORMAT_FULLRES_16G 0x0B /* page 17: FULL_RES, +-16 g, 4-wire, right-justified */
#define ADXL345_MEASURE          0x08   /* page 16: MEASURE bit */

/* ---- Private helpers ---- */
static void cs_low(adxl345_t *dev)
{
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void cs_high(adxl345_t *dev)
{
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

/* ---- Public API ---- */
uint8_t adxl345_read_reg(adxl345_t *dev, uint8_t reg)
{
  uint8_t tx[2] = { ADXL345_READ | reg, 0x00 };
  uint8_t rx[2] = { 0 };

  cs_low(dev);
  HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2, HAL_MAX_DELAY);
  cs_high(dev);

  return rx[1];
}

void adxl345_write_reg(adxl345_t *dev, uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = { reg, value };

  cs_low(dev);
  HAL_SPI_Transmit(dev->hspi, tx, 2, HAL_MAX_DELAY);
  cs_high(dev);
}

bool adxl345_init(adxl345_t *dev)
{
  if (adxl345_read_reg(dev, ADXL345_REG_DEVID) != ADXL345_DEVID_VALUE)
  {
    return false;                                   /* sensor not found */
  }

  adxl345_write_reg(dev, ADXL345_REG_BW_RATE,     ADXL345_RATE_3200HZ);
  adxl345_write_reg(dev, ADXL345_REG_DATA_FORMAT, ADXL345_FORMAT_FULLRES_16G);
  adxl345_write_reg(dev, ADXL345_REG_POWER_CTL,   ADXL345_MEASURE);  /* last */
  return true;
}

void adxl345_read_xyz(adxl345_t *dev, int16_t *x, int16_t *y, int16_t *z)
{
  uint8_t tx[7] = { ADXL345_READ | ADXL345_MULTIBYTE | ADXL345_REG_DATAX0 };  /* = 0xF2 */
  uint8_t rx[7] = { 0 };

  cs_low(dev);
  HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 7, HAL_MAX_DELAY);
  cs_high(dev);

  *x = (int16_t)((rx[2] << 8) | rx[1]);
  *y = (int16_t)((rx[4] << 8) | rx[3]);
  *z = (int16_t)((rx[6] << 8) | rx[5]);
}