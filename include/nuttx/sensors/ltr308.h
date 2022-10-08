#ifndef __INCLUDE_NUTTX_SENSORS_LTR308_H
#define __INCLUDE_NUTTX_SENSORS_LTR308_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#include <stdbool.h>

/* Configuration ************************************************************
 * Prerequisites:
 *
 * CONFIG_I2C
 *   Enables support for I2C drivers
 * CONFIG_SENSORS_LTR308
 *   Enables support for the LTR308 driver
 */

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LTR308)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_PARAMS_SET    2
#define MAX_PARAMS_GET    3

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

struct ltr308_ctrl_s
{
  bool reset;
  bool enable;
};

struct ltr308_measrate_s
{
  uint8_t integration_time;
  uint8_t measurement_rate;
};

struct ltr308_status_s
{
  bool power_on;
  bool interrupt_pending;
  bool data_pending;
};

struct ltr308_threshold_s
{
  uint32_t upper_limit;
  uint32_t lower_limit;
};

struct ltr308_lux_s
{
  double lux;
  uint32_t ch;
  uint8_t gain;
  uint8_t integration_time;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ltr308_register
 *
 * Description:
 *   Register the LTR308 character device as 'devpath'.
 *
 * Input Parameters:
 *   devno   - Number of device (i.e. ltr0, ltr1, ...)
 *   i2c     - An I2C driver instance.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ltr308_register(const char *devpath, FAR struct i2c_master_s *i2c);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_LTR308 */
#endif /* __INCLUDE_NUTTX_SENSORS_LTR308_H */
