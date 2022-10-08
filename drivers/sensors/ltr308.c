/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/ltr308.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LTR308)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LTR308_ADDR           0x53
#define LTR308_FREQ           CONFIG_LTR308_I2C_FREQUENCY
#define DEVID                 0xB1

#define LTR308_CTRL           0x00
#define LTR308_MEAS_RATE      0x04
#define LTR308_ALS_GAIN       0x05
#define LTR308_PART_ID        0x06
#define LTR308_STATUS         0x07
#define LTR308_DATA_0         0x0D
#define LTR308_DATA_1         0x0E
#define LTR308_DATA_2         0x0F
#define LTR308_INTERRUPT      0x19
#define LTR308_INTR_PERS      0x1A
#define LTR308_THRES_UP_0     0x21
#define LTR308_THRES_UP_1     0x22
#define LTR308_THRES_UP_2     0x23
#define LTR308_THRES_LOW_0    0x24
#define LTR308_THRES_LOW_1    0x25
#define LTR308_THRES_LOW_2    0x26

#define BYTE_TO_BITS          8

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct ltr308_dev_s
{
  FAR struct i2c_master_s *i2c;  /* I2C interface */
  uint8_t addr;                  /* I2C address */
  int freq;                      /* I2C bus frequency */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  bool unlinked;                 /* True, driver has been unlinked */
  int16_t crefs;                 /* Number of open references */
#endif
  sem_t devsem;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int ltr308_open(FAR struct file *filep);
static int ltr308_close(FAR struct file *filep);
#endif
static ssize_t ltr308_read(FAR struct file *filep, FAR char *buffer,
                           size_t len);
static ssize_t ltr308_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int ltr308_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
****************************************************************************/

static const struct file_operations g_ltr308fops =
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  .open = ltr308_open,
  .close = ltr308_close,
#endif
  .read = ltr308_read,
  .ioctl = ltr308_ioctl,
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  .unlink = ltr308_unlink,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ltr308_set_reg8
 *
 * Description:
 *   Write to an 8-bit LTR308 register
 *
 ****************************************************************************/

static int ltr308_set_reg8(FAR struct ltr308_dev_s *priv, uint8_t regaddr,
                           uint8_t regval)
{
  struct i2c_msg_s msg;
  uint8_t txbuffer[2];
  int ret;

  txbuffer[0]   = regaddr;
  txbuffer[1]   = regval;

  msg.frequency = priv->freq;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = txbuffer;
  msg.length    = 2;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed\n");
    }

  return ret;
}

/****************************************************************************
 * Name: ltr308_get_reg8
 *
 * Description:
 *   Read from an 8-bit LTR308 register
 *
 ****************************************************************************/

static int ltr308_get_reg8(FAR struct ltr308_dev_s *priv, uint8_t regaddr,
                           FAR uint8_t *regval)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = regval;
  msg[1].length    = 1;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ltr308_set_reg24
 *
 * Description:
 *   Write to 3 8-bit LTR308 registers
 *
 ****************************************************************************/

static int ltr308_set_reg24(FAR struct ltr308_dev_s *priv, uint8_t regaddr,
                            uint32_t val)
{
  int i, ret;

  for (i = 0; i < 3; i++, regaddr++)
    {
      ret = ltr308_set_reg8(priv, regaddr, val);
      if (ret < 0)
        {
          return ret;
        }

      val >>= BYTE_TO_BITS;
    }

  return ret;
}

/****************************************************************************
 * Name: ltr308_get_reg24
 *
 * Description:
 *   Read from 3 8-bit LTR308 registers
 *
 ****************************************************************************/

static int ltr308_get_reg24(FAR struct ltr308_dev_s *priv, uint8_t regaddr,
                            FAR uint32_t *val)
{
  int i, ret;

  *val = 0;
  for (i = 0; i < 3; i++, regaddr++)
    {
      ret = ltr308_get_reg8(priv, regaddr, ((uint8_t *)val) + i);
      if (ret < 0)
        {
          return ret;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: ltr308_set_ctrl
 *
 * Description:
 *   Power up, shut down or reset LTR308
 *
 ****************************************************************************/

static inline int ltr308_set_ctrl(FAR struct ltr308_dev_s *priv,
                                  bool reset, bool enable)
{
  return ltr308_set_reg8(priv, LTR308_CTRL, (reset << 4) | (enable << 1));
}

/****************************************************************************
 * Name: ltr308_get_ctrl
 *
 ****************************************************************************/

static int ltr308_get_ctrl(FAR struct ltr308_dev_s *priv, FAR bool *reset,
                           FAR bool *enable)
{
  uint8_t ctrl;
  int ret;

  ret = ltr308_get_reg8(priv, LTR308_CTRL, &ctrl);
  if (ret < 0)
    {
      return ret;
    }

  *reset = ctrl >> 4;
  *enable = ctrl & 0x02;

  return ret;
}

/****************************************************************************
 * Name: ltr308_set_measrate
 *
 * Description:
 *   Set the integration time and measurement rate of LTR308
 *   Integration time is the measurement time for each ALS cycle
 *   Measurement rate is the interval between DATA_REGISTERS update
 *   Measurement rate must be set to be equal or greater than integration time
 *
 ****************************************************************************/

static int ltr308_set_measrate(FAR struct ltr308_dev_s *priv,
                               uint8_t integration_time,
                               uint8_t measurement_rate)
{
  uint8_t measurement = 0x00;

  /* Sanity checks */

  if (integration_time >= 5)
    {
      integration_time = 0;
    }
  if (measurement_rate >= 6 || measurement_rate == 4)
    {
      measurement_rate = 0;
    }

  measurement |= integration_time << 4;
  measurement |= measurement_rate;

  return ltr308_set_reg8(priv, LTR308_MEAS_RATE, measurement);
}

/****************************************************************************
 * Name: ltr308_get_measrate
 *
 ****************************************************************************/

static int ltr308_get_measrate(FAR struct ltr308_dev_s *priv,
                               FAR uint8_t *integration_time,
                               FAR uint8_t *measurement_rate)
{
  uint8_t measurement;
  int ret;

  ret = ltr308_get_reg8(priv, LTR308_MEAS_RATE, &measurement);
  if (ret < 0)
    {
      return ret;
    }

  *integration_time = (measurement & 0x70) >> 4;
  *measurement_rate = measurement & 0x07;

  return ret;
}

/****************************************************************************
 * Name: ltr308_set_gain
 *
 * Description:
 *   Set the gain of LTR308
 *
 ****************************************************************************/

static int ltr308_set_gain(FAR struct ltr308_dev_s *priv, uint8_t gain)
{
  /* sanity check */

  if (gain >= 5)
    {
      gain = 0;
    }

  return ltr308_set_reg8(priv, LTR308_ALS_GAIN, gain);
}

/****************************************************************************
 * Name: ltr308_get_gain
 *
 ****************************************************************************/

static inline int ltr308_get_gain(FAR struct ltr308_dev_s *priv,
                                  FAR uint8_t *gain)
{
  return ltr308_get_reg8(priv, LTR308_ALS_GAIN, gain);
}

/****************************************************************************
 * Name: ltr308_checkid
 *
 * Description:
 *   Read and verify the LTR308 chip ID
 *
 ****************************************************************************/

static int ltr308_checkid(FAR struct ltr308_dev_s *priv)
{
  uint8_t devid;
  int ret;

  /* Read device ID */

  ret = ltr308_get_reg8(priv, LTR308_PART_ID, &devid);
  if (ret < 0)
    {
      return ret;
    }
  sninfo("devid: 0x%02x\n", devid);

  if (devid != (uint8_t) DEVID)
      return -ENODEV;

  return ret;
}

/****************************************************************************
 * Name: ltr308_get_status
 *
 * Description:
 *   Get the status information of LTR308
 *
 ****************************************************************************/

static int ltr308_get_status(FAR struct ltr308_dev_s *priv,
                             FAR bool *power_on,
                             FAR bool *interrupt_pending,
                             FAR bool *data_pending)
{
  uint8_t status;
  uint8_t ret;

  ret = ltr308_get_reg8(priv, LTR308_STATUS, &status);
  if (ret < 0)
    {
      return ret;
    }

  *power_on = status & 0x20;
  *interrupt_pending = status & 0x10;
  *data_pending = status & 0x08;

  return ret;
}

/****************************************************************************
 * Name: ltr308_get_data
 *
 * Description:
 *   Get the ALS channel data of LTR308
 *
 ****************************************************************************/

static inline int ltr308_get_data(FAR struct ltr308_dev_s *priv,
                                  FAR uint32_t *data)
{
  return ltr308_get_reg24(priv, LTR308_DATA_0, data);
}

/****************************************************************************
 * Name: ltr308_set_intrmode
 *
 * Description:
 *   Set up interrupt operations
 *
 ****************************************************************************/

static inline int ltr308_set_intrmode(FAR struct ltr308_dev_s *priv,
                                      bool on)
{
  return ltr308_set_reg8(priv, LTR308_INTERRUPT, 0x10 | (on << 2));
}

/****************************************************************************
 * Name: ltr308_get_intrmode
 *
 ****************************************************************************/

static int ltr308_get_intrmode(FAR struct ltr308_dev_s *priv,
                               FAR bool *on)
{
  uint8_t interrupt_control;
  int ret;

  ret = ltr308_get_reg8(priv, LTR308_INTERRUPT, &interrupt_control);
  if (ret < 0)
    {
      return ret;
    }

  *on = interrupt_control & 0x04;

  return ret;
}

/****************************************************************************
 * Name: ltr308_set_intrpers
 *
 * Description:
 *   Set up interrupt persistance (controls the N number of times the
 *   measurement data is outside the range defined by upper and lower
 *   threshold)
 *
 ****************************************************************************/

static int ltr308_set_intrpers(FAR struct ltr308_dev_s *priv,
                               uint8_t intr_pers)
{
  /* Sanity check */

  if (intr_pers >= 16)
    {
      intr_pers = 0;
    }

  return ltr308_set_reg8(priv, LTR308_INTR_PERS, intr_pers << 4);
}

/****************************************************************************
 * Name: ltr308_get_intrpers
 *
 ****************************************************************************/

static int ltr308_get_intrpers(FAR struct ltr308_dev_s *priv,
                               FAR uint8_t *intr_pers)
{
  int ret;

  ret = ltr308_get_reg8(priv, LTR308_INTR_PERS, intr_pers);
  if (ret < 0)
    {
      return ret;
    }

  *intr_pers >>= 4;

  return ret;
}

/****************************************************************************
 * Name: ltr308_set_threshold
 *
 * Description:
 *   Set the upper and lower limits of the threshold
 *
 ****************************************************************************/

static inline int ltr308_set_threshold(FAR struct ltr308_dev_s *priv,
                                       uint32_t upper_limit,
                                       uint32_t lower_limit)
{
  return ltr308_set_reg24(priv, LTR308_THRES_UP_0, upper_limit) |
         ltr308_set_reg24(priv, LTR308_THRES_LOW_0, lower_limit);
}

/****************************************************************************
 * Name: ltr308_get_threshold
 *
 ****************************************************************************/

static inline int ltr308_get_threshold(FAR struct ltr308_dev_s *priv,
                                       FAR uint32_t *upper_limit,
                                       FAR uint32_t *lower_limit)
{
  return ltr308_get_reg24(priv, LTR308_THRES_UP_0, upper_limit) |
         ltr308_get_reg24(priv, LTR308_THRES_LOW_0, lower_limit);
}

/****************************************************************************
 * Name: ltr308_get_lux
 *   Convert raw data to lux
 *
 ****************************************************************************/

static int ltr308_get_lux(FAR struct ltr308_dev_s *priv, uint8_t gain,
                          uint8_t integration_time, uint32_t ch,
                          FAR double *lux)
{
  double d0;

  /*
   * Determine if sensor is saturated. If so, abandon ship (calculation will
   * not be accurate)
   */
  if (ch == 0x000FFFFF)
    {
      *lux = 0.0;
      return -ENODATA;
    }

  d0 = ((double)ch * 0.06);
  switch (gain)
    {
      case 0:
        break;

      case 1:
        d0 /= 3;
        break;

      case 2:
        d0 /= 6;
        break;

      case 3:
        d0 /= 9;
        break;

      case 4:
        d0 /= 18;
        break;

      default:
        d0 = 0.0;
        break;
    }

  switch(integration_time)
    {
      case 0:
        *lux = d0 / 4;
        break;

      case 1:
        *lux = d0 / 2;
        break;

      case 2:
        *lux = d0;
        break;

      case 3:
        *lux = d0 * 2;
        break;

      case 4:
        *lux = d0 * 4;
        break;

      default:
        *lux = 0.0;
        break;
    }

  return 0;
}

/****************************************************************************
 * Name: ltr308_open
 *
 * Description:
 *   Character driver open method. This gets called when an LTR308 device
 *   is opened. Increments the number of references to the LTR308 device.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int ltr308_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ltr308_dev_s *priv  = inode->i_private;
  int ret;

  /* Get exclusive access */

  ret = nxsem_wait_uninterruptible(&priv->devsem);
  if (ret < 0)
    {
      snerr("nxsem_wait_uninterruptible (err = %d)\n", ret);
      return ret;
    }

  /* Increment the count of open references on the driver */

  priv->crefs++;
  DEBUGASSERT(priv->crefs > 0);

  nxsem_post(&priv->devsem);
  return OK;
}
#endif

/****************************************************************************
 * Name: ltr308_close
 *
 * Description:
 *   The routine is called when an LTR308 device is closed. Decrements the
 *   number of references to the LTR308 device and frees the memory if 0 is
 *   reached.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int ltr308_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ltr308_dev_s *priv  = inode->i_private;
  int ret;

  /* Get exclusive access */

  ret = nxsem_wait_uninterruptible(&priv->devsem);
  if (ret < 0)
    {
      snerr("nxsem_wait_uninterruptible(err = %d)\n", ret);
      return ret;
    }

  /* Decrement the count of open references on the driver */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

  /* If the count has decremented to zero and the driver has been unlinked,
   * then free memory now.
   */

  if (priv->crefs <= 0 && priv->unlinked)
    {
      nxsem_destroy(&priv->devsem);
      kmm_free(priv);
      return OK;
    }

  nxsem_post(&priv->devsem);
  return OK;
}
#endif

/****************************************************************************
 * Name: ltr308_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t ltr308_read(FAR struct file *filep, FAR char *buffer,
                           size_t len)
{
  /* Use ioctl to read from sensor */

  return -ENOSYS;
}

/****************************************************************************
 * Name: ltr308_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static ssize_t ltr308_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ltr308_dev_s *priv = inode->i_private;
  int ret;

  /* Get exclusive access */

  ret = nxsem_wait_uninterruptible(&priv->devsem);
  if (ret < 0)
    {
      snerr("nxsem_wait_uninterruptible(err = %d)\n", ret);
      return ret;
    }

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (priv->unlinked)
    {
      /* Sensor is unlinked. Do not allow any operations */

      nxsem_post(&priv->devsem);
      return -ENODEV;
    }
#endif

  switch (cmd)
    {
      case SNIOC_SET_CTRL:
        {
          FAR struct ltr308_ctrl_s *data = (FAR struct ltr308_ctrl_s *)arg;
          ret = ltr308_set_ctrl(priv, data->reset, data->enable);
        }
        break;

      case SNIOC_GET_CTRL:
        {
          FAR struct ltr308_ctrl_s *data = (FAR struct ltr308_ctrl_s *)arg;
          ret = ltr308_get_ctrl(priv, &data->reset, &data->enable);
        }
        break;

      case SNIOC_SET_MEASRATE:
        {
          FAR struct ltr308_measrate_s *data =
            (FAR struct ltr308_measrate_s *)arg;
          ret = ltr308_set_measrate(priv, data->integration_time,
                                    data->measurement_rate);
        }
        break;

      case SNIOC_GET_MEASRATE:
        {
          FAR struct ltr308_measrate_s *data =
            (FAR struct ltr308_measrate_s *)arg;
          ret = ltr308_get_measrate(priv, &data->integration_time,
                                    &data->measurement_rate);
        }
        break;

      case SNIOC_SET_GAIN:
        {
          uint8_t gain = arg;
          ret = ltr308_set_gain(priv, gain);
        }
        break;

      case SNIOC_GET_GAIN:
        {
          FAR uint8_t *gain = (FAR uint8_t *)arg;
          ret = ltr308_get_gain(priv, gain);
        }
        break;

      case SNIOC_GET_STATUS:
        {
          FAR struct ltr308_status_s *data = (FAR struct ltr308_status_s *)arg;
          ret = ltr308_get_status(priv, &data->power_on,
                                  &data->interrupt_pending,
                                  &data->data_pending);
        }
        break;

      case SNIOC_GET_DATA:
        {
          FAR uint32_t *data = (FAR uint32_t *)arg;
          ret = ltr308_get_data(priv, data);
        }
        break;

      case SNIOC_SET_INTRMODE:
        {
          /* Currently not supported. To be addressed in separate patch. */

          return -EOPNOTSUPP;
        }
        break;

      case SNIOC_GET_INTRMODE:
        {
          FAR bool *on = (FAR bool *)arg;
          ret = ltr308_get_intrmode(priv, on);
        }
        break;

      case SNIOC_SET_INTRPERS:
        {
          uint8_t data = arg;
          ret = ltr308_set_intrpers(priv, data);
        }
        break;

      case SNIOC_GET_INTRPERS:
        {
          FAR uint8_t *data = (FAR uint8_t *)arg;
          ret = ltr308_get_intrpers(priv, data);
        }
        break;

      case SNIOC_SET_THRESHOLD:
        {
          FAR struct ltr308_threshold_s *data =
            (FAR struct ltr308_threshold_s *)arg;
          ret = ltr308_set_threshold(priv, data->upper_limit,
                                     data->lower_limit);
        }
        break;

      case SNIOC_GET_THRESHOLD:
        {
          FAR struct ltr308_threshold_s *data =
            (FAR struct ltr308_threshold_s *)arg;
          ret = ltr308_get_threshold(priv, &data->upper_limit,
                                     &data->lower_limit);
        }
        break;

      case SNIOC_GET_LUX:
        {
          FAR struct ltr308_lux_s *data = (FAR struct ltr308_lux_s *)arg;
          ret = ltr308_get_lux(priv, data->gain, data->integration_time,
                               data->ch, &data->lux);
        }
        break;

      default:
        {
          snerr("Unrecognized cmd: %d\n", cmd);
          ret = -ENOTTY;
        }
        break;
    }

  nxsem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: ltr308_unlink
 *
 * Description:
 *   Standard character driver unlink method.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int ltr308_unlink(FAR struct inode *inode)
{
  FAR struct ltr308_dev_s *priv;
  int ret;

  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv = (FAR struct ltr308_dev_s *)inode->i_private;

  /* Get exclusive access */

  ret = nxsem_wait_uninterruptible(&priv->devsem);
  if (ret < 0)
    {
      snerr("nxsem_wait_uninterruptible (err = %d)\n", ret);
      return ret;
    }

  /* Check if there any open references to the driver */

  if (priv->crefs <= 0)
    {
      nxsem_destroy(&priv->devsem);
      kmm_free(priv);
      return OK;
    }

  /*
   * Mark the driver as unlinked and free the resources when the last client
   * closes their reference to the driver.
   */

  priv->unlinked = true;
  nxsem_post(&priv->devsem);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ltr308_register
 *
 * Description:
 *   Register the LTR308 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/light0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             LTR308
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ltr308_register(const char *devpath, FAR struct i2c_master_s *i2c)
{
  FAR struct ltr308_dev_s *priv;
  int ret = 0;

  DEBUGASSERT(i2c != NULL);

  /* Initialize the LTR308 device structure */

  priv = (FAR struct ltr308_dev_s *)kmm_zalloc(sizeof(struct ltr308_dev_s));
  if (!priv)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = LTR308_ADDR;
  priv->freq = LTR308_FREQ;
  nxsem_init(&priv->devsem, 0, 1);

  /* Check Device ID */

  ret = ltr308_checkid(priv);
  if (ret < 0)
    {
      snerr("ERROR: Wrong device ID!\n");
      kmm_free(priv);
      return ret;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_ltr308fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  sninfo("LTR308 driver loaded successfully!\n");
  return OK;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_LTR308 */
