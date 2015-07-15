/*  Date: 2014.2.27
 *  Revision: 2.5
 */

/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */


/* file BMA250.c
   brief This file contains all function implementations for the BMA250 in linux

*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/delay.h>

#include <mach/system.h>
#include <mach/hardware.h>
#include <mach/sys_config.h>
#include <linux/init-input.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_PM)
#include <linux/pm.h>
#endif

static u32 debug_mask = 0;
#define dprintk(level_mask, fmt, arg...)	if (unlikely(debug_mask & level_mask)) \
	printk(KERN_DEBUG fmt , ## arg)

module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);


#define SENSOR_NAME                             "bma250"
#define GRAVITY_EARTH                           9806550
#define ABSMIN_2G                               (-GRAVITY_EARTH * 2)
#define ABSMAX_2G                               (GRAVITY_EARTH * 2)
#define SLOPE_THRESHOLD_VALUE                   32
#define SLOPE_DURATION_VALUE                    1
#define INTERRUPT_LATCH_MODE                    13
#define INTERRUPT_ENABLE                        1
#define INTERRUPT_DISABLE                       0
#define MAP_SLOPE_INTERRUPT                     2
#define SLOPE_X_INDEX                           5
#define SLOPE_Y_INDEX                           6
#define SLOPE_Z_INDEX                           7
#define BMA250_MAX_DELAY                        200

enum bma_chip_id
{
	BMA_CHIP_ID_150  = 0x02,
	BMA_CHIP_ID_250  = 0x03,
	BMA_CHIP_ID_250E = 0xF9,
	BMA_CHIP_ID_255  = 0xFA,
	BMA_CHIP_ID_280  = 0xFB,
};

#define BMA250_MODE_SET                         BMA250_MODE_NORMAL
#define BMA250_RANGE_SET                        BMA250_RANGE_2G
#define BMA250_BW_SET                           BMA250_BW_125HZ


/*
 *
 *      register definitions
 *
 */

#define BMA250_CHIP_ID_REG                      0x00
#define BMA250_VERSION_REG                      0x01
#define BMA250_X_AXIS_LSB_REG                   0x02
#define BMA250_X_AXIS_MSB_REG                   0x03
#define BMA250_Y_AXIS_LSB_REG                   0x04
#define BMA250_Y_AXIS_MSB_REG                   0x05
#define BMA250_Z_AXIS_LSB_REG                   0x06
#define BMA250_Z_AXIS_MSB_REG                   0x07
#define BMA250_TEMP_RD_REG                      0x08
#define BMA250_STATUS1_REG                      0x09
#define BMA250_STATUS2_REG                      0x0A
#define BMA250_STATUS_TAP_SLOPE_REG             0x0B
#define BMA250_STATUS_ORIENT_HIGH_REG           0x0C
#define BMA250_RANGE_SEL_REG                    0x0F
#define BMA250_BW_SEL_REG                       0x10
#define BMA250_MODE_CTRL_REG                    0x11
#define BMA250_LOW_NOISE_CTRL_REG               0x12
#define BMA250_DATA_CTRL_REG                    0x13
#define BMA250_RESET_REG                        0x14
#define BMA250_INT_ENABLE1_REG                  0x16
#define BMA250_INT_ENABLE2_REG                  0x17
#define BMA250_INT1_PAD_SEL_REG                 0x19
#define BMA250_INT_DATA_SEL_REG                 0x1A
#define BMA250_INT2_PAD_SEL_REG                 0x1B
#define BMA250_INT_SRC_REG                      0x1E
#define BMA250_INT_SET_REG                      0x20
#define BMA250_INT_CTRL_REG                     0x21
#define BMA250_LOW_DURN_REG                     0x22
#define BMA250_LOW_THRES_REG                    0x23
#define BMA250_LOW_HIGH_HYST_REG                0x24
#define BMA250_HIGH_DURN_REG                    0x25
#define BMA250_HIGH_THRES_REG                   0x26
#define BMA250_SLOPE_DURN_REG                   0x27
#define BMA250_SLOPE_THRES_REG                  0x28
#define BMA250_TAP_PARAM_REG                    0x2A
#define BMA250_TAP_THRES_REG                    0x2B
#define BMA250_ORIENT_PARAM_REG                 0x2C
#define BMA250_THETA_BLOCK_REG                  0x2D
#define BMA250_THETA_FLAT_REG                   0x2E
#define BMA250_FLAT_HOLD_TIME_REG               0x2F
#define BMA250_STATUS_LOW_POWER_REG             0x31
#define BMA250_SELF_TEST_REG                    0x32
#define BMA250_EEPROM_CTRL_REG                  0x33
#define BMA250_SERIAL_CTRL_REG                  0x34
#define BMA250_CTRL_UNLOCK_REG                  0x35
#define BMA250_OFFSET_CTRL_REG                  0x36
#define BMA250_OFFSET_PARAMS_REG                0x37
#define BMA250_OFFSET_FILT_X_REG                0x38
#define BMA250_OFFSET_FILT_Y_REG                0x39
#define BMA250_OFFSET_FILT_Z_REG                0x3A
#define BMA250_OFFSET_UNFILT_X_REG              0x3B
#define BMA250_OFFSET_UNFILT_Y_REG              0x3C
#define BMA250_OFFSET_UNFILT_Z_REG              0x3D
#define BMA250_SPARE_0_REG                      0x3E
#define BMA250_SPARE_1_REG                      0x3F




#define BMA250_ACC_X_LSB__POS                   6
#define BMA250_ACC_X_LSB__LEN                   2
#define BMA250_ACC_X_LSB__MSK                   0xC0
#define BMA250_ACC_X_LSB__REG                   BMA250_X_AXIS_LSB_REG

#define BMA250_ACC_X_MSB__POS                   0
#define BMA250_ACC_X_MSB__LEN                   8
#define BMA250_ACC_X_MSB__MSK                   0xFF
#define BMA250_ACC_X_MSB__REG                   BMA250_X_AXIS_MSB_REG

#define BMA250_ACC_Y_LSB__POS                   6
#define BMA250_ACC_Y_LSB__LEN                   2
#define BMA250_ACC_Y_LSB__MSK                   0xC0
#define BMA250_ACC_Y_LSB__REG                   BMA250_Y_AXIS_LSB_REG

#define BMA250_ACC_Y_MSB__POS                   0
#define BMA250_ACC_Y_MSB__LEN                   8
#define BMA250_ACC_Y_MSB__MSK                   0xFF
#define BMA250_ACC_Y_MSB__REG                   BMA250_Y_AXIS_MSB_REG

#define BMA250_ACC_Z_LSB__POS                   6
#define BMA250_ACC_Z_LSB__LEN                   2
#define BMA250_ACC_Z_LSB__MSK                   0xC0
#define BMA250_ACC_Z_LSB__REG                   BMA250_Z_AXIS_LSB_REG

#define BMA250_ACC_Z_MSB__POS                   0
#define BMA250_ACC_Z_MSB__LEN                   8
#define BMA250_ACC_Z_MSB__MSK                   0xFF
#define BMA250_ACC_Z_MSB__REG                   BMA250_Z_AXIS_MSB_REG

#define BMA250_RANGE_SEL__POS                   0
#define BMA250_RANGE_SEL__LEN                   4
#define BMA250_RANGE_SEL__MSK                   0x0F
#define BMA250_RANGE_SEL__REG                   BMA250_RANGE_SEL_REG

#define BMA250_BW__POS                          0
#define BMA250_BW__LEN                          5
#define BMA250_BW__MSK                          0x1F
#define BMA250_BW__REG                          BMA250_BW_SEL_REG

#define BMA250_EN_LOW_POWER__POS                6
#define BMA250_EN_LOW_POWER__LEN                1
#define BMA250_EN_LOW_POWER__MSK                0x40
#define BMA250_EN_LOW_POWER__REG                BMA250_MODE_CTRL_REG

#define BMA250_EN_SUSPEND__POS                  7
#define BMA250_EN_SUSPEND__LEN                  1
#define BMA250_EN_SUSPEND__MSK                  0x80
#define BMA250_EN_SUSPEND__REG                  BMA250_MODE_CTRL_REG

#define BMA250_GET_BITSLICE(regvar, bitname)\
			((regvar & bitname##__MSK) >> bitname##__POS)


#define BMA250_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))


/* range and bandwidth */

enum bma250_range {
	BMA250_RANGE_2G  = 0x3,
	BMA250_RANGE_4G  = 0x5,
	BMA250_RANGE_8G  = 0x8,
	BMA250_RANGE_16G = 0xC,

	BMA250_RANGE_COUNT = 16
};

static unsigned char* bma250_range_name[] = {
	NULL , NULL, NULL, "2g",
	NULL , "4g", NULL, NULL,
	"8g" , NULL, NULL, NULL,
	"16g", NULL, NULL, NULL
};


enum bma250_bw {
	BMA250_BW_7_81HZ  = 0x8,
	BMA250_BW_15_63HZ = 0x9,
	BMA250_BW_31_25HZ = 0xA,
	BMA250_BW_62_50HZ = 0xB,
	BMA250_BW_125HZ   = 0xC,
	BMA250_BW_250HZ   = 0xD,
	BMA250_BW_500HZ   = 0xE,
	BMA250_BW_1000HZ  = 0xF,
};

static unsigned int bma250_bandwidth_frequency[] = {
	   7810,    7810,    7810,    7810,    7810,    7810,    7810,    7810,
	   7810,   15630,   31250,   62500,  125000,  250000,  500000, 1000000,
	1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000,
	1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000,
};

static unsigned int bma250_bandwidth_update_time[] = {
	64000, 64000, 64000, 64000, 64000, 64000, 64000, 64000,
	64000, 32000, 16000,  8000,  4000,  2000,  1000,   500,
	  500,   500,   500,   500,   500,   500,   500,   500,
	  500,   500,   500,   500,   500,   500,   500,   500,
};

/* mode settings */

enum bma250_mode {
	BMA250_MODE_NORMAL,
	BMA250_MODE_LOWPOWER,
	BMA250_MODE_SUSPEND,

	BMA250_MODE_COUNT
};

const char* bma250_mode_name[] = {
	"normal",
	"lowpower",
	"suspend",

	NULL
};



struct bma250acc {
	s16	x, y, z;
};

struct bma250_data {
	struct i2c_client *bma250_client;

	enum bma_chip_id chip_id;

	enum bma250_mode mode;

	unsigned char mode_ctrl;
	unsigned char range_sel;
	unsigned char bw_sel;

	atomic_t delay;
	atomic_t enable;
	struct input_dev *input;
	struct bma250acc value;
	struct mutex value_mutex;
	struct mutex enable_mutex;
	struct delayed_work work;
	struct work_struct irq_work;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

/* Addresses to scan */
static const unsigned short normal_i2c[] = { 0x18, 0x19, 0x38, 0x08, I2C_CLIENT_END };
struct input_dev *this_dev = NULL;
extern int flag_suspend;
static int old_value = 0;

static const int chip_id_value[] = { BMA_CHIP_ID_150, BMA_CHIP_ID_250, BMA_CHIP_ID_250E, BMA_CHIP_ID_255, BMA_CHIP_ID_280, 0 };
static const char* chip_id_name[] = { "bma150", "bma250", "bma250e", "bma255", "bma280", NULL };

static struct sensor_config_info config_info = {
	.input_type = GSENSOR_TYPE,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bma250_early_suspend(struct early_suspend *h);
static void bma250_late_resume(struct early_suspend *h);
#endif


/**
 * gsensor_detect - Device detection callback for automatic device creation
 * return value:  
 *                    = 0; success;
 *                    < 0; err
 */
static int gsensor_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	int ret, i = 0, retry = 2;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	if (config_info.twi_id == adapter->nr) {
		while (retry--) {
			ret = i2c_smbus_read_byte_data(client, BMA250_CHIP_ID_REG);

			if (ret >= 0)
			{
				for (i = 0; chip_id_value[i]; i++) {
					if ((ret & 0xFF) == chip_id_value[i]) {
						printk(KERN_INFO "BMA chip identified as %s (0x%02X).\n",
							chip_id_name[i], chip_id_value[i]);
						strlcpy(info->type, SENSOR_NAME, I2C_NAME_SIZE);
						return 0;
					}
				}

				printk(KERN_WARNING "Unsupported Bosch Sensortec device id (%d).\n", ret);
				break;
			}

			msleep(1);
		}

		printk(KERN_WARNING "Bosch Sensortec device not found.\n");
	}

	return -ENODEV;
}

static int bma250_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if (dummy < 0)
		return -1;
	*data = dummy & 0x000000ff;

	return 0;
}

static int bma250_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);

	if (dummy < 0)
		return -1;
	return 0;
}

static void key_press_powerkey_power(void)
{
	input_report_key(this_dev, KEY_POWER, 1);
	input_sync(this_dev);
	input_report_key(this_dev, KEY_POWER, 0);
	input_sync(this_dev);
	flag_suspend = 0;
}


static int bma250_get_chip_id(struct bma250_data *bma250, enum bma_chip_id *chip_id)
{
	unsigned char id;

	if (!bma250)
		return -1;

	if (bma250_smbus_read_byte(bma250->bma250_client,
		BMA250_CHIP_ID_REG, &id) < 0)
		return -1;
	bma250->chip_id = id;

	if (chip_id)
		*chip_id = bma250->chip_id;
	return 0;
}

static int bma250_set_mode(struct bma250_data *bma250, enum bma250_mode mode)
{
	unsigned char mode_ctrl;

	if (!bma250 || (mode >= BMA250_MODE_COUNT))
		return -1;

	if (bma250->mode == mode)
		return 0;

	mode_ctrl = bma250->mode_ctrl;

	switch (mode) {
	case BMA250_MODE_LOWPOWER:
		mode_ctrl = BMA250_SET_BITSLICE(mode_ctrl, BMA250_EN_LOW_POWER, 1);
		break;
	default:
		mode_ctrl = BMA250_SET_BITSLICE(mode_ctrl, BMA250_EN_LOW_POWER, 0);
		break;
	}

	switch (mode) {
	case BMA250_MODE_SUSPEND:
		mode_ctrl = BMA250_SET_BITSLICE(mode_ctrl, BMA250_EN_SUSPEND, 1);
		break;
	default:
		mode_ctrl = BMA250_SET_BITSLICE(mode_ctrl, BMA250_EN_SUSPEND, 0);
		break;
	}

	if (mode_ctrl != bma250->mode_ctrl) {
		if (bma250_smbus_write_byte(bma250->bma250_client,
			BMA250_MODE_CTRL_REG, &mode_ctrl) < 0)
			return -1;

		bma250->mode_ctrl = mode_ctrl;
		bma250->mode      = mode;
	}

	return 0;
}

static int bma250_get_mode(struct bma250_data *bma250, unsigned char *mode)
{
	unsigned char mode_ctrl;

	if (!bma250)
		return -1;

	if (bma250_smbus_read_byte(bma250->bma250_client,
		BMA250_MODE_CTRL_REG, &mode_ctrl) < 0)
		return -1;

	bma250->mode_ctrl = mode_ctrl;

	if(BMA250_GET_BITSLICE(mode_ctrl, BMA250_EN_SUSPEND)) {
		bma250->mode = BMA250_MODE_SUSPEND;
	} else if(BMA250_GET_BITSLICE(mode_ctrl, BMA250_EN_LOW_POWER)) {
		bma250->mode = BMA250_MODE_LOWPOWER;
	} else {
		bma250->mode = BMA250_MODE_NORMAL;
	}

	if (mode)
		*mode = bma250->mode;
	return 0;
}

static int bma250_set_range(struct bma250_data *bma250, enum bma250_range range)
{
	unsigned char range_sel;

	if (!bma250)
		return -1;

	switch (range) {
		case BMA250_RANGE_2G:
		case BMA250_RANGE_4G:
		case BMA250_RANGE_8G:
		case BMA250_RANGE_16G:
			break;
		default:
			printk(KERN_WARNING "bma250: Setting range to unknown value (%u).\n",
				(unsigned int) range);
			break;
	}

	range_sel = bma250->range_sel;
	range_sel = BMA250_SET_BITSLICE(range_sel,
		BMA250_RANGE_SEL, range);

	if (range_sel != bma250->range_sel) {
		if (bma250_smbus_write_byte(bma250->bma250_client,
			BMA250_RANGE_SEL_REG, &range_sel) < 0)
			return -1;
		bma250->range_sel = range_sel;
	}

	return 0;
}

static int bma250_get_range(struct bma250_data *bma250, enum bma250_range *range)
{
	unsigned char range_sel;

	if (!bma250)
		return -1;

	if (bma250_smbus_read_byte(bma250->bma250_client,
		BMA250_RANGE_SEL_REG, &range_sel) < 0)
		return -1;

	bma250->range_sel = range_sel;

	if (range)
		*range = BMA250_GET_BITSLICE(range_sel, BMA250_RANGE_SEL);
	return 0;
}


static int bma250_set_bandwidth(struct bma250_data *bma250, enum bma250_bw bw)
{
	unsigned char bw_sel;

	if (!bma250 || (bw >= 32))
		return -1;

	bw_sel = bma250->bw_sel;
	bw_sel = BMA250_SET_BITSLICE(bw_sel, BMA250_BW, bw);

	if (bw_sel != bma250->bw_sel) {
		if (bma250_smbus_write_byte(bma250->bma250_client,
			BMA250_BW__REG, &bw_sel) < 0)
			return -1;
		bma250->bw_sel = bw_sel;
	}

	return 0;
}

static int bma250_get_bandwidth(struct bma250_data *bma250, unsigned char *bw)
{
	unsigned char bw_sel;

	if (!bma250)
		return -1;

	if (bma250_smbus_read_byte(bma250->bma250_client,
		BMA250_BW__REG, &bw_sel) < 0)
		return -1;
	bma250->bw_sel = bw_sel;

	if (bw)
		*bw = BMA250_GET_BITSLICE(bw_sel, BMA250_BW);
	return 0;
}

static int bma250_read_accel_xyz(struct i2c_client *client,
							struct bma250acc *acc)
{
	int comres;
	unsigned char data[6] = {0};
	if (client == NULL) {
		comres = -1;
	} else {
		comres = i2c_smbus_read_i2c_block_data(client, BMA250_ACC_X_LSB__REG, 6, data);

		acc->x = BMA250_GET_BITSLICE(data[0], BMA250_ACC_X_LSB)
			|(BMA250_GET_BITSLICE(data[1],
				BMA250_ACC_X_MSB)<<BMA250_ACC_X_LSB__LEN);
		acc->x = acc->x << (sizeof(short)*8-(BMA250_ACC_X_LSB__LEN
					+ BMA250_ACC_X_MSB__LEN));
		acc->x = acc->x >> (sizeof(short)*8-(BMA250_ACC_X_LSB__LEN
					+ BMA250_ACC_X_MSB__LEN));
		acc->y = BMA250_GET_BITSLICE(data[2], BMA250_ACC_Y_LSB)
			| (BMA250_GET_BITSLICE(data[3],
				BMA250_ACC_Y_MSB)<<BMA250_ACC_Y_LSB__LEN);
		acc->y = acc->y << (sizeof(short)*8-(BMA250_ACC_Y_LSB__LEN
					+ BMA250_ACC_Y_MSB__LEN));
		acc->y = acc->y >> (sizeof(short)*8-(BMA250_ACC_Y_LSB__LEN
					+ BMA250_ACC_Y_MSB__LEN));

		acc->z = BMA250_GET_BITSLICE(data[4], BMA250_ACC_Z_LSB)
			| (BMA250_GET_BITSLICE(data[5],
				BMA250_ACC_Z_MSB)<<BMA250_ACC_Z_LSB__LEN);
		acc->z = acc->z << (sizeof(short)*8-(BMA250_ACC_Z_LSB__LEN
					+ BMA250_ACC_Z_MSB__LEN));
		acc->z = acc->z >> (sizeof(short)*8-(BMA250_ACC_Z_LSB__LEN
					+ BMA250_ACC_Z_MSB__LEN));
	}

	return comres;
}

static void bma250_work_func(struct work_struct *work)
{
	struct bma250_data *bma250 = container_of((struct delayed_work *)work,
			struct bma250_data, work);
	static struct bma250acc acc;
	unsigned long delay = msecs_to_jiffies(atomic_read(&bma250->delay));

	bma250_read_accel_xyz(bma250->bma250_client, &acc);
	input_report_abs(bma250->input, ABS_X, acc.x);
	input_report_abs(bma250->input, ABS_Y, acc.y);
	input_report_abs(bma250->input, ABS_Z, acc.z);
	dprintk(DEBUG_DATA_INFO, "acc.x %d, acc.y %d, acc.z %d\n", acc.x, acc.y, acc.z);

	if (flag_suspend
		&& (((old_value + 20) < acc.z)
			|| ((old_value - 20) > acc.z)))
	{
	     key_press_powerkey_power();
	}

	input_sync(bma250->input);
	mutex_lock(&bma250->value_mutex);
	bma250->value = acc;
	mutex_unlock(&bma250->value_mutex);
	old_value = acc.z;
	schedule_delayed_work(&bma250->work, delay);
}

static ssize_t bma250_chip_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i;
	const char* chip_name = "unknown";

	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	for (i = 0; chip_id_value[i]; i++)
	{
		if (chip_id_value[i] == bma250->chip_id) {
			chip_name = chip_id_name[i];
			break;
		}
	}

	dprintk(DEBUG_CONTROL_INFO, "%s (0x%02X), %s\n", chip_name, bma250->chip_id, __FUNCTION__);
	return sprintf(buf, "%s (0x%02X)\n", chip_name, bma250->chip_id);
}

static ssize_t bma250_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	enum bma250_range range;

	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	range = BMA250_GET_BITSLICE(bma250->range_sel, BMA250_RANGE_SEL);
	if (bma250_range_name[range])
		return sprintf(buf, "%s\n", bma250_range_name[range]);
	return sprintf(buf, "%u\n", (unsigned int) range);
}

static ssize_t bma250_range_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long range;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	for (range = 0; range < BMA250_RANGE_COUNT; range++) {
		if (bma250_range_name[range]
			&& (strcmp(buf, bma250_range_name[range]) == 0))
			break;
	}
	if (range >= BMA250_RANGE_COUNT) {
		error = strict_strtoul(buf, 10, &range);
		if (error)
			return error;
		if (range >= BMA250_RANGE_COUNT)
			return -EINVAL;
	}

	if (bma250_set_range(bma250, (enum bma250_range) range) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_bw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char bw_sel;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	bw_sel = BMA250_GET_BITSLICE(bma250->bw_sel, BMA250_BW);

	return sprintf(buf, "%u\n", (unsigned int) bw_sel);
}

static ssize_t bma250_bw_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long bw;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &bw);
	if (error)
		return error;
	if (bma250_set_bandwidth(bma250, (unsigned char) bw) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_bandwidth_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char bw_sel;
	unsigned int  bandwidth;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	bw_sel = BMA250_GET_BITSLICE(bma250->bw_sel, BMA250_BW);
	bandwidth = bma250_bandwidth_frequency[bw_sel];

	return sprintf(buf, "%u.%03u Hz\n",
		(bandwidth / 1000), (bandwidth % 1000));
}

static ssize_t bma250_update_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char bw_sel;
	unsigned int update_time;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	bw_sel = BMA250_GET_BITSLICE(bma250->bw_sel, BMA250_BW);
	update_time = bma250_bandwidth_update_time[bw_sel];

	return sprintf(buf, "%u.%03u ms\n",
		(update_time / 1000), (update_time % 1000));
}

static ssize_t bma250_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	const char* mode_name = "unknown";

	if (bma250->mode < BMA250_MODE_COUNT)
		mode_name = bma250_mode_name[bma250->mode];

	return sprintf(buf, "%s\n", mode_name);
}

static ssize_t bma250_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int i;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	for (i = 0; bma250_mode_name[i]; i++) {
		if (strcmp(bma250_mode_name[i], buf) == 0) {
			if (bma250_set_mode(bma250, i) < 0) {
				printk(KERN_WARNING "Failed to set mode '%.*s'.\n", count, buf);
			} else {
				return count;
			}
		}
	}

	printk(KERN_WARNING "Invalid mode '%.*s'.\n", count, buf);
	return -EINVAL;
}


static ssize_t bma250_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma250_data *bma250 = input_get_drvdata(input);
	struct bma250acc acc_value;

	mutex_lock(&bma250->value_mutex);
	acc_value = bma250->value;
	mutex_unlock(&bma250->value_mutex);

	printk("x=%d, y=%d, z=%d ,%s\n", acc_value.x, acc_value.y, acc_value.z, __FUNCTION__);
	return sprintf(buf, "%d %d %d\n", acc_value.x, acc_value.y,
			acc_value.z);
}

static ssize_t bma250_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	dprintk(DEBUG_CONTROL_INFO, "%d, %s\n", atomic_read(&bma250->delay), __FUNCTION__);
	return sprintf(buf, "%d\n", atomic_read(&bma250->delay));

}

static ssize_t bma250_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);

	if (error)
		return error;

	if (data > BMA250_MAX_DELAY)
		data = BMA250_MAX_DELAY;

	atomic_set(&bma250->delay, (unsigned int) data);

	return count;
}


static ssize_t bma250_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	dprintk(DEBUG_CONTROL_INFO, "%d, %s\n", atomic_read(&bma250->enable), __FUNCTION__);
	return sprintf(buf, "%d\n", atomic_read(&bma250->enable));

}

static void bma250_set_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);
	int pre_enable = atomic_read(&bma250->enable);

    printk("bma250_set_enable\n");
	mutex_lock(&bma250->enable_mutex);

	if (enable && !pre_enable) {
		bma250_set_mode(bma250,
						BMA250_MODE_NORMAL);
		schedule_delayed_work(&bma250->work,
			msecs_to_jiffies(atomic_read(&bma250->delay)));
		atomic_set(&bma250->enable, 1);
	} else if (!enable && pre_enable) {
		printk("bma250_set_enable pre_enable\n");
/*
		bma250_set_mode(bma250,
						BMA250_MODE_SUSPEND);
		cancel_delayed_work_sync(&bma250->work);
*/
		atomic_set(&bma250->enable, 0);
	}

	mutex_unlock(&bma250->enable_mutex);
}

static ssize_t bma250_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = strict_strtoul(buf, 10, &data);

	if (error)
		return error;

	if ((data == 0) || (data == 1)) {
		bma250_set_enable(dev, data);
	}

	return count;
}

static DEVICE_ATTR(chip_id, S_IRUGO,
		bma250_chip_id_show, NULL);
static DEVICE_ATTR(range, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_range_show, bma250_range_store);
static DEVICE_ATTR(bw, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_bw_show, bma250_bw_store);
static DEVICE_ATTR(bandwidth, S_IRUGO,
		bma250_bandwidth_show, NULL);
static DEVICE_ATTR(update_time, S_IRUGO,
		bma250_update_time_show, NULL);
static DEVICE_ATTR(mode, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_mode_show, bma250_mode_store);
static DEVICE_ATTR(value, S_IRUGO,
		bma250_value_show, NULL);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_delay_show, bma250_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_enable_show, bma250_enable_store);

static struct attribute *bma250_attributes[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_range.attr,
	&dev_attr_bw.attr,
	&dev_attr_bandwidth.attr,
	&dev_attr_update_time.attr,
	&dev_attr_mode.attr,
	&dev_attr_value.attr,
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group bma250_attribute_group = {
	.attrs = bma250_attributes
};

static int bma250_input_init(struct bma250_data *bma250)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_X, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_drvdata(dev, bma250);
    set_bit(EV_KEY, dev->evbit);
    set_bit(EV_REL, dev->evbit);
	set_bit(KEY_POWER, dev->keybit);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	this_dev = dev;
	bma250->input = dev;

	return 0;
}

static void bma250_input_delete(struct bma250_data *bma250)
{
	struct input_dev *dev = bma250->input;

	input_unregister_device(dev);
	input_free_device(dev);
}

static int bma250_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	struct bma250_data *data;

	dprintk(DEBUG_INIT, "bma250: probe\n");
	dprintk(DEBUG_INIT, "bma250 probe i2c address is 0x%x \n", 
	        client->addr);

	data = kzalloc(sizeof(struct bma250_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);
	data->bma250_client = client;

	mutex_init(&data->value_mutex);
	mutex_init(&data->enable_mutex);

	bma250_get_chip_id(data, NULL);

	bma250_set_mode(data, BMA250_MODE_SET);
	bma250_set_bandwidth(data, BMA250_BW_SET);
	bma250_set_range(data, BMA250_RANGE_SET);

	INIT_DELAYED_WORK(&data->work, bma250_work_func);
	dprintk(DEBUG_INIT, "bma: INIT_DELAYED_WORK\n");
	atomic_set(&data->delay, BMA250_MAX_DELAY);
	atomic_set(&data->enable, 0);

	err = bma250_input_init(data);
	if (err < 0){
		printk("bma: bma250_input_init err\n");
		goto kfree_exit;
	}

	err = sysfs_create_group(&data->input->dev.kobj,
				 &bma250_attribute_group);
	if (err < 0){
		printk("bma: sysfs_create_group err\n");
		goto error_sysfs;
	}

/*
#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = bma250_early_suspend;
	data->early_suspend.resume = bma250_late_resume;
	register_early_suspend(&data->early_suspend);
#endif
*/

	dprintk(DEBUG_INIT, "bma250: probe end\n");
	return 0;

error_sysfs:
	bma250_input_delete(data);

kfree_exit:
	kfree(data);
exit:
	return err;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bma250_early_suspend(struct early_suspend *h)
{
	struct bma250_data *data =
		container_of(h, struct bma250_data, early_suspend);

	dprintk(DEBUG_SUSPEND, "bma250: early suspend\n");
	printk("bma250_early_suspend\n");
	if (NORMAL_STANDBY == standby_type) {
		printk("bma250_late_resume\n");

/*
		mutex_lock(&data->enable_mutex);
		if (atomic_read(&data->enable) == 1) {
			bma250_set_mode(data, BMA250_MODE_SUSPEND);
			cancel_delayed_work_sync(&data->work);
		}
		mutex_unlock(&data->enable_mutex);
*/
	} else if (SUPER_STANDBY == standby_type) {
		mutex_lock(&data->enable_mutex);
		if (atomic_read(&data->enable) == 1) {
			bma250_set_mode(data, BMA250_MODE_SUSPEND);
			cancel_delayed_work_sync(&data->work);
		}
		mutex_unlock(&data->enable_mutex);
	}
}

static void bma250_late_resume(struct early_suspend *h)
{
	struct bma250_data *data =
		container_of(h, struct bma250_data, early_suspend);

	dprintk(DEBUG_SUSPEND, "bma250: late resume\n");
	printk("bma250_late_resume\n");
	if (NORMAL_STANDBY == standby_type) {
		printk("bma250_late_resume\n");

/*
		mutex_lock(&data->enable_mutex);
		if (atomic_read(&data->enable) == 1) {
			bma250_set_mode(data, BMA250_MODE_NORMAL);
			schedule_delayed_work(&data->work,
				msecs_to_jiffies(atomic_read(&data->delay)));
		}
		mutex_unlock(&data->enable_mutex);
*/
	} else if (SUPER_STANDBY == standby_type) {
		printk("bma250_late_resume\n");
		if (bma250_smbus_write_byte(data->bma250_client,
			BMA250_BW_SEL_REG, &data->bw_sel) < 0)
			printk("suspend: write bandwidth err\n");

		if (bma250_smbus_write_byte(data->bma250_client,
			BMA250_RANGE_SEL_REG, &data->range_sel) < 0)
			printk("suspend: write range err\n");

		mutex_lock(&data->enable_mutex);
		if (atomic_read(&data->enable) == 1) {
			bma250_set_mode(data, BMA250_MODE_NORMAL);
			schedule_delayed_work(&data->work,
				msecs_to_jiffies(atomic_read(&data->delay)));
		}
		mutex_unlock(&data->enable_mutex);
	}
}
#else
#ifdef CONFIG_PM
static int bma250_resume(struct i2c_client *client)
{
	struct bma250_data *data = i2c_get_clientdata(client);

	dprintk(DEBUG_SUSPEND, "bma250: resume\n");

	if (NORMAL_STANDBY == standby_type) {
		mutex_lock(&data->enable_mutex);
		
		if (atomic_read(&data->enable) == 1) {
			bma250_set_mode(data, BMA250_MODE_NORMAL);
			schedule_delayed_work(&data->work,
				msecs_to_jiffies(atomic_read(&data->delay)));
		}

		mutex_unlock(&data->enable_mutex);
	} else if (SUPER_STANDBY == standby_type) {
		if (bma250_smbus_write_byte(data->bma250_client,
			BMA250_BW_SEL_REG, &data->bw_sel) < 0)
			printk("suspend: write bandwidth err\n");

		if (bma250_smbus_write_byte(data->bma250_client,
			BMA250_RANGE_SEL_REG, &data->range_sel) < 0)
			printk("suspend: write range err\n");

		mutex_lock(&data->enable_mutex);
		if (atomic_read(&data->enable) == 1) {
			bma250_set_mode(data, BMA250_MODE_NORMAL);
			schedule_delayed_work(&data->work,
				msecs_to_jiffies(atomic_read(&data->delay)));
		}
		mutex_unlock(&data->enable_mutex);
	}
	return 0;
}

static int bma250_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct bma250_data *data = i2c_get_clientdata(client);

	dprintk(DEBUG_SUSPEND, "bma250: suspend\n");

	if (NORMAL_STANDBY == standby_type) {
		mutex_lock(&data->enable_mutex);

		if (atomic_read(&data->enable) == 1) {
			bma250_set_mode(data, BMA250_MODE_SUSPEND);
			cancel_delayed_work_sync(&data->work);
		}

		mutex_unlock(&data->enable_mutex);
	} else if (SUPER_STANDBY == standby_type) {
		mutex_lock(&data->enable_mutex);
		if (atomic_read(&data->enable) == 1) {
			bma250_set_mode(data, BMA250_MODE_SUSPEND);
			cancel_delayed_work_sync(&data->work);
		}
		mutex_unlock(&data->enable_mutex);
	}
	return 0;
}
#endif
#endif /* CONFIG_HAS_EARLYSUSPEND */

static int bma250_remove(struct i2c_client *client)
{
	struct bma250_data *data = i2c_get_clientdata(client);

	bma250_set_enable(&client->dev, 0);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif
	sysfs_remove_group(&data->input->dev.kobj, &bma250_attribute_group);
	bma250_input_delete(data);
	i2c_set_clientdata(client, NULL);
	kfree(data);
	return 0;
}


static const struct i2c_device_id bma250_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma250_id);

static struct i2c_driver bma250_driver = {
	.class  = I2C_CLASS_HWMON,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= SENSOR_NAME,
	},
	.id_table	= bma250_id,
	.probe		= bma250_probe,
	.remove		= bma250_remove,
/*
#ifdef CONFIG_HAS_EARLYSUSPEND
#else
#ifdef CONFIG_PM
	.suspend = bma250_suspend,
	.resume  = bma250_resume,
#endif
#endif
*/
	.address_list	= normal_i2c,
};

static int __init BMA250_init(void)
{
	int ret = -1;
	dprintk(DEBUG_INIT, "bma250: init\n");

	if (input_fetch_sysconfig_para(&(config_info.input_type))){
		printk("%s: err.\n", __func__);
		return -1;
	}

	if (config_info.sensor_used == 0){
		printk("*** used set to 0 !\n");
		printk("*** if use sensor,please put the sys_config.fex gsensor_used set to 1. \n");
		return 0;
	}

	bma250_driver.detect = gsensor_detect;

	ret = i2c_add_driver(&bma250_driver);

	return ret;
}

static void __exit BMA250_exit(void)
{
	i2c_del_driver(&bma250_driver);
}

MODULE_AUTHOR("Albert Zhang <xu.zhang@bosch-sensortec.com>");
MODULE_DESCRIPTION("BMA250 driver");
MODULE_LICENSE("GPL");

module_init(BMA250_init);
module_exit(BMA250_exit);
