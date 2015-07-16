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
#define BMA250_SLEEP_DUR_SET                    100000
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
#define BMA250_ACC_X_LSB__REG                   BMA250_X_AXIS_LSB_REG

#define BMA250_ACC_X_MSB__POS                   0
#define BMA250_ACC_X_MSB__LEN                   8
#define BMA250_ACC_X_MSB__REG                   BMA250_X_AXIS_MSB_REG

#define BMA250_ACC_Y_LSB__POS                   6
#define BMA250_ACC_Y_LSB__LEN                   2
#define BMA250_ACC_Y_LSB__REG                   BMA250_Y_AXIS_LSB_REG

#define BMA250_ACC_Y_MSB__POS                   0
#define BMA250_ACC_Y_MSB__LEN                   8
#define BMA250_ACC_Y_MSB__REG                   BMA250_Y_AXIS_MSB_REG

#define BMA250_ACC_Z_LSB__POS                   6
#define BMA250_ACC_Z_LSB__LEN                   2
#define BMA250_ACC_Z_LSB__REG                   BMA250_Z_AXIS_LSB_REG

#define BMA250_ACC_Z_MSB__POS                   0
#define BMA250_ACC_Z_MSB__LEN                   8
#define BMA250_ACC_Z_MSB__REG                   BMA250_Z_AXIS_MSB_REG

#define BMA250_SLOPE_INT__POS                   2
#define BMA250_SLOPE_INT__LEN                   1
#define BMA250_SLOPE_INT__REG                   BMA250_STATUS1_REG

#define BMA250_RANGE_SEL__POS                   0
#define BMA250_RANGE_SEL__LEN                   4
#define BMA250_RANGE_SEL__REG                   BMA250_RANGE_SEL_REG

#define BMA250_BW__POS                          0
#define BMA250_BW__LEN                          5
#define BMA250_BW__REG                          BMA250_BW_SEL_REG

#define BMA250E_LOW_POWER_MODE__POS             6
#define BMA250E_LOW_POWER_MODE__LEN             1
#define BMA250E_LOW_POWER_MODE__REG             BMA250_LOW_NOISE_CTRL_REG

#define BMA250_SLEEP_DUR__POS                   1
#define BMA250_SLEEP_DUR__LEN                   4
#define BMA250_SLEEP_DUR__REG                   BMA250_MODE_CTRL_REG

#define BMA250E_DEEP_SUSPEND__POS               5
#define BMA250E_DEEP_SUSPEND__LEN               1
#define BMA250E_DEEP_SUSPEND__REG               BMA250_MODE_CTRL_REG

#define BMA250_EN_LOW_POWER__POS                6
#define BMA250_EN_LOW_POWER__LEN                1
#define BMA250_EN_LOW_POWER__REG                BMA250_MODE_CTRL_REG

#define BMA250_EN_SUSPEND__POS                  7
#define BMA250_EN_SUSPEND__LEN                  1
#define BMA250_EN_SUSPEND__REG                  BMA250_MODE_CTRL_REG

#define BMA250_SLOPE_EN_X__POS                  0
#define BMA250_SLOPE_EN_X__LEN                  1
#define BMA250_SLOPE_EN_X__REG                  BMA250_INT_ENABLE1_REG

#define BMA250_SLOPE_EN_Y__POS                  1
#define BMA250_SLOPE_EN_Y__LEN                  1
#define BMA250_SLOPE_EN_Y__REG                  BMA250_INT_ENABLE1_REG

#define BMA250_SLOPE_EN_Z__POS                  2
#define BMA250_SLOPE_EN_Z__LEN                  1
#define BMA250_SLOPE_EN_Z__REG                  BMA250_INT_ENABLE1_REG

#define BMA250_LATCH_INT__POS                   0
#define BMA250_LATCH_INT__LEN                   4
#define BMA250_LATCH_INT__REG                   BMA250_INT_CTRL_REG

#define BMA250_INT_SRC_SLOPE__POS               0
#define BMA250_INT_SRC_SLOPE__LEN               2
#define BMA250_INT_SRC_SLOPE__REG               BMA250_INT_SRC_REG

#define BMA250_SLOPE_DUR__POS                   0
#define BMA250_SLOPE_DUR__LEN                   2
#define BMA250_SLOPE_DUR__REG                   BMA250_SLOPE_DURN_REG

#define BMA250_SLOPE_TH__POS                    0
#define BMA250_SLOPE_TH__LEN                    8
#define BMA250_SLOPE_TH__REG                    BMA250_SLOPE_THRES_REG


#define BMA250_BITMASK(bitname)\
	(((1U << bitname##__LEN) - 1) << bitname##__POS)

#define BMA250_GET_BITSLICE(regvar, bitname)\
	((regvar & BMA250_BITMASK(bitname)) >> bitname##__POS)

#define BMA250_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~BMA250_BITMASK(bitname)) | ((val << bitname##__POS) & BMA250_BITMASK(bitname)))

#define BMA250_GET_STATE_BITSLICE(state, bitname)\
	BMA250_GET_BITSLICE(state[bitname##__REG], bitname)

#define BMA250_SET_STATE_BITSLICE(state, bitname, val)\
	BMA250_SET_BITSLICE(state[bitname##__REG], bitname, val)


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

static unsigned int bma250_sleep_dur_value[] = {
	  500,    500,    500,     500,
	  500,    500,   1000,    2000,
	 4000,   6000,  10000,   25000,
	50000, 100000, 500000, 1000000,
};


/* interrupt latching */

enum bma250_latch_int {
	BMA250_LATCH_INT_NON_LATCHED      =  0,
	BMA250_LATCH_INT_TEMPORARY_250MS  =  1,
	BMA250_LATCH_INT_TEMPORARY_500MS  =  2,
	BMA250_LATCH_INT_TEMPORARY_1S     =  3,
	BMA250_LATCH_INT_TEMPORARY_2S     =  4,
	BMA250_LATCH_INT_TEMPORARY_4S     =  5,
	BMA250_LATCH_INT_TEMPORARY_8S     =  6,
	BMA250_LATCH_INT_LATCHED          =  7,
	BMA250_LATCH_INT_NON_LATCHED_1    =  8,
	BMA250_LATCH_INT_TEMPORARY_250US  =  9,
	BMA250_LATCH_INT_TEMPORARY_500US  = 10,
	BMA250_LATCH_INT_TEMPORARY_1MS    = 11,
	BMA250_LATCH_INT_TEMPORARY_12_5MS = 12,
	BMA250_LATCH_INT_TEMPORARY_25MS   = 13,
	BMA250_LATCH_INT_TEMPORARY_50MS   = 14,
	BMA250_LATCH_INT_LATCHED_1        = 15,

	BMA250_LATCH_INT_COUNT = 16
};

static unsigned int bma250_latch_int_time[] = {
	0, 250000, 500000, 1000000, 2000000, 4000000, 8000000, 0,
	0,    250,    500,    1000,   12500,   25000,   50000, 0,
};

static const char* bma250_latch_int_name[] = {
	"non-latched", NULL, NULL, NULL, NULL, NULL, NULL, "latched",
	"non-latched", NULL, NULL, NULL, NULL, NULL, NULL, "latched",
};


/* mode settings */

enum bma250_mode {
	BMA250_MODE_NORMAL,
	BMA250_MODE_LOWPOWER1,
	BMA250_MODE_SUSPEND,

	BMA250E_MODE_LOWPOWER2,
	BMA250E_MODE_STANDBY,
	BMA250E_MODE_DEEP_SUSPEND,

	BMA250_MODE_COUNT
};

const char* bma250_mode_name[] = {
	"normal",
	"lowpower1",
	"suspend",

	"lowpower2",
	"standby",
	"deep-suspend",

	NULL
};



struct bma250_acc {
	s16	x, y, z;
};

struct bma250_data {
	struct i2c_client *bma250_client;

	enum bma250_mode mode;

	unsigned char state[64];

	atomic_t delay;
	atomic_t enable;
	struct input_dev *input;
	struct bma250_acc value;
	struct mutex device_mutex;
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


static int bma250_reg_is_state_r[64] = {
	1, 1, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 1,
	1, 1, 1, 1, 0, 0, 1, 1,
	1, 1, 1, 1, 0, 0, 1, 0,
	1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1,
	1, 0, 1, 1, 1, 0, 1, 1,
	1, 1, 1, 1, 1, 0, 1, 0,
};

static int bma250_reg_is_state_w[64] = {
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 1,
	1, 1, 1, 1, 0, 0, 1, 1,
	1, 1, 1, 1, 0, 0, 1, 0,
	1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1,
	1, 0, 1, 1, 1, 0, 1, 1,
	1, 1, 1, 1, 1, 0, 1, 0,
};

static int bma250_read_state(struct bma250_data *bma250)
{
	unsigned char state[64];
	int i;

	if (!bma250 || !bma250->bma250_client)
		return -1;

	for (i = 0; i < 64; i++) {
		if (!bma250_reg_is_state_r[i])
			state[i] = 0x00;
		else if (bma250_smbus_read_byte(
			bma250->bma250_client, i, &state[i]) < 0)
			return -1;
	}

	memcpy(bma250->state, state, 64);

	if (BMA250_GET_STATE_BITSLICE(state, BMA250E_DEEP_SUSPEND)) {
		bma250->mode = BMA250E_MODE_DEEP_SUSPEND;
	} else if(BMA250_GET_STATE_BITSLICE(state, BMA250_EN_SUSPEND)) {
		if (BMA250_GET_STATE_BITSLICE(state, BMA250E_LOW_POWER_MODE))
			bma250->mode = BMA250E_MODE_STANDBY;
		else
			bma250->mode = BMA250_MODE_SUSPEND;
	} else if(BMA250_GET_STATE_BITSLICE(state, BMA250_EN_LOW_POWER)) {
		if (BMA250_GET_STATE_BITSLICE(state, BMA250E_LOW_POWER_MODE))
			bma250->mode = BMA250_MODE_LOWPOWER1;
		else
			bma250->mode = BMA250E_MODE_LOWPOWER2;
	} else {
		bma250->mode = BMA250_MODE_NORMAL;
	}

	return 0;
}

static int bma250_write_state(struct bma250_data *bma250)
{
	int i, error = 0;

	if (!bma250 || !bma250->bma250_client)
		return -1;

	for (i = 0; i < 64; i++) {
		if (!bma250_reg_is_state_w[i])
			continue;

		if (bma250_smbus_write_byte(
			bma250->bma250_client, i, &bma250->state[i]) < 0)
			error++;
	}

	return (error == 0 ? 0 : -1);
}


static enum bma_chip_id bma250_chip_id(struct bma250_data *bma250)
{
	return bma250->state[BMA250_CHIP_ID_REG];
}

static int bma250_mode_is_suspend(enum bma250_mode mode)
{
	switch (mode) {
	case BMA250_MODE_SUSPEND:
	case BMA250E_MODE_STANDBY:
	case BMA250E_MODE_DEEP_SUSPEND:
		return 1;
	default:
		break;
	}
	return 0;
}

static int bma250_is_suspended(struct bma250_data *bma250)
{
	return bma250_mode_is_suspend(bma250->mode);
}

static int bma250_deep_suspend_unsafe(struct bma250_data *bma250, int enable)
{
	unsigned char mode_ctrl;

	if (!bma250 || (enable != !!enable))
		return -1;

	if (bma250_chip_id(bma250) <= BMA_CHIP_ID_250) {
		printk(KERN_WARNING "BMA device doesn't support deep suspend.\n");
		return -1;
	}

	mode_ctrl = bma250->state[BMA250_MODE_CTRL_REG];

	if (enable == BMA250_GET_BITSLICE(mode_ctrl, BMA250E_DEEP_SUSPEND))
		return 0;

	mode_ctrl = BMA250_SET_BITSLICE(mode_ctrl, BMA250E_DEEP_SUSPEND, enable);
	mode_ctrl = BMA250_SET_BITSLICE(mode_ctrl, BMA250_EN_LOW_POWER, 0);
	mode_ctrl = BMA250_SET_BITSLICE(mode_ctrl, BMA250_EN_SUSPEND, 0);

	if (bma250_smbus_write_byte(bma250->bma250_client,
		BMA250_MODE_CTRL_REG, &mode_ctrl) < 0)
		return -1;

	if (enable && !bma250_is_suspended(bma250))
		cancel_delayed_work_sync(&bma250->work);

	bma250->state[BMA250_MODE_CTRL_REG] = mode_ctrl;
	bma250->mode = (enable ? BMA250E_MODE_DEEP_SUSPEND : BMA250_MODE_NORMAL);

	if (!enable) {
		bma250_write_state(bma250);
		schedule_delayed_work(&bma250->work,
			msecs_to_jiffies(atomic_read(&bma250->delay)));
	}

	return 0;
}

static int bma250_deep_suspend(struct bma250_data *bma250, int enable)
{
	int ret;

	if (!bma250)
		return -1;

	mutex_lock(&bma250->device_mutex);
	ret = bma250_deep_suspend_unsafe(bma250, enable);
	mutex_unlock(&bma250->device_mutex);

	return ret;
}

static int bma250_set_mode_unsafe(struct bma250_data *bma250, enum bma250_mode mode)
{
	int mode_dirty = 0;
	unsigned char mode_ctrl;
	unsigned char low_noise_ctrl;
	int was_suspended, is_suspended;

	if (!bma250 || (mode >= BMA250_MODE_COUNT))
		return -1;

	if (bma250->mode == mode)
		return 0;

	if (bma250_chip_id(bma250) <= BMA_CHIP_ID_250) {
		switch (mode) {
		case BMA250E_MODE_LOWPOWER2:
		case BMA250E_MODE_STANDBY:
		case BMA250E_MODE_DEEP_SUSPEND:
			printk(KERN_WARNING "Attempting to switch to unsupported mode.\n");
			return -1;
		default:
			break;
		}
	}

	if (mode == BMA250E_MODE_DEEP_SUSPEND)
		return bma250_deep_suspend_unsafe(bma250, 1);

	if ((bma250->mode == BMA250E_MODE_DEEP_SUSPEND)
		&& (bma250_deep_suspend_unsafe(bma250, 0) < 0))
		return -1;

	was_suspended = bma250_is_suspended(bma250);
	is_suspended  = bma250_mode_is_suspend(mode);

	mode_ctrl      = bma250->state[BMA250_MODE_CTRL_REG];
	low_noise_ctrl = bma250->state[BMA250_LOW_NOISE_CTRL_REG];

	switch (mode) {
	case BMA250_MODE_LOWPOWER1:
	case BMA250E_MODE_LOWPOWER2:
		mode_ctrl = BMA250_SET_BITSLICE(mode_ctrl, BMA250_EN_LOW_POWER, 1);
		break;
	default:
		mode_ctrl = BMA250_SET_BITSLICE(mode_ctrl, BMA250_EN_LOW_POWER, 0);
		break;
	}

	switch (mode) {
	case BMA250_MODE_SUSPEND:
	case BMA250E_MODE_STANDBY:
		mode_ctrl = BMA250_SET_BITSLICE(mode_ctrl, BMA250_EN_SUSPEND, 1);
		break;
	default:
		mode_ctrl = BMA250_SET_BITSLICE(mode_ctrl, BMA250_EN_SUSPEND, 0);
		break;
	}

	switch (mode) {
	case BMA250E_MODE_LOWPOWER2:
	case BMA250E_MODE_STANDBY:
		low_noise_ctrl = BMA250_SET_BITSLICE(low_noise_ctrl, BMA250E_LOW_POWER_MODE, 1);
		break;
	default:
		low_noise_ctrl = BMA250_SET_BITSLICE(low_noise_ctrl, BMA250E_LOW_POWER_MODE, 0);
		break;
	}

	if (low_noise_ctrl != bma250->state[BMA250_LOW_NOISE_CTRL_REG]) {
		if (bma250->mode != BMA250_MODE_NORMAL) {
			if (bma250_set_mode_unsafe(bma250, BMA250_MODE_NORMAL) < 0)
				return -1;
			mode_dirty = 1;
		}

		if (bma250_smbus_write_byte(bma250->bma250_client,
			BMA250_LOW_NOISE_CTRL_REG, &low_noise_ctrl) < 0) {
			if (mode_dirty)
				printk(KERN_WARNING "BMA250 mode set failed, switched to NORMAL mode.\n");
			return -1;
		}

		bma250->state[BMA250_LOW_NOISE_CTRL_REG] = low_noise_ctrl;
	}

	if (mode_ctrl != bma250->state[BMA250_MODE_CTRL_REG]) {
		if (bma250_smbus_write_byte(bma250->bma250_client,
			BMA250_MODE_CTRL_REG, &mode_ctrl) < 0) {
			if (mode_dirty)
				printk(KERN_WARNING "BMA250 mode set failed, switched to NORMAL mode.\n");
			return -1;
		}

		bma250->state[BMA250_MODE_CTRL_REG] = mode_ctrl;
		bma250->mode = mode;
	}

	if (was_suspended != is_suspended) {
		if (is_suspended) {
			cancel_delayed_work_sync(&bma250->work);
		} else {
			schedule_delayed_work(&bma250->work,
				msecs_to_jiffies(atomic_read(&bma250->delay)));
		}
	}

	return 0;
}

static int bma250_set_mode(struct bma250_data *bma250, enum bma250_mode mode)
{
	int ret;

	if (!bma250)
		return -1;

	mutex_lock(&bma250->device_mutex);
	ret = bma250_set_mode_unsafe(bma250, mode);
	mutex_unlock(&bma250->device_mutex);

	return ret;
}


static int bma250_set_reg(struct bma250_data *bma250,
	unsigned int reg, unsigned int pos, unsigned int len, unsigned char val)
{
	unsigned char mask;
	unsigned char regval;

	if (!bma250 || !bma250->bma250_client)
		return -1;

	mask = (1U << len) - 1;
	if ((val & mask) != val)
		return -1;

	mutex_lock(&bma250->device_mutex);

	regval = bma250->state[reg];
	regval = (regval & (0xFF ^ (mask << pos))) | (val << pos);

	if ((regval != bma250->state[reg])
		&& (bma250_smbus_write_byte(bma250->bma250_client, reg, &regval) < 0)) {
		mutex_unlock(&bma250->device_mutex);
		return -1;
	}

	bma250->state[reg] = regval;
	mutex_unlock(&bma250->device_mutex);
	return 0;
}

#define BMA250_SET_REG(device, bitname, val)\
	bma250_set_reg(device, bitname##__REG, bitname##__POS, bitname##__LEN, val)


static unsigned char bma250_usecs_to_sleep_duration(unsigned int usecs)
{
	unsigned char i;
	for (i = 0; i < 15; i++) {
		if (usecs <= bma250_sleep_dur_value[i])
			break;
	}
	return i;
}

static int bma250_set_sleep_dur(struct bma250_data *bma250, unsigned int usecs)
{
	return BMA250_SET_REG(bma250, BMA250_SLEEP_DUR,
		bma250_usecs_to_sleep_duration(usecs));
}

static int bma250_set_range(struct bma250_data *bma250, enum bma250_range range)
{
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

	return BMA250_SET_REG(bma250, BMA250_RANGE_SEL, range);
}

static int bma250_read_accel_xyz(struct bma250_data *bma250, struct bma250_acc *acc)
{
	unsigned char data[6];

	if (!bma250)
		return -1;

	mutex_lock(&bma250->device_mutex);

	if (i2c_smbus_read_i2c_block_data(bma250->bma250_client,
		BMA250_ACC_X_LSB__REG, 6, data) < 0) {
		mutex_unlock(&bma250->device_mutex);
		return -1;
	}

	acc->x = BMA250_GET_BITSLICE(data[0], BMA250_ACC_X_LSB)
		| (BMA250_GET_BITSLICE(data[1], BMA250_ACC_X_MSB) << BMA250_ACC_X_LSB__LEN);

	acc->y = BMA250_GET_BITSLICE(data[2], BMA250_ACC_Y_LSB)
		| (BMA250_GET_BITSLICE(data[3], BMA250_ACC_Y_MSB) << BMA250_ACC_Y_LSB__LEN);

	acc->z = BMA250_GET_BITSLICE(data[4], BMA250_ACC_Z_LSB)
		| (BMA250_GET_BITSLICE(data[5], BMA250_ACC_Z_MSB) << BMA250_ACC_Z_LSB__LEN);

	mutex_unlock(&bma250->device_mutex);
	return 0;
}

static void bma250_work_func(struct work_struct *work)
{
	struct bma250_data *bma250 = container_of((struct delayed_work *)work,
			struct bma250_data, work);
	static struct bma250_acc acc;
	unsigned long delay = msecs_to_jiffies(atomic_read(&bma250->delay));

	if (!bma250_is_suspended(bma250)) {
		bma250_read_accel_xyz(bma250, &acc);
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
	}

	schedule_delayed_work(&bma250->work, delay);
}

static ssize_t bma250_chip_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i;
	const char* chip_name = "unknown";
	unsigned char chip_id;

	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	chip_id = bma250_chip_id(bma250);

	for (i = 0; chip_id_value[i]; i++) {
		if (chip_id_value[i] == chip_id) {
			chip_name = chip_id_name[i];
			break;
		}
	}

	dprintk(DEBUG_CONTROL_INFO, "%s (0x%02X), %s\n", chip_name, chip_id, __FUNCTION__);
	return sprintf(buf, "%s (0x%02X)\n", chip_name, chip_id);
}

static ssize_t bma250_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	enum bma250_range range;

	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	range = BMA250_GET_STATE_BITSLICE(bma250->state, BMA250_RANGE_SEL);
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

	bw_sel = BMA250_GET_STATE_BITSLICE(bma250->state, BMA250_BW);

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
	if (BMA250_SET_REG(bma250, BMA250_BW, (unsigned char) bw) < 0)
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

	bw_sel = BMA250_GET_STATE_BITSLICE(bma250->state, BMA250_BW);
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

	bw_sel = BMA250_GET_STATE_BITSLICE(bma250->state, BMA250_BW);
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

static ssize_t bma250_sleep_dur_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char sleep_dur;
	unsigned int sleep_usecs;

	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	sleep_dur = BMA250_GET_STATE_BITSLICE(bma250->state, BMA250_SLEEP_DUR);
	sleep_usecs = bma250_sleep_dur_value[sleep_dur];

	return sprintf(buf, "%u\n", sleep_usecs);
}

static ssize_t bma250_sleep_dur_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long sleep_usecs;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &sleep_usecs);

	if (error)
		return error;

	if (bma250_set_sleep_dur(bma250, sleep_usecs) < 0)
		return -EINVAL;

	return count;
}

static enum bma250_latch_int bma250_usecs_to_latch_int(unsigned long usecs)
{
	unsigned long times[] = {
		8000000,
		4000000,
		2000000,
		1000000,
		 500000,
		 250000,
		  50000,
		  25000,
		  12500,
		   1000,
		    500,
		    250,
		      0,
	};

	enum bma250_latch_int values[] = {
		BMA250_LATCH_INT_TEMPORARY_8S,
		BMA250_LATCH_INT_TEMPORARY_4S,
		BMA250_LATCH_INT_TEMPORARY_2S,
		BMA250_LATCH_INT_TEMPORARY_1S,
		BMA250_LATCH_INT_TEMPORARY_500MS,
		BMA250_LATCH_INT_TEMPORARY_250MS,
		BMA250_LATCH_INT_TEMPORARY_50MS,
		BMA250_LATCH_INT_TEMPORARY_25MS,
		BMA250_LATCH_INT_TEMPORARY_12_5MS,
		BMA250_LATCH_INT_TEMPORARY_1MS,
		BMA250_LATCH_INT_TEMPORARY_500US,
		BMA250_LATCH_INT_TEMPORARY_250US,
	};

	unsigned char i;

	if (usecs == 0)
		return BMA250_LATCH_INT_NON_LATCHED;

	for (i = 0; times[i] != 0; i++) {
		if (usecs < times[i])
			return values[i];
	}

	return BMA250_LATCH_INT_COUNT;
}

static ssize_t bma250_latch_int_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char latch_int;

	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	latch_int = BMA250_GET_STATE_BITSLICE(bma250->state, BMA250_LATCH_INT);

	if (bma250_latch_int_name[latch_int])
		return sprintf(buf, "%s\n", bma250_latch_int_name[latch_int]);
	return sprintf(buf, "%u\n", bma250_latch_int_time[latch_int]);
}

static ssize_t bma250_latch_int_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long latch_usecs;
	enum bma250_latch_int latch_int;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (strcmp(buf, "non-latched") == 0) {
		latch_int = BMA250_LATCH_INT_NON_LATCHED;
	} else if (strcmp(buf, "latched") == 0) {
		latch_int = BMA250_LATCH_INT_LATCHED;
	} else {
		error = strict_strtoul(buf, 10, &latch_usecs);
		if (error)
			return error;
		latch_int = bma250_usecs_to_latch_int(latch_usecs);
	}

	if (BMA250_SET_REG(bma250, BMA250_LATCH_INT, latch_int) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_slope_th_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int threshold;

	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	threshold = BMA250_GET_STATE_BITSLICE(bma250->state, BMA250_SLOPE_TH);

	return sprintf(buf, "%u\n", threshold);
}

static ssize_t bma250_slope_th_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long threshold;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &threshold);
	if (error)
		return error;

	if (BMA250_SET_REG(bma250, BMA250_SLOPE_TH, threshold) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_slope_dur_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int duration;

	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	duration = BMA250_GET_STATE_BITSLICE(bma250->state, BMA250_SLOPE_DUR) + 1;

	return sprintf(buf, "%u\n", duration);
}

static ssize_t bma250_slope_dur_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long duration;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &duration);
	if (error)
		return error;

	if (duration <= 0)
		return -EINVAL;

	if (BMA250_SET_REG(bma250, BMA250_SLOPE_DUR, (duration - 1)) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_int_src_slope_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int src;

	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	src = BMA250_GET_STATE_BITSLICE(bma250->state, BMA250_INT_SRC_SLOPE);

	return sprintf(buf, "%u\n", src);
}

static ssize_t bma250_int_src_slope_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long src;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &src);
	if (error)
		return error;

	if (BMA250_SET_REG(bma250, BMA250_INT_SRC_SLOPE, src) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_slope_en_x_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int enable;

	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	enable = BMA250_GET_STATE_BITSLICE(bma250->state, BMA250_SLOPE_EN_X);

	return sprintf(buf, "%u\n", enable);
}

static ssize_t bma250_slope_en_x_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long enable;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &enable);
	if (error)
		return error;

	if (BMA250_SET_REG(bma250, BMA250_SLOPE_EN_X, enable) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_slope_en_y_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int enable;

	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	enable = BMA250_GET_STATE_BITSLICE(bma250->state, BMA250_SLOPE_EN_Y);

	return sprintf(buf, "%u\n", enable);
}

static ssize_t bma250_slope_en_y_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long enable;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &enable);
	if (error)
		return error;

	if (BMA250_SET_REG(bma250, BMA250_SLOPE_EN_Y, enable) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_slope_en_z_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int enable;

	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	enable = BMA250_GET_STATE_BITSLICE(bma250->state, BMA250_SLOPE_EN_Z);

	return sprintf(buf, "%u\n", enable);
}

static ssize_t bma250_slope_en_z_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long enable;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &enable);
	if (error)
		return error;

	if (BMA250_SET_REG(bma250, BMA250_SLOPE_EN_Z, enable) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_slope_int_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int enable;

	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	enable = BMA250_GET_STATE_BITSLICE(bma250->state, BMA250_SLOPE_INT);

	return sprintf(buf, "%u\n", enable);
}


static ssize_t bma250_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma250_data *bma250 = input_get_drvdata(input);
	struct bma250_acc acc_value;

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
		atomic_set(&bma250->enable, 1);
	} else if (!enable && pre_enable) {
		printk("bma250_set_enable pre_enable\n");
/*
		bma250_set_mode(bma250,
						BMA250_MODE_SUSPEND);
		atomic_set(&bma250->enable, 0);
*/
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
static DEVICE_ATTR(sleep_dur, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_sleep_dur_show, bma250_sleep_dur_store);
static DEVICE_ATTR(latch_int, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_latch_int_show, bma250_latch_int_store);
static DEVICE_ATTR(slope_th, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_slope_th_show, bma250_slope_th_store);
static DEVICE_ATTR(slope_dur, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_slope_dur_show, bma250_slope_dur_store);
static DEVICE_ATTR(int_src_slope, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_int_src_slope_show, bma250_int_src_slope_store);
static DEVICE_ATTR(slope_en_x, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_slope_en_x_show, bma250_slope_en_x_store);
static DEVICE_ATTR(slope_en_y, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_slope_en_y_show, bma250_slope_en_y_store);
static DEVICE_ATTR(slope_en_z, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_slope_en_z_show, bma250_slope_en_z_store);
static DEVICE_ATTR(slope_int, S_IRUGO,
		bma250_slope_int_show, NULL);
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
	&dev_attr_sleep_dur.attr,
	&dev_attr_latch_int.attr,
	&dev_attr_slope_th.attr,
	&dev_attr_slope_dur.attr,
	&dev_attr_int_src_slope.attr,
	&dev_attr_slope_en_x.attr,
	&dev_attr_slope_en_y.attr,
	&dev_attr_slope_en_z.attr,
	&dev_attr_slope_int.attr,
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

	mutex_init(&data->device_mutex);
	mutex_init(&data->value_mutex);
	mutex_init(&data->enable_mutex);

	bma250_read_state(data);

	bma250_set_mode(data, BMA250_MODE_SET);
	bma250_set_sleep_dur(data, BMA250_SLEEP_DUR_SET);
	BMA250_SET_REG(data, BMA250_BW, BMA250_BW_SET);
	bma250_set_range(data, BMA250_RANGE_SET);

	INIT_DELAYED_WORK(&data->work, bma250_work_func);
	dprintk(DEBUG_INIT, "bma: INIT_DELAYED_WORK\n");
	atomic_set(&data->delay, BMA250_MAX_DELAY);
	schedule_delayed_work(&data->work,
			msecs_to_jiffies(atomic_read(&data->delay)));

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
		if (atomic_read(&data->enable) == 1)
			bma250_set_mode(data, BMA250_MODE_SUSPEND);
		mutex_unlock(&data->enable_mutex);
*/
	} else if (SUPER_STANDBY == standby_type) {
		mutex_lock(&data->enable_mutex);
		if (atomic_read(&data->enable) == 1)
			bma250_set_mode(data, BMA250_MODE_SUSPEND);
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
		if (atomic_read(&data->enable) == 1)
			bma250_set_mode(data, BMA250_MODE_NORMAL);
		mutex_unlock(&data->enable_mutex);
*/
	} else if (SUPER_STANDBY == standby_type) {
		printk("bma250_late_resume\n");
		if (bma250_smbus_write_byte(data->bma250_client,
			BMA250_BW_SEL_REG, &data->state[BMA250_BW_SEL_REG]) < 0)
			printk("suspend: write bandwidth err\n");

		if (bma250_smbus_write_byte(data->bma250_client,
			BMA250_RANGE_SEL_REG, &data->state[BMA250_RANGE_SEL_REG]) < 0)
			printk("suspend: write range err\n");

		mutex_lock(&data->enable_mutex);
		if (atomic_read(&data->enable) == 1)
			bma250_set_mode(data, BMA250_MODE_NORMAL);
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
		
		if (atomic_read(&data->enable) == 1)
			bma250_set_mode(data, BMA250_MODE_NORMAL);

		mutex_unlock(&data->enable_mutex);
	} else if (SUPER_STANDBY == standby_type) {
		if (bma250_smbus_write_byte(data->bma250_client,
			BMA250_BW_SEL_REG, &data->state[BMA250_BW_SEL_REG]) < 0)
			printk("suspend: write bandwidth err\n");

		if (bma250_smbus_write_byte(data->bma250_client,
			BMA250_RANGE_SEL_REG, &data->state[BMA250_RANGE_SEL_REG]) < 0)
			printk("suspend: write range err\n");

		mutex_lock(&data->enable_mutex);
		if (atomic_read(&data->enable) == 1)
			bma250_set_mode(data, BMA250_MODE_NORMAL);
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

		if (atomic_read(&data->enable) == 1)
			bma250_set_mode(data, BMA250_MODE_SUSPEND);

		mutex_unlock(&data->enable_mutex);
	} else if (SUPER_STANDBY == standby_type) {
		mutex_lock(&data->enable_mutex);
		if (atomic_read(&data->enable) == 1)
			bma250_set_mode(data, BMA250_MODE_SUSPEND);
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
