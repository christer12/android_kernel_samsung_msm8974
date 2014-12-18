
#ifndef _LINUX_WACOM_I2C_H
#define _LINUX_WACOM_I2C_H

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/wakelock.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/* FW UPDATE FEATURE */
#define WACOM_MAX_FW_PATH		64
#define WACOM_FW_NAME_W9001		"epen/W9001_B911.bin"
#define WACOM_FW_NAME_W9010		"epen/W9010_0070.hex"
#define WACOM_FW_NAME_W9007_BL92		"epen/W9007_0104.bin"
#define WACOM_FW_NAME_W9007_BL91		"epen/W9007_0104.bin"
#define WACOM_FW_NAME_NONE 		NULL

#define NAMEBUF 12
#define WACNAME "WAC_I2C_EMR"
#define WACFLASH "WAC_I2C_FLASH"

#define WACOM_FW_SIZE 61440

extern unsigned int system_rev;

/*Wacom Command*/

#define COM_COORD_NUM	7
#define COM_QUERY_NUM	9

#define COM_SAMPLERATE_STOP 0x30
#define COM_SAMPLERATE_40  0x33
#define COM_SAMPLERATE_80  0x32
#define COM_SAMPLERATE_133 0x31
#define COM_SURVEYSCAN     0x2B
#define COM_QUERY          0x2A
#define COM_FLASH          0xff
#define COM_CHECKSUM       0x63

/*I2C address for digitizer and its boot loader*/
#define WACOM_I2C_ADDR 0x56
#define WACOM_I2C_BOOT 0x09

/*Information for input_dev*/
#define EMR 0
#define WACOM_PKGLEN_I2C_EMR 0

/*Enable/disable irq*/
#define ENABLE_IRQ 1
#define DISABLE_IRQ 0

/*Special keys*/
#define EPEN_TOOL_PEN		0x220
#define EPEN_TOOL_RUBBER	0x221
#define EPEN_STYLUS			0x22b
#define EPEN_STYLUS2		0x22c

#define WACOM_DELAY_FOR_RST_RISING 200
/* #define INIT_FIRMWARE_FLASH */

#define WACOM_PDCT_WORK_AROUND/*****************/
#define WACOM_USE_QUERY_DATA

/*PDCT Signal*/
#define PDCT_NOSIGNAL 1
#define PDCT_DETECT_PEN 0

#define WACOM_PRESSURE_MAX 255

/*Digitizer Type*/
#define EPEN_DTYPE_B660	1
#define EPEN_DTYPE_B713 2
#define EPEN_DTYPE_B746 3
#define EPEN_DTYPE_B887 4
#define EPEN_DTYPE_B911 5

#define WACOM_HAVE_FWE_PIN

/* For Android origin */
#if defined(CONFIG_SEC_VIENNA_PROJECT)
#define WACOM_MAX_COORD_X 26266
#define WACOM_MAX_COORD_Y 16416

#define WACOM_POSX_MAX WACOM_MAX_COORD_X
#define WACOM_POSY_MAX WACOM_MAX_COORD_Y
#else
#define WACOM_MAX_COORD_X 12576
#define WACOM_MAX_COORD_Y 7074

#define WACOM_POSX_MAX WACOM_MAX_COORD_Y
#define WACOM_POSY_MAX WACOM_MAX_COORD_X
#endif

#define WACOM_MAX_PRESSURE 1023

#define WACOM_USE_SOFTKEY

#define COOR_WORK_AROUND

#define WACOM_IMPORT_FW_ALGO
/*#define WACOM_USE_OFFSET_TABLE*/
#define WACOM_USE_AVERAGING
#if 0
#define WACOM_USE_AVE_TRANSITION
#define WACOM_USE_BOX_FILTER
#define WACOM_USE_TILT_OFFSET
#define WACOM_USE_HEIGHT
#endif

#define MAX_ROTATION	4
#define MAX_HAND		2

#define WACOM_PEN_DETECT/*****************/

/* origin offset */
#define EPEN_B660_ORG_X 456
#define EPEN_B660_ORG_Y 504

#define EPEN_B713_ORG_X 676
#define EPEN_B713_ORG_Y 724

/*Box Filter Parameters*/
#define  X_INC_S1  1500
#define  X_INC_E1  (WACOM_MAX_COORD_X - 1500)
#define  Y_INC_S1  1500
#define  Y_INC_E1  (WACOM_MAX_COORD_Y - 1500)

#define  Y_INC_S2  500
#define  Y_INC_E2  (WACOM_MAX_COORD_Y - 500)
#define  Y_INC_S3  1100
#define  Y_INC_E3  (WACOM_MAX_COORD_Y - 1100)

#undef CONFIG_SEC_TOUCHSCREEN_DVFS_LOCK
#define WACOM_DVFS_LOCK_FREQ 800000
#define BATTERY_SAVING_MODE/**************/

/*HWID to distinguish Detect Switch*/
#define WACOM_DETECT_SWITCH_HWID 0xFFFF

/*HWID to distinguish FWE1*/
#define WACOM_FWE1_HWID 0xFFFF

/*HWID to distinguish B911 Digitizer*/
#define WACOM_DTYPE_B911_HWID 1

/*End of Model config*/

#ifndef WACOM_X_INVERT
#define WACOM_X_INVERT 1
#endif
#ifndef WACOM_Y_INVERT
#define WACOM_Y_INVERT 0
#endif
#ifndef WACOM_XY_SWITCH
#define WACOM_XY_SWITCH 1
#endif

#ifdef BATTERY_SAVING_MODE
#ifndef WACOM_PEN_DETECT
#define WACOM_PEN_DETECT
#endif
#endif

#ifdef WACOM_USE_PDATA
#undef WACOM_USE_QUERY_DATA
#endif

#define WACOM_COORDS_ARR_SIZE	9

/*Parameters for wacom own features*/
struct wacom_features {
	int x_max;
	int y_max;
	int pressure_max;
	char comstat;
	u8 data[COM_COORD_NUM];
	unsigned int fw_ic_version;
	unsigned int fw_version;
	int firm_update_status;
};

/*sec_class sysfs*/
extern struct class *sec_class;

struct wacom_g5_callbacks {
	int (*check_prox)(struct wacom_g5_callbacks *);
};

/*Parameters for i2c driver*/
struct wacom_i2c {
	struct i2c_client *client;
	struct i2c_client *client_boot;
	struct input_dev *input_dev;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	struct mutex lock;
	struct wake_lock wakelock;
	struct device	*dev;
	int irq;
#ifdef WACOM_PDCT_WORK_AROUND
	int irq_pdct;
	bool rdy_pdct;
#endif
	int pen_pdct;
	int gpio;
	int irq_flag;
	int pen_prox;
	int pen_pressed;
	int side_pressed;
	int tool;
	s16 last_x;
	s16 last_y;
#ifdef WACOM_PEN_DETECT
	struct delayed_work pen_insert_dwork;
	bool pen_insert;
	int gpio_pen_insert;
	int ic_mpu_ver;

#endif
	int invert_pen_insert;
#ifdef WACOM_HAVE_FWE_PIN
	int gpio_fwe;
	bool have_fwe_pin;
#endif
#ifdef WACOM_IMPORT_FW_ALGO
	bool use_offset_table;
	bool use_aveTransition;
#endif
	bool checksum_result;
	const char name[NAMEBUF];
	struct wacom_features *wac_feature;
	struct wacom_g5_platform_data *wac_pdata;
	struct wacom_g5_callbacks callbacks;
	int (*power)(int on);
	struct delayed_work resume_work;
#ifdef CONFIG_SEC_TOUCHSCREEN_DVFS_LOCK
	unsigned int cpufreq_level;
	bool dvfs_lock_status;
	struct delayed_work dvfs_work;
	struct device *bus_dev;
#endif
#ifdef WACOM_CONNECTION_CHECK
	bool connection_check;
#endif
#ifdef BATTERY_SAVING_MODE
	bool battery_saving_mode;
#endif
	bool power_enable;
	bool boot_mode;
	bool query_status;
};

struct wacom_g5_platform_data {
	char *name;
/* using dts feature */
	bool i2c_pull_up;
	int gpio_int;
	u32 irq_gpio_flags;
	int gpio_sda;
	u32 sda_gpio_flags;
	int gpio_scl;
	u32 scl_gpio_flags;
	int gpio_pdct;
	u32 pdct_gpio_flags;
	int vdd_en;
	int gpio_pen_reset_n;
	u32 pen_reset_n_gpio_flags;
	int gpio_pen_fwe1;
	u32 pen_fwe1_gpio_flags;
	int gpio_pen_pdct;
	u32 pen_pdct_gpio_flags;

	int x_invert;
	int y_invert;
	int xy_switch;
	int min_x;
	int max_x;
	int min_y;
	int max_y;
	int max_pressure;
	int min_pressure;
	int gpio_pendct;
/* using dts feature */
#ifdef WACOM_PEN_DETECT
	int gpio_pen_insert;
#endif
#ifdef WACOM_HAVE_FWE_PIN
	void (*compulsory_flash_mode)(struct wacom_i2c *, bool);
#endif
	int (*init_platform_hw)(void);
	int (*exit_platform_hw)(void);
	int (*suspend_platform_hw)(struct wacom_i2c *);
	int (*resume_platform_hw)(struct wacom_i2c *);
	int (*early_suspend_platform_hw)(struct wacom_i2c *);
	int (*late_resume_platform_hw)(struct wacom_i2c *);
	int (*reset_platform_hw)(struct wacom_i2c *);
	void (*register_cb)(struct wacom_g5_callbacks *);
};

#endif /* _LINUX_WACOM_I2C_H */
