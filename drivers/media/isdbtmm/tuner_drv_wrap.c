/*
*
* drivers/media/isdbtmm/tuner_drv_wrap.c
*
* MM Tuner Driver
*
* Copyright (C) (2013, Samsung Electronics)
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation version 2.
*
* This program is distributed "as is" WITHOUT ANY WARRANTY of any
* kind, whether express or implied; without even the implied warranty
* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

/******************************************************************************
 * include
 ******************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include "tuner_drv.h"
#include <asm/system_info.h>
#include <linux/barcode_emul.h>
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include <mach/isdbtmm_pdata.h>

#include <linux/regulator/consumer.h>

#ifdef TUNER_CONFIG_IRQ_PC_LINUX
#include "../../../i2c-parport-x/i2c-parport.h"
#endif  /* TUNER_CONFIG_IRQ_PC_LINUX */
 
#include "tuner.h"

extern struct isdbtmm_platform_data *isdbtmm_pdata;


/******************************************************************************/
/* function                                                                   */
/******************************************************************************/
int tuner_drv_ctl_power( int data );
int tuner_drv_set_interrupt( void );
void tuner_drv_release_interrupt( void );
void tuner_drv_gpio_config_poweron( void );
void tuner_drv_gpio_config_poweroff( void );



/******************************************************************************
 * code area
 ******************************************************************************/
/******************************************************************************
 *    function:   tuner_drv_ctl_power
 *    brief   :   power control of a driver
 *    date    :   2011.08.26
 *    author  :   M.Takahashi(*)
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   data                 setting data
 *    output  :   none
 ******************************************************************************/
int tuner_drv_ctl_power( int data )
{	
	/* power on */
	if( data == TUNER_DRV_CTL_POWON )
	{
		DEBUG_PRINT("tuner_drv_ctl_power poweron");
		
		/* poweron gpio config setting */
		tuner_drv_gpio_config_poweron();

		/* TMM_PWR_EN high */
		gpio_set_value(isdbtmm_pdata->gpio_en, 1);
		/* 15ms sleep */
		usleep_range(15000, 15000);
		
		/* TMM_RST high */
		gpio_set_value(isdbtmm_pdata->gpio_rst, 1);
		
		/* 2ms sleep */
		usleep_range(2000, 2000);
	}
	/* power off */
	else
	{
		DEBUG_PRINT("tuner_drv_ctl_power poweroff");
		
		/* poweroff gpio config setting */
		tuner_drv_gpio_config_poweroff();
		
		/* 1ms sleep */
		usleep_range(1000, 1000);
	
		/* TMM_RST low */
		gpio_set_value(isdbtmm_pdata->gpio_rst, 0);
		
		/* 2ms sleep */
		usleep_range(2000, 2000);
		
		/* TMM_PWR_EN low */
		gpio_set_value(isdbtmm_pdata->gpio_en, 0);
		
	}
	return 0;
}

/******************************************************************************
 *    function:   tuner_drv_set_interrupt
 *    brief   :   interruption registration control of a driver
 *    date    :   2011.08.26
 *    author  :   M.Takahashi(*)
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   pdev
 *    output  :   none
 ******************************************************************************/
int tuner_drv_set_interrupt( void )
{
#ifndef TUNER_CONFIG_IRQ_PC_LINUX
    int ret;
	
	ret = request_irq( isdbtmm_pdata->gpio_int,
					   tuner_interrupt,
					   IRQF_DISABLED | IRQF_TRIGGER_RISING,
					   "mm_tuner",
					   NULL );

    if( ret != 0 )
    {
        return -1;
    }
#else  /* TUNER_CONFIG_IRQ_PC_LINUX */
    i2c_set_interrupt( tuner_interrupt );
#endif /* TUNER_CONFIG_IRQ_PC_LINUX */
    return 0;
}

/******************************************************************************
 *    function:   tuner_drv_release_interrupt
 *    brief   :   interruption registration release control of a driver
 *    date    :   2011.08.26
 *    author  :   M.Takahashi(*)
 *
 *    return  :   none
 *    input   :   none
 *    output  :   none
 ******************************************************************************/
void tuner_drv_release_interrupt( void )
{
#ifndef TUNER_CONFIG_IRQ_PC_LINUX
	free_irq( isdbtmm_pdata->gpio_int, NULL );
#else  /* TUNER_CONFIG_IRQ_PC_LINUX */
    i2c_release_interrupt( NULL );
#endif /* TUNER_CONFIG_IRQ_PC_LINUX */
}

#ifdef TUNER_CONFIG_IRQ_LEVELTRIGGER
/******************************************************************************
 *    function:   tuner_drv_enable_interrupt
 *    brief   :   interruption registration enable control of a driver
 *    date    :   2011.09.18
 *    author  :   M.Takahashi(*)(*)
 *
 *    return  :   none
 *    input   :   none
 *    output  :   none
 ******************************************************************************/
void tuner_drv_enable_interrupt( void )
{
#ifndef TUNER_CONFIG_IRQ_PC_LINUX
	enable_irq( isdbtmm_pdata->gpio_int);
#else  /* TUNER_CONFIG_IRQ_PC_LINUX */
    i2c_set_interrupt( tuner_interrupt );
#endif /* TUNER_CONFIG_IRQ_PC_LINUX */
	return;
}

/******************************************************************************
 *    function:   tuner_drv_disable_interrupt
 *    brief   :   interruption registration disable control of a driver
 *    date    :   2011.09.18
 *    author  :   M.Takahashi(*)(*)
 *
 *    return  :   none
 *    input   :   none
 *    output  :   none
 ******************************************************************************/
void tuner_drv_disable_interrupt( void )
{
#ifndef TUNER_CONFIG_IRQ_PC_LINUX
	disable_irq( isdbtmm_pdata->gpio_int);
#else  /* TUNER_CONFIG_IRQ_PC_LINUX */
    i2c_release_interrupt( NULL );
#endif /* TUNER_CONFIG_IRQ_PC_LINUX */
	return;
}
#endif /* TUNER_CONFIG_IRQ_LEVELTRIGGER */

/******************************************************************************
 *    function:   tuner_drv_gpio_config_poweron
 *    brief   :   interruption registration disable control of a driver
 *    date    :   2012.12.18
 *    author  :   K.Matsumaru(*)(*)
 *
 *    return  :   none
 *    input   :   none
 *    output  :   none
 ******************************************************************************/
void tuner_drv_gpio_config_poweron( void )
{
	
	gpio_tlmm_config(GPIO_CFG(isdbtmm_pdata->gpio_spi_di, GPIOMUX_FUNC_1,
					 GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
					 GPIO_CFG_ENABLE);
	
	gpio_tlmm_config(GPIO_CFG(isdbtmm_pdata->gpio_spi_do, GPIOMUX_FUNC_1,
					 GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
					 GPIO_CFG_ENABLE);
	
	gpio_tlmm_config(GPIO_CFG(isdbtmm_pdata->gpio_spi_cs, GPIOMUX_FUNC_1,
					 GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
					 GPIO_CFG_ENABLE);
	
	gpio_tlmm_config(GPIO_CFG(isdbtmm_pdata->gpio_spi_clk, GPIOMUX_FUNC_1,
					 GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
					 GPIO_CFG_ENABLE);
	
	gpio_tlmm_config(GPIO_CFG(isdbtmm_pdata->gpio_int, GPIOMUX_FUNC_1,
					 GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
					 GPIO_CFG_ENABLE);

	gpio_tlmm_config(GPIO_CFG(isdbtmm_pdata->gpio_en, GPIOMUX_FUNC_GPIO,
					 GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
					 GPIO_CFG_ENABLE);
		
	gpio_tlmm_config(GPIO_CFG(isdbtmm_pdata->gpio_rst, GPIOMUX_FUNC_GPIO,
					 GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
					 GPIO_CFG_ENABLE);
					 
	gpio_tlmm_config(GPIO_CFG(isdbtmm_pdata->gpio_i2c_sda, GPIOMUX_FUNC_GPIO,
					 GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
					 GPIO_CFG_ENABLE);		
	
	gpio_tlmm_config(GPIO_CFG(isdbtmm_pdata->gpio_i2c_scl, GPIOMUX_FUNC_GPIO,
					 GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
					 GPIO_CFG_ENABLE);		
#if defined(CONFIG_TMM_CHG_CTRL)
	tmm_chg_ctrl_start();

	tmm_chg_log(KERN_ALERT"%s: tmm charging control start!\n", __func__);
#endif
	
	return;
}

/******************************************************************************
 *    function:   tuner_drv_gpio_config_poweroff
 *    brief   :   interruption registration disable control of a driver
 *    date    :   2012.12.18
 *    author  :   K.Matsumaru(*)(*)
 *
 *    return  :   none
 *    input   :   none
 *    output  :   none
 ******************************************************************************/
void tuner_drv_gpio_config_poweroff( void )
{
	gpio_tlmm_config(GPIO_CFG(isdbtmm_pdata->gpio_spi_di, GPIOMUX_FUNC_GPIO,
					 GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
					 GPIO_CFG_ENABLE);
	
	gpio_tlmm_config(GPIO_CFG(isdbtmm_pdata->gpio_spi_do, GPIOMUX_FUNC_GPIO,
					 GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
					 GPIO_CFG_ENABLE);
	
	gpio_tlmm_config(GPIO_CFG(isdbtmm_pdata->gpio_spi_cs, GPIOMUX_FUNC_GPIO,
					 GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
					 GPIO_CFG_ENABLE);
	
	gpio_tlmm_config(GPIO_CFG(isdbtmm_pdata->gpio_spi_clk, GPIOMUX_FUNC_GPIO,
					 GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
					 GPIO_CFG_ENABLE);
	
	gpio_tlmm_config(GPIO_CFG(isdbtmm_pdata->gpio_int, GPIOMUX_FUNC_GPIO,
					 GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
					 GPIO_CFG_ENABLE);

	gpio_tlmm_config(GPIO_CFG(isdbtmm_pdata->gpio_en, GPIOMUX_FUNC_GPIO,
					 GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
					 GPIO_CFG_ENABLE);
		
	gpio_tlmm_config(GPIO_CFG(isdbtmm_pdata->gpio_rst, GPIOMUX_FUNC_GPIO,
					 GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
					 GPIO_CFG_ENABLE);
#if defined(CONFIG_TMM_CHG_CTRL)
	tmm_chg_ctrl_stop();

	tmm_chg_log(KERN_ALERT"%s: tmm charging control stop!\n", __func__);
#endif
	
	return;
}

