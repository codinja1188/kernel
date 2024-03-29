/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/hardware.h>
#include <mach/device.h>
#include <mach/pinctrl.h>
#include <mach/regs-ocotp.h>
#include <linux/spi/spi_gpio.h>

#include "device.h"
#include "mx23evk.h"
#include "mx23_pins.h"
#include <linux/spi/fpc1080.h>
#include <linux/input/msg2133_touch.h>
#ifdef __hcit__
static struct mxs_mma7450_platform_data mma7450_platdata = {
	.reg_dvdd_io = "vddio",
	.reg_avdd = "vdda",
	.gpio_pin_get = mx23evk_mma7450_pin_init,
	.gpio_pin_put = mx23evk_mma7450_pin_release,
	/* int1 and int2 will be initialized in
	i2c_device_init */
	.int1 = 0,
	.int2 = 0,
};

static struct i2c_board_info __initdata mma7450_i2c_device = {
	I2C_BOARD_INFO("mma7450", 0x3A),
	.platform_data = (void *)&mma7450_platdata,
};

static void i2c_device_init(void)
{
	mma7450_platdata.int1 = gpio_to_irq(MXS_PIN_TO_GPIO(PINID_GPMI_D14));
	mma7450_platdata.int2 = gpio_to_irq(MXS_PIN_TO_GPIO(PINID_GPMI_D15));
	i2c_register_board_info(0, &mma7450_i2c_device, 1);
}

#endif
static struct msg2133_platform_data msg2133_platdata = {
        .irq_active_high        = 0,
        .reset_gpio             = 0,
        .irq_gpio               = 0,
        .gpio_pin_get           = msg2133_pin_init,
        .gpio_pin_put           = msg2133_pin_release,
};
static struct i2c_board_info __initdata msg2133_i2c_device = {
        I2C_BOARD_INFO("msg2133", 0x60),
        .platform_data          =(void *)&msg2133_platdata,
};

static void i2c_device_init(void)
{
//      msg2133_platdata.irq_gpio = gpio_to_irq(MXS_PIN_TO_GPIO(PINID_SSP1_DATA1));
        msg2133_platdata.irq_gpio = MXS_PIN_TO_GPIO(PINID_SSP1_DATA1);
        msg2133_platdata.reset_gpio = MXS_PIN_TO_GPIO(PINID_SSP1_DATA2);
        i2c_register_board_info(0, &msg2133_i2c_device, 1);
}

static struct mxs_spi_platform_data enc_data = {
        .hw_pin_init = mxs_spi_enc_pin_init,
        .hw_pin_release = mxs_spi_enc_pin_release,
};

/********* changes done by vasubabu adding code lines between two comment lines ****/
/*static struct mxs_spi_platform_data fpc1080_pdata = {
	.hw_pin_init = mxs_spi_fpc1080_pin_init,
	.hw_pin_release =  mxs_spi_fpc1080_pin_release,

};*/
/********* changes done by vasubabu adding code lines between two comment lines ****/

static struct spi_gpio_platform_data spi_gpio_data = {
       	.sck = 70,
     	.mosi = 64,
     	.miso = 66,
	.num_chipselect = 1,
};
static struct platform_device spi_gpio_device = {
        .name   = "spi_gpio",
        .id     = 0,
        .dev    = {
                .platform_data   = &spi_gpio_data,
        },

};

#define FPC1080_IRQ_GPIO    67
#define FPC1080_RESET_GPIO  68

static struct fpc1080_platform_data fpc1080_pdata = {
        .irq_gpio 	= FPC1080_IRQ_GPIO,
        .reset_gpio 	= FPC1080_RESET_GPIO,
};

static struct mxs_spi_platform_data fpc1080_control_pdata = {
        .hw_pin_init = mxs_spi_fpc1080_pin_init,
        .hw_pin_release =  mxs_spi_fpc1080_pin_release,
};
/* ------------------changes done by hcit -----------*/
/*
static struct spi_board_info spi_board_info[] __initdata = {
        {
                .modalias       	= "fpc1080",
                .max_speed_hz   	= 12 * 1000 * 1000,
                .bus_num        	= 1,
		.irq 			= 0,
                .chip_select    	= 0,
		.mode 			= SPI_MODE_0,
		.platform_data  	= &fpc1080_pdata,
		.controller_data	= &fpc1080_control_pdata
        },
};
*/

/*static struct mxs_spi_platform_data spidev_data = {
	.hw_pin_init = mxs_spi_spidev_pin_init,
	.hw_pin_release = mxs_spi_spidev_pin_release,
};
static struct spi_board_info spi_board_info[] __initdata = {
        {
                .modalias       = "spidev",
                .max_speed_hz   = 6 * 1000 * 1000,
                .bus_num        = 1,
                .irq            = 0,
                .chip_select    = 0,
                .mode           = SPI_MODE_0,
                .platform_data  = NULL,
        },
};
*/


static struct spi_board_info spi_board_info[] __initdata = {
	{
                .modalias       	= "fpc1080",
                .bus_num        	= 0,
                .chip_select    	= 0,
                .max_speed_hz   	= 12 * 1000 * 1000,
                .irq            	= 0,
         	.mode           	= SPI_MODE_0,
                .controller_data 	= (void *) 69,       
		.platform_data  	= &fpc1080_pdata,
        }
};

/*static struct spi_board_info spi_board_info[] __initdata = {
	{
                .modalias       = "spidev",
                .bus_num        = 0,
                .chip_select    = 0,
                .max_speed_hz   = 6 * 1000 * 1000,
                .mode           = SPI_MODE_0,
        //        .irq            = 0,
           //     .controller_data = (void *) 69,        
                .platform_data  = spidev_data,
        },

};*/

/********************* changes done by vasu**************************/
/*static struct spi_board_info spi_board_info[] __initdata = {
#if defined(CONFIG_ENC28J60) || defined(CONFIG_ENC28J60_MODULE)
        {
                .modalias       = "enc28j60",
                .max_speed_hz   = 6 * 1000 * 1000,
                .bus_num        = 1,
                .chip_select    = 0,
//	 	.controller_data = (void *) SPI_GPIO_NO_CHIPSELECT,
		//.controller_data = (void *) 69,
                .platform_data  = &enc_data,
        },
#endif
};*/
/******************* changes done by vasu ***************************/


static void spi_device_init(void)
{
	printk(KERN_ERR "mx23evk spi device init started\n");
	spi_board_info[0].irq = gpio_to_irq(MXS_PIN_TO_GPIO(PINID_SSP1_DATA1));
//	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
/* ------------------changes done by hcit Enter---------------*/
	platform_device_register(&spi_gpio_device);
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
/* ------------------changes done by hcit exit---------------*/
}

static void __init fixup_board(struct machine_desc *desc, struct tag *tags,
			       char **cmdline, struct meminfo *mi)
{
	mx23_set_input_clk(24000000, 24000000, 32000, 50000000);
}

#if defined(CONFIG_SND_MXS_SOC_ADC) || defined(CONFIG_SND_MXS_SOC_ADC_MODULE)
static void __init mx23evk_init_adc(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-adc", 0);
	if (pdev == NULL)
		return;
	mxs_add_device(pdev, 3);
}
#else
static void __init mx23evk_init_adc(void)
{

}
#endif

#define REGS_OCOTP_BASE	IO_ADDRESS(OCOTP_PHYS_ADDR)
int get_evk_board_version()
{
	int boardid;
	boardid = __raw_readl(REGS_OCOTP_BASE + HW_OCOTP_CUSTCAP);
	boardid &= 0x30000000;
	boardid = boardid >> 28;

	return boardid;
}
EXPORT_SYMBOL_GPL(get_evk_board_version);

static void __init mx23evk_device_init(void)
{
	/* Add mx23evk special code */
	printk(KERN_ERR "mx23evk device init called \n");
	i2c_device_init();
	spi_device_init();
	mx23evk_init_adc();
}


static void __init mx23evk_init_machine(void)
{

	printk(KERN_ERR " mx23evk machine init started \n");
	mx23_pinctrl_init();

	/* Init iram allocate */
#ifdef CONFIG_VECTORS_PHY_ADDR
	/* reserve the first page for irq vectors table*/
	iram_init(MX23_OCRAM_PHBASE + PAGE_SIZE, MX23_OCRAM_SIZE - PAGE_SIZE);
#else
	iram_init(MX23_OCRAM_PHBASE, MX23_OCRAM_SIZE);
#endif

	mx23_gpio_init();
	mx23evk_pins_init();
	mx23evk_mma7450_pin_init();
	mx23_device_init();
	mx23evk_device_init();
}

MACHINE_START(MX23EVK, "Freescale MX23EVK board")
	.phys_io	= 0x80000000,
	.io_pg_offst	= ((0xf0000000) >> 18) & 0xfffc,
	.boot_params	= 0x40000100,
	.fixup		= fixup_board,
	.map_io		= mx23_map_io,
	.init_irq	= mx23_irq_init,
	.init_machine	= mx23evk_init_machine,
	.timer		= &mx23_timer.timer,
MACHINE_END
