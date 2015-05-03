/*
 * linux/arch/arm/mach-exynos4/mach-nanopc.c
 *
 * Copyright (c) 2012 AgreeYa Mobility Co., Ltd.
 *		http://www.agreeyamobility.net
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/i2c/pca953x.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/serial_core.h>
#include <linux/platform_data/s3c-hsotg.h>
#include <linux/platform_data/i2c-s3c2410.h>
#include <linux/platform_data/usb-ehci-s5p.h>
#include <linux/platform_data/usb-exynos.h>
#include <linux/platform_data/usb3503.h>
#include <linux/delay.h>
#include <linux/lcd.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/reboot.h>

#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>

#include <plat/backlight.h>
#include <plat/clock.h>
#include <plat/cpu.h>
#include <plat/devs.h>
#include <plat/gpio-cfg.h>
#include <plat/keypad.h>
#include <plat/mfc.h>
#include <plat/regs-serial.h>
#include <plat/sdhci.h>
#include <plat/fb.h>
#include <plat/hdmi.h>

#include <video/platform_lcd.h>
#include <video/samsung_fimd.h>
#include <linux/platform_data/spi-s3c64xx.h>

#include <mach/gpio.h>
#include <mach/map.h>
#include <mach/regs-pmu.h>
#include <mach/dwmci.h>

#include "common.h"
#include "pmic-77686.h"


/* Audio */
static struct platform_device tiny4412_audio = {
	.name		= "tiny4412-audio",
	.id			= -1,
};


#include <mach/s3cfb.h>
#define CONFIG_FB_S3C_NR_BUFFERS 3
static void __init nanopc_fb_init_pdata(struct s3c_fb_platdata *pd) {
	struct s3cfb_lcd *lcd;
	struct s3c_fb_pd_win *win;
	struct fb_videomode *mode = pd->vtiming;
	unsigned long val = 0;
	u64 pixclk = 1000000000000ULL;
	u32 div;
	int i;

	lcd = tiny4412_get_lcd();

	for (i = 0; i < S3C_FB_MAX_WIN; i++) {
		if (pd->win[i] == NULL)
			continue;

		win = pd->win[i];
		win->xres		= lcd->width;
		win->yres		= lcd->height;
		win->default_bpp= lcd->bpp ? : 24;
		win->virtual_x	= win->xres;
		win->virtual_y	= win->yres * CONFIG_FB_S3C_NR_BUFFERS;
		win->width		= lcd->p_width;
		win->height		= lcd->p_height;
	}

	mode->left_margin	= lcd->timing.h_bp;
	mode->right_margin	= lcd->timing.h_fp;
	mode->upper_margin	= lcd->timing.v_bp;
	mode->lower_margin	= lcd->timing.v_fp;
	mode->hsync_len		= lcd->timing.h_sw;
	mode->vsync_len		= lcd->timing.v_sw;
	mode->xres			= lcd->width;
	mode->yres			= lcd->height;

	/* calculates pixel clock */
	div  = mode->left_margin + mode->hsync_len + mode->right_margin +
		mode->xres;
	div *= mode->upper_margin + mode->vsync_len + mode->lower_margin +
		mode->yres;
	div *= lcd->freq ? : 60;

	do_div(pixclk, div);

	mode->pixclock		= pixclk + 386;

	/* initialize signal polarity of RGB interface */
	if (lcd->polarity.rise_vclk)
		val |= VIDCON1_INV_VCLK;
	if (lcd->polarity.inv_hsync)
		val |= VIDCON1_INV_HSYNC;
	if (lcd->polarity.inv_vsync)
		val |= VIDCON1_INV_VSYNC;
	if (lcd->polarity.inv_vden)
		val |= VIDCON1_INV_VDEN;

	pd->vidcon1 = val;
}

extern void exynos4_setup_dwmci_cfg_gpio(struct platform_device *dev, int width);

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define nanopc_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define nanopc_ULCON_DEFAULT	S3C2410_LCON_CS8

#define nanopc_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg nanopc_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= nanopc_UCON_DEFAULT,
		.ulcon		= nanopc_ULCON_DEFAULT,
		.ufcon		= nanopc_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= nanopc_UCON_DEFAULT,
		.ulcon		= nanopc_ULCON_DEFAULT,
		.ufcon		= nanopc_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= nanopc_UCON_DEFAULT,
		.ulcon		= nanopc_ULCON_DEFAULT,
		.ufcon		= nanopc_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= nanopc_UCON_DEFAULT,
		.ulcon		= nanopc_ULCON_DEFAULT,
		.ufcon		= nanopc_UFCON_DEFAULT,
	},
};

static struct s3c2410_platform_i2c nanopc_i2c0_data __initdata = {
	.flags			= 0,
	.bus_num		= 0,
	.slave_addr		= 0x10,
	.frequency		= 200*1000,
	.sda_delay		= 100,
};

#ifdef CONFIG_SND_SOC_WM8960_TINY4412
#include <sound/wm8960.h>
static struct wm8960_data wm8960_pdata = {
	.capless	= 0,
	.dres		= WM8960_DRES_400R,
};
#endif
static struct i2c_board_info nanopc_i2c_devs0[] __initdata = {
#ifdef CONFIG_SND_SOC_WM8960_TINY4412
		{
			I2C_BOARD_INFO("wm8960", 0x1a),
			.platform_data = &wm8960_pdata,
		},
#endif
};

static struct i2c_board_info nanopc_i2c_devs1[] __initdata = {
};

/* I2C2 bus GPIO-Bitbanging */
#define		GPIO_I2C2_SDA	EXYNOS4_GPA0(6)
#define		GPIO_I2C2_SCL	EXYNOS4_GPA0(7)
static struct 	i2c_gpio_platform_data 	i2c2_gpio_platdata = {
	.sda_pin = GPIO_I2C2_SDA,
	.scl_pin = GPIO_I2C2_SCL,
	.udelay  = 5,
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0
};

static struct 	platform_device 	gpio_device_i2c2 = {
	.name 	= "i2c-gpio",
	.id  	= 2,    // adepter number
	.dev.platform_data = &i2c2_gpio_platdata,
};

static struct i2c_board_info nanopc_i2c_devs2[] __initdata = {

};

static struct i2c_board_info nanopc_i2c_devs3[] __initdata = {
	/* nothing here yet */
};

/* NanoPC-t1 schematics show the DDC of the remote HDMI device connected to
 * I2C7. HDMI specs state that DDC always sits at bus address 0x50. */
static struct i2c_board_info nanopc_i2c_devs7[] __initdata = {
	{
		I2C_BOARD_INFO("s5p_ddc", 0x50),
	},
};

#if defined(CONFIG_ODROID_U2) || defined(CONFIG_NANOPC_T1)
/* for u3 I/O shield board */
#define		GPIO_I2C4_SDA	EXYNOS4_GPX1(1)
#define		GPIO_I2C4_SCL	EXYNOS4_GPX1(0)

static struct 	i2c_gpio_platform_data 	i2c4_gpio_platdata = {
	.sda_pin = GPIO_I2C4_SDA,
	.scl_pin = GPIO_I2C4_SCL,
	.udelay  = 0,
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0
};

static struct 	platform_device 	gpio_device_i2c4 = {
	.name 	= "i2c-gpio",
	.id  	= 4,    // adepter number
	.dev.platform_data = &i2c4_gpio_platdata,
};

static struct i2c_board_info nanopc_i2c_devs4[] __initdata = {
};
#endif

static struct gpio_led nanopc_gpio_leds[] = {
	{
		.name		= "led1",	
		.default_trigger	= "heartbeat",	
		.gpio		= EXYNOS4X12_GPM4(0),
		.active_low	= 1,
	},
	{
		.name		= "led2",	
		.default_trigger	= "heartbeat",
		.gpio		= EXYNOS4X12_GPM4(1),
		.active_low	= 1,
	},
};

static struct gpio_led_platform_data nanopc_gpio_led_info = {
	.leds		= nanopc_gpio_leds,
	.num_leds	= ARRAY_SIZE(nanopc_gpio_leds),
};

static struct platform_device nanopc_leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &nanopc_gpio_led_info,
	},
};

/* LCD Backlight data */
static struct samsung_bl_gpio_info nanopc_bl_gpio_info = {
	.no	= EXYNOS4_GPD0(1),
	.func	= S3C_GPIO_SFN(2),
};

static struct platform_pwm_backlight_data nanopc_bl_data = {
	.pwm_id		= 1,
	.pwm_period_ns	= 1000,
};

#if defined(CONFIG_LCD_LP101WH1)
static struct s3c_fb_pd_win nanopc_fb_win0 = {
	.max_bpp	= 32,
	.default_bpp	= 24,
	.xres		= 1360,
	.yres		= 768,
};

static struct fb_videomode nanopc_lcd_timing = {
	.left_margin	= 80,
	.right_margin	= 48,
	.upper_margin	= 14,
	.lower_margin	= 3,
	.hsync_len	= 32,
	.vsync_len	= 5,
	.xres		= 1360,
	.yres		= 768,
};

static struct s3c_fb_platdata nanopc_fb_pdata __initdata = {
	.win[0]		= &nanopc_fb_win0,
	.vtiming	= &nanopc_lcd_timing,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
	.setup_gpio	= exynos4_fimd0_gpio_setup_24bpp,
};

static void lcd_lp101wh1_set_power(struct plat_lcd_data *pd,
				   unsigned int power)
{
	gpio_request(EXYNOS4_GPA1(3), "bl_enable");
	gpio_direction_output(EXYNOS4_GPA1(3), power);
	gpio_free(EXYNOS4_GPA1(3));
}

static struct plat_lcd_data nanopc_lcd_lp101wh1_data = {
	.set_power	= lcd_lp101wh1_set_power,
};

static struct platform_device nanopc_lcd_lp101wh1 = {
	.name	= "platform-lcd",
	.dev	= {
		.parent		= &s5p_device_fimd0.dev,
		.platform_data	= &nanopc_lcd_lp101wh1_data,
	},
};
#endif

/* GPIO KEYS */
static struct gpio_keys_button nanopc_gpio_keys_tables[] = {
	{
		.code			= KEY_1,
		.gpio			= EXYNOS4_GPX3(2),	
		.desc			= "KEY_USER_1",
		.type			= EV_KEY,
		.active_low		= 1,
		.wakeup			= 1,
		.debounce_interval	= 1,
	},
	{
		.code			= KEY_2,
		.gpio			= EXYNOS4_GPX3(3),
		.desc			= "KEY_USER_2",
		.type			= EV_KEY,
		.active_low		= 1,
		.wakeup			= 1,
		.debounce_interval	= 1,
	},
};

static struct gpio_keys_platform_data nanopc_gpio_keys_data = {
	.buttons	= nanopc_gpio_keys_tables,
	.nbuttons	= ARRAY_SIZE(nanopc_gpio_keys_tables),
};

static struct platform_device nanopc_gpio_keys = {
	.name	= "gpio-keys",
	.dev	= {
		.platform_data	= &nanopc_gpio_keys_data,
	},
};

/* USB EHCI */
static struct s5p_ehci_platdata nanopc_ehci_pdata;

static void __init nanopc_ehci_init(void)
{
	struct s5p_ehci_platdata *pdata = &nanopc_ehci_pdata;
	int err;

	s5p_ehci_set_platdata(pdata);

#define GPIO_USBH_RESET		EXYNOS4X12_GPM2(4)
	err = gpio_request_one(GPIO_USBH_RESET,
			GPIOF_OUT_INIT_HIGH, "USBH_RESET");
	if (err)
		pr_err("failed to request GPM2_4 for USB reset control\n");

	s3c_gpio_setpull(GPIO_USBH_RESET, S3C_GPIO_PULL_UP);
	gpio_set_value(GPIO_USBH_RESET, 0);
	mdelay(1);
	gpio_set_value(GPIO_USBH_RESET, 1);
	gpio_free(GPIO_USBH_RESET);
}

/* USB OHCI */
static struct exynos4_ohci_platdata nanopc_ohci_pdata;

static void __init nanopc_ohci_init(void)
{
	struct exynos4_ohci_platdata *pdata = &nanopc_ohci_pdata;

	exynos4_ohci_set_platdata(pdata);
}

/* USB OTG */
static struct s3c_hsotg_plat nanopc_hsotg_pdata;

/* SDCARD */
static struct s3c_sdhci_platdata nanopc_hsmmc2_pdata __initdata = {
	.max_width	= 4,
	.host_caps	= MMC_CAP_4_BIT_DATA |
			MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
	.cd_type	= S3C_SDHCI_CD_NONE,
};
static struct s3c_sdhci_platdata nanopc_hsmmc3_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_PERMANENT,
};


/* DWMMC */
static int nanopc_dwmci_get_bus_wd(u32 slot_id)
{
       return 8;
}

static int nanopc_dwmci_init(u32 slot_id, irq_handler_t handler, void *data)
{
       return 0;
}

static struct dw_mci_board nanopc_dwmci_pdata = {
	.num_slots			= 1,
	.quirks				= DW_MCI_QUIRK_BROKEN_CARD_DETECTION | DW_MCI_QUIRK_HIGHSPEED,
	.caps				= MMC_CAP_UHS_DDR50 | MMC_CAP_1_8V_DDR | MMC_CAP_8_BIT_DATA | MMC_CAP_CMD23,
	.fifo_depth			= 0x80,
	.bus_hz				= 104 * 1000 * 1000,
	.detect_delay_ms	= 200,
	.init				= nanopc_dwmci_init,
	.get_bus_wd			= nanopc_dwmci_get_bus_wd,
	.cfg_gpio			= exynos4_setup_dwmci_cfg_gpio,
};

static struct resource tmu_resource[] = {
	[0] = {
		.start = EXYNOS4_PA_TMU,
		.end = EXYNOS4_PA_TMU + 0x0100,
		.flags = IORESOURCE_MEM,
	},
	[1] = { 
		.start = EXYNOS4_IRQ_TMU_TRIG0,
		.end = EXYNOS4_IRQ_TMU_TRIG0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device nanopc_tmu = {
	.id = -1,
	.name = "exynos5250-tmu",
	.num_resources = ARRAY_SIZE(tmu_resource),
	.resource = tmu_resource,
};

// SPI1
static struct s3c64xx_spi_csinfo spi1_csi = {
		.fb_delay = 0x2,
		.line = EXYNOS4_GPB(5),
};

static struct spi_board_info spi1_board_info[] __initdata = {
	[0] = {
		.modalias = "spidev",
		.max_speed_hz = 10 * 1000 * 1000, // 10 mhz
		.bus_num = 1,
		.chip_select = 0,
		.mode = SPI_MODE_3,
		.controller_data = &spi1_csi,
	},
};

static struct platform_device *nanopc_devices[] __initdata = {
	&s3c_device_hsmmc2,
	&s3c_device_hsmmc3,
	&s3c_device_i2c0,
	&s3c_device_i2c1,
	&gpio_device_i2c2,
	&s3c_device_i2c3,
	&gpio_device_i2c4,
	&s3c_device_i2c7,
	&s3c_device_rtc,
	&s3c_device_usb_hsotg,
	&s3c_device_wdt,
	&s5p_device_ehci,
#ifdef CONFIG_SND_SAMSUNG_I2S
	&exynos4_device_i2s0,
#endif
	&s5p_device_fimc0,
	&s5p_device_fimc1,
	&s5p_device_fimc2,
	&s5p_device_fimc3,
	&s5p_device_fimc_md,
	&s5p_device_fimd0,
	&s5p_device_mfc,
	&s5p_device_mfc_l,
	&s5p_device_mfc_r,
	&s5p_device_g2d,
	&mali_gpu_device,
#if defined(CONFIG_S5P_DEV_TV)
	&s5p_device_hdmi,
	&s5p_device_cec,
	&s5p_device_i2c_hdmiphy,
	&s5p_device_mixer,
	&hdmi_fixed_voltage,
#endif
	&exynos4_device_ohci,
	&exynos_device_dwmci,
	&nanopc_leds_gpio,
#if defined(CONFIG_LCD_LP101WH1)
	&nanopc_lcd_lp101wh1,
#endif
	&nanopc_gpio_keys,
	&samsung_asoc_dma,
	&samsung_asoc_idma,
#if defined(CONFIG_EXYNOS_THERMAL)
	&nanopc_tmu,
#endif
	&s3c64xx_device_spi1,
	&tiny4412_audio,
};

#if defined(CONFIG_S5P_DEV_TV)
static struct s5p_platform_cec hdmi_cec_data __initdata = {

};
#endif

static void __init nanopc_map_io(void)
{
	clk_xusbxti.rate = 24000000;

	exynos_init_io(NULL, 0);
	s3c24xx_init_clocks(clk_xusbxti.rate);
	s3c24xx_init_uarts(nanopc_uartcfgs, ARRAY_SIZE(nanopc_uartcfgs));
}

static void __init nanopc_reserve(void)
{
	s5p_mfc_reserve_mem(0x43000000, 64 << 20, 0x51000000, 64 << 20);
}

#if defined(CONFIG_S5P_DEV_TV)
/* I2C module and id for HDMIPHY */
static struct i2c_board_info hdmiphy_info = {
	I2C_BOARD_INFO("s5p_hdmiphy", 0x38),
};
#endif



static void __init nanopc_gpio_init(void)
{
	/* Peripheral power enable (P3V3) */
	gpio_request_one(EXYNOS4_GPA1(1), GPIOF_OUT_INIT_HIGH, "p3v3_en");

	/* Power on/off button */
	s3c_gpio_cfgpin(EXYNOS4_GPX1(3), S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(EXYNOS4_GPX1(3), S3C_GPIO_PULL_NONE);
}

static void nanopc_power_off(void)
{
	pr_emerg("Bye...\n");

	writel(0x5200, S5P_PS_HOLD_CONTROL);
	while (1) {
		pr_emerg("%s : should not reach here!\n", __func__);
		msleep(1000);
	}
}

static int nanopc_reboot_notifier(struct notifier_block *this, unsigned long code, void *_cmd) {
	pr_emerg("exynos4-reboot: Notifier called\n");

	__raw_writel(0, S5P_INFORM4);

    // eMMC HW_RST  
    gpio_request(EXYNOS4_GPK1(2), "GPK1");
    gpio_direction_output(EXYNOS4_GPK1(2), 0);
    msleep(150);
    gpio_direction_output(EXYNOS4_GPK1(2), 1);
    gpio_free(EXYNOS4_GPK1(2));
	msleep(500);
    return NOTIFY_DONE;
}	


static struct notifier_block nanopc_reboot_notifier_nb = {
	.notifier_call = nanopc_reboot_notifier,
};

static int exynos_boot_dev;
#define is_bootfrom_emmc()	\
	((exynos_boot_dev == 0x6) || (exynos_boot_dev == 0x7))
#define is_bootfrom_sd()	\
	 (exynos_boot_dev == 0x3)
	 
static void __init exynos_bootdev_init(void)
{
	u32 capboot = MMC_CAP2_BOOT_DEVICE;

	exynos_boot_dev = __raw_readl(S5P_INFORM3);

	if (is_bootfrom_emmc()) {
#if defined(CONFIG_EXYNOS4_DEV_DWMCI)
		nanopc_dwmci_pdata.caps2 |= capboot;
#endif
	} else if (is_bootfrom_sd()) {
		nanopc_hsmmc2_pdata.host_caps2 |= capboot;
	} else {
		/* oops...should never fly to here */
		printk(KERN_ERR "Unknown boot device\n");
	}
}

static void __init nanopc_machine_init(void)
{
	exynos_bootdev_init();

	nanopc_gpio_init();

	/* Register power off function */
	pm_power_off = nanopc_power_off;

	s3c_i2c0_set_platdata(&nanopc_i2c0_data);
	i2c_register_board_info(0, nanopc_i2c_devs0,
				ARRAY_SIZE(nanopc_i2c_devs0));

	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(1, nanopc_i2c_devs1,
				ARRAY_SIZE(nanopc_i2c_devs1));

	i2c_register_board_info(2, nanopc_i2c_devs2,
				ARRAY_SIZE(nanopc_i2c_devs2));

	s3c_i2c3_set_platdata(NULL);
	i2c_register_board_info(3, nanopc_i2c_devs3,
				ARRAY_SIZE(nanopc_i2c_devs3));

#if defined(CONFIG_ODROID_U2) || defined(CONFIG_NANOPC_T1)
	i2c_register_board_info(4, nanopc_i2c_devs4,
				ARRAY_SIZE(nanopc_i2c_devs4));
#endif

	s3c_i2c7_set_platdata(NULL);
	i2c_register_board_info(7, nanopc_i2c_devs7,
				ARRAY_SIZE(nanopc_i2c_devs7));

	s3c_sdhci2_set_platdata(&nanopc_hsmmc2_pdata);
	s3c_sdhci3_set_platdata(&nanopc_hsmmc3_pdata);

	exynos4_setup_dwmci_cfg_gpio(NULL, MMC_BUS_WIDTH_8);
	exynos_dwmci_set_platdata(&nanopc_dwmci_pdata);

	nanopc_ehci_init();
	nanopc_ohci_init();
	s3c_hsotg_set_platdata(&nanopc_hsotg_pdata);

#ifdef CONFIG_LCD_LP101WH1
	nanopc_fb_init_pdata(&nanopc_fb_pdata);
   	s5p_fimd0_set_platdata(&nanopc_fb_pdata);
#endif

	s3c64xx_spi1_set_platdata(NULL, 0, 1);
	spi_register_board_info(spi1_board_info, ARRAY_SIZE(spi1_board_info));

#if defined(CONFIG_S5P_DEV_TV)
	s5p_i2c_hdmiphy_set_platdata(NULL);
	s5p_hdmi_set_platdata(&hdmiphy_info, NULL, 0, EXYNOS4_GPX3(7));
	s5p_hdmi_cec_set_platdata(&hdmi_cec_data);
	/* FIXME: hdmiphy i2c adapter has dynamic ID, and setting it to 8 causes
	 * a failure to initialize (can't find clock?). so for now we are relying
	 * on the hdmiphy i2c adapter being dynamically assigned address 8. */
	i2c_register_board_info(8, &hdmiphy_info, 1);
#endif

	platform_add_devices(nanopc_devices, ARRAY_SIZE(nanopc_devices));

	samsung_bl_set(&nanopc_bl_gpio_info, &nanopc_bl_data);

	register_reboot_notifier(&nanopc_reboot_notifier_nb);
}

MACHINE_START(NANOPC_T1, "NANOPC_T1")
	/* Maintainer: FriendlyARM (www.arm9.net) */
	/* Maintainer: Dongjin Kim <dongjin.kim@agreeyamobiity.net> */
	.atag_offset	= 0x100,
	.smp		= smp_ops(exynos_smp_ops),
	.init_irq	= exynos4_init_irq,
	.init_early	= exynos_firmware_init,
	.map_io		= nanopc_map_io,
	.handle_irq	= gic_handle_irq,
	.init_machine	= nanopc_machine_init,
	.init_late	= exynos_init_late,
	.timer		= &exynos4_timer,
	.restart	= exynos4_restart,
	.reserve	= &nanopc_reserve,
MACHINE_END
