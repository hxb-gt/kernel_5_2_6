// SPDX-License-Identifier: GPL-2.0
// linux/arch/arm/mach-s3c2440/mach-embedsky2440.c
//
// Copyright (c) 2004-2005 Simtec Electronics
//	Ben Dooks <ben@simtec.co.uk>
//
// http://www.fluff.org/ben/embedsky2440/
//
// Thanks to Dimity Andric and TomTom for the loan of an embedsky2440.

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/serial_s3c.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <mach/regs-gpio.h>
#include <mach/regs-lcd.h>
#include <mach/gpio-samsung.h>

#include <mach/fb.h>
#include <linux/platform_data/i2c-s3c2410.h>
#include <linux/platform_data/usb-s3c2410_udc.h>

#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/samsung-time.h>

#include <linux/dm9000.h>
#include "common.h"
#include "common-smdk.h"

static struct map_desc embedsky2440_iodesc[] __initdata = {
	/* ISA IO Space map (memory space selected by A24) */

	{
		.virtual	= (u32)S3C24XX_VA_ISA_WORD,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_WORD + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}
};

#define UCON S3C2410_UCON_DEFAULT | S3C2410_UCON_UCLK
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

static struct s3c2410_uartcfg embedsky2440_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	},
	/* IR port */
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x43,
		.ufcon	     = 0x51,
	}
};

/* LCD driver info */

static struct s3c2410fb_display embedsky2440_lcd_cfg __initdata = {

	.lcdcon5	= S3C2410_LCDCON5_FRM565 |
			  S3C2410_LCDCON5_INVVLINE |
			  S3C2410_LCDCON5_INVVFRAME |
			  S3C2410_LCDCON5_PWREN |
			  S3C2410_LCDCON5_HWSWP,

	.type		= S3C2410_LCDCON1_TFT,

	.width		= 240,
	.height		= 320,

	.pixclock	= 166667, /* HCLK 60 MHz, divisor 10 */
	.xres		= 240,
	.yres		= 320,
	.bpp		= 16,
	.left_margin	= 20,
	.right_margin	= 8,
	.hsync_len	= 4,
	.upper_margin	= 8,
	.lower_margin	= 7,
	.vsync_len	= 4,
};

static struct s3c2410fb_mach_info embedsky2440_fb_info __initdata = {
	.displays	= &embedsky2440_lcd_cfg,
	.num_displays	= 1,
	.default_display = 0,

#if 0
	/* currently setup by downloader */
	.gpccon		= 0xaa940659,
	.gpccon_mask	= 0xffffffff,
	.gpcup		= 0x0000ffff,
	.gpcup_mask	= 0xffffffff,
	.gpdcon		= 0xaa84aaa0,
	.gpdcon_mask	= 0xffffffff,
	.gpdup		= 0x0000faff,
	.gpdup_mask	= 0xffffffff,
#endif

	.lpcsel		= ((0xCE6) & ~7) | 1<<4,
};

static struct resource dm9000_res[]=
{
	[0] = {
		.start = 0x20000000,
		.end = 0x20000003,
		.name = "dm9000_addr",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = 0x20000004,
		.end = 0x20000007,
		.name = "dm9000_data",
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.start = IRQ_EINT7,
		.end = 0,
		.name = "dm9000_irq",
		.flags = IORESOURCE_IRQ | IRQ_TYPE_LEVEL_HIGH,
	},
};
struct dm9000_plat_data dm9000_prvdata = {
	.flags = DM9000_PLATF_16BITONLY,
};

static struct platform_device s3c_device_dm9000 = {
	.name = "dm9000",
	.dev = {
		.platform_data = &dm9000_prvdata,
	},
	.id = 0,
	.num_resources = ARRAY_SIZE(dm9000_res),
	.resource = dm9000_res,
};

struct s3c2410_udc_mach_info s3c2440_udc_info = {
	NULL, NULL, S3C2410_GPC(5), 0, 0, 0
};


static struct platform_device *embedsky2440_devices[] __initdata = {
	&s3c_device_ohci,
	&s3c_device_lcd,
	&s3c_device_wdt,
	&s3c_device_i2c0,
	&s3c_device_iis,
	&s3c_device_dm9000,
	&s3c_device_usbgadget,
};

static void __init embedsky2440_map_io(void)
{
	s3c24xx_init_io(embedsky2440_iodesc, ARRAY_SIZE(embedsky2440_iodesc));
	s3c24xx_init_uarts(embedsky2440_uartcfgs, ARRAY_SIZE(embedsky2440_uartcfgs));
	samsung_set_timer_source(SAMSUNG_PWM3, SAMSUNG_PWM4);
}

static void __init embedsky2440_init_time(void)
{
	s3c2440_init_clocks(12000000);
	samsung_timer_init();
}

static void __init embedsky2440_machine_init(void)
{
	s3c24xx_fb_set_platdata(&embedsky2440_fb_info);
	s3c_i2c0_set_platdata(NULL);
	s3c24xx_udc_set_platdata(&s3c2440_udc_info);

	platform_add_devices(embedsky2440_devices, ARRAY_SIZE(embedsky2440_devices));
	smdk_machine_init();
}

MACHINE_START(EMBEDSKY2440, "embedsky2440")
	/* Maintainer: Ben Dooks <ben-linux@fluff.org> */
	.atag_offset	= 0x100,

	.init_irq	= s3c2440_init_irq,
	.map_io		= embedsky2440_map_io,
	.init_machine	= embedsky2440_machine_init,
	.init_time	= embedsky2440_init_time,
MACHINE_END
