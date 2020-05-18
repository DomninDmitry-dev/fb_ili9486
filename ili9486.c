#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h> /* For platform devices */

#include "ili9486_image.h"

// https://www.displaytech-us.com/forum/ili9341-initialization-code

#define ILI9486_MADCTL_RGB	0x00
#define ILI9486_MADCTL_BGR	0x08
#define ILI9486_MADCTL_MY	0x80
#define ILI9486_MADCTL_MX	0x40
#define ILI9486_MADCTL_MV	0x20
#define ILI9486_MADCTL_ML	0x10

// 1.44" display, default orientation
#define ILI9486_WIDTH		480
#define ILI9486_HEIGHT		320
#define ILI9486_XSTART		0
#define ILI9486_YSTART		0
#define ILI9486_ROTATION	(ILI9486_MADCTL_MV | ILI9486_MADCTL_MY)

// Commands
#define ILI9486_SWRESET		0x01 // SoftWare Reset
#define ILI9486_SLPOUT		0x11 // Sleep Out
#define ILI9486_FRMCTR1		0xB1 // Norman Mode (Full colors)
#define ILI9486_FRMCTR2		0xB2 // In idle Mode (8 colors)
#define ILI9486_FRMCTR3		0xB3 // In partial Mode + Full Colors
#define ILI9486_INVCTR		0xB4 // Display inversion control
#define ILI9486_DFCTR		0xB6 // Display Function Control
#define ILI9486_PWCTR1		0xC0 // Power control setting
#define ILI9486_PWCTR2		0xC1 // Power control setting
#define ILI9486_PWCTR3		0xC2 // In normal mode (Full colors)
#define ILI9486_PWCTR4		0xC3 // In idle mode (8 colors)
#define ILI9486_PWCTR5		0xC4 // In partial mode + Full colors
#define ILI9486_VMCTR1		0xC5 // VCOM control 1
#define ILI9486_CABCCTRL9	0xC6

#define ILI9486_INVOFF		0x20 // Display inversion off
#define ILI9486_MADCTL		0x36 // Memory data access control
#define ILI9486_COLMOD		0x3A // Interface pixel format
#define ILI9486_CASET		0x2A
#define ILI9486_RASET		0x2B
#define ILI9486_GMCTRP1		0xE0
#define ILI9486_GMCTRN1		0xE1
#define ILI9486_DGAMMA		0xE2
#define ILI9486_NORON		0x13
#define ILI9486_DISPON		0x29
#define ILI9486_DISPOFF		0x28
#define ILI9486_RAMWR		0x2C

#define DELAY			0x80

#define DEVICE_NAME "ili9486"

#define REFRESHRATE	15
#define MAX_PALETTE	16
#define BPP	16

static u_int refreshrate = REFRESHRATE;
module_param(refreshrate, uint, 0);

// https://sourceforge.net/p/rozoom/code-0/6/tree/trunk/Arduino/libraries/UTFT/tft_drivers/ili9486/initlcd.h
// https://www.youtube.com/watch?v=tTT4Lb1jr4s

static const u16 init_cmds[] = {
	// Init for ili9486, part 1 (red or green tab)
	16,

	ILI9486_SWRESET, DELAY,	// 1: Software reset, 0 args, w/delay
	150,			// 150 ms delay

	ILI9486_DISPOFF, DELAY,	// 4: Main screen turn off, no args w/delay
	100,

	ILI9486_SLPOUT, DELAY,	// 2: Out of sleep mode, 0 args, w/delay
	255,			// 255 ms delay

	/*0xF2, 9,
	0x1C, 0xA3, 0x32, 0x02, 0xB2, 0x12, 0xFF, 0x12, 0x00,

	0xF1, 2,
	0x36, 0xA4,

	0xF8, 2,
	0x21, 0x04,

	0xF9, 2,
	0x00, 0x08,*/

	ILI9486_PWCTR1, 2,
	0x0D, 0x0D,

	ILI9486_PWCTR2, 2,
	0x43, 0x00,

	ILI9486_PWCTR3, 1,
	0x44,

	ILI9486_VMCTR1, 4,
	0x00, 0x00, 0x00, 0x00,

	ILI9486_GMCTRP1, 15,	// PGAMCTRL (Positive Gamma Control)
	0x0F, 0x1F, 0x1C, 0x0C,
	0x0F, 0x08, 0x48, 0x98,
	0x37, 0x0A, 0x13, 0x04,
	0x11, 0x0D, 0x00,

	ILI9486_GMCTRN1, 15,	// NGAMCTRL (Negative Gamma Control)
	0x0F, 0x32, 0x2E, 0x0B,
	0x0D, 0x05, 0x47, 0x75,
	0x37, 0x06, 0x10, 0x03,
	0x24, 0x20, 0x00,

	ILI9486_INVOFF, 1,	// Display Inversion OFF
	0x00,

	ILI9486_MADCTL, 1,	// Memory Access Control
	ILI9486_ROTATION,

	ILI9486_COLMOD, 1,	// Interface Pixel Format
	0x55,

	ILI9486_CASET, 4,	// 1: Column addr set, 4 args, no delay:
	0x00, 0x00,		// XSTART = 0
	0x01, 0xDF,		// XEND = 480

	ILI9486_RASET, 4,	// 2: Row addr set, 4 args, no delay:
	0x00, 0x00,		// XSTART = 0
	0x01, 0x3F,		// XEND = 320

	ILI9486_DISPON, DELAY,	// 4: Main screen turn on, no args w/delay
	100,

	ILI9486_RAMWR, DELAY,
	100
}; // 16-bit color

enum ili9486_pin {
	PIN_DB0 = 0,	/* Optional */
	PIN_DB1,	/* Optional */
	PIN_DB2,	/* Optional */
	PIN_DB3,	/* Optional */
	PIN_DB4,	/* Optional */
	PIN_DB5,	/* Optional */
	PIN_DB6,	/* Optional */
	PIN_DB7,	/* Optional */
	PIN_DB8,	/* Optional */
	PIN_DB9,	/* Optional */
	PIN_DB10,	/* Optional */
	PIN_DB11,	/* Optional */
	PIN_DB12,	/* Optional */
	PIN_DB13,	/* Optional */
	PIN_DB14,	/* Optional */
	PIN_DB15,	/* Optional */
	PIN_DB_MAX
};

struct ili9486_data {
	struct device dev;
	struct gpio_desc *gpiod_data[PIN_DB_MAX];
	struct gpio_desc *gpiod_wr;
	struct gpio_desc *gpiod_rs;
	struct gpio_desc *gpiod_reset;
	struct fb_info *lcd_info;
	struct mutex io_lock;
	u32 height;
	u32 width;
	struct device_attribute dev_attr_power;
	u8 statePower;
	//u8 *ssbuf;
};

static void ili9486_reset(struct ili9486_data *lcd)
{
	gpiod_set_value(lcd->gpiod_reset, 0);
	mdelay(5);
	gpiod_set_value(lcd->gpiod_reset, 1);
}

static void ili9486_write_command(struct ili9486_data *lcd, u8 cmd)
{
	gpiod_set_value(lcd->gpiod_rs, 0);
	gpiod_set_value(lcd->gpiod_wr, 0);
	gpiod_set_value(lcd->gpiod_data[PIN_DB0], (cmd & 0x01)?1:0);
	gpiod_set_value(lcd->gpiod_data[PIN_DB1], (cmd & 0x02)?1:0);
	gpiod_set_value(lcd->gpiod_data[PIN_DB2], (cmd & 0x04)?1:0);
	gpiod_set_value(lcd->gpiod_data[PIN_DB3], (cmd & 0x08)?1:0);
	gpiod_set_value(lcd->gpiod_data[PIN_DB4], (cmd & 0x10)?1:0);
	gpiod_set_value(lcd->gpiod_data[PIN_DB5], (cmd & 0x20)?1:0);
	gpiod_set_value(lcd->gpiod_data[PIN_DB6], (cmd & 0x40)?1:0);
	gpiod_set_value(lcd->gpiod_data[PIN_DB7], (cmd & 0x80)?1:0);
	//dev_info(&lcd->dev, "command: 0x%02x", cmd);
	gpiod_set_value(lcd->gpiod_wr, 1);
}

static void ili9486_write_data(struct ili9486_data *lcd, u16 *buff, size_t buff_size)
{
	int cnt;
	gpiod_set_value(lcd->gpiod_rs, 1);
	for (cnt = 0; cnt < buff_size; cnt++) {
		gpiod_set_value(lcd->gpiod_wr, 0);
		gpiod_set_value(lcd->gpiod_data[PIN_DB0], (buff[cnt] & 0x0001)?1:0);
		gpiod_set_value(lcd->gpiod_data[PIN_DB1], (buff[cnt] & 0x0002)?1:0);
		gpiod_set_value(lcd->gpiod_data[PIN_DB2], (buff[cnt] & 0x0004)?1:0);
		gpiod_set_value(lcd->gpiod_data[PIN_DB3], (buff[cnt] & 0x0008)?1:0);
		gpiod_set_value(lcd->gpiod_data[PIN_DB4], (buff[cnt] & 0x0010)?1:0);
		gpiod_set_value(lcd->gpiod_data[PIN_DB5], (buff[cnt] & 0x0020)?1:0);
		gpiod_set_value(lcd->gpiod_data[PIN_DB6], (buff[cnt] & 0x0040)?1:0);
		gpiod_set_value(lcd->gpiod_data[PIN_DB7], (buff[cnt] & 0x0080)?1:0);
		gpiod_set_value(lcd->gpiod_data[PIN_DB8], (buff[cnt] & 0x0100)?1:0);
		gpiod_set_value(lcd->gpiod_data[PIN_DB9], (buff[cnt] & 0x0200)?1:0);
		gpiod_set_value(lcd->gpiod_data[PIN_DB10], (buff[cnt] & 0x0400)?1:0);
		gpiod_set_value(lcd->gpiod_data[PIN_DB11], (buff[cnt] & 0x0800)?1:0);
		gpiod_set_value(lcd->gpiod_data[PIN_DB12], (buff[cnt] & 0x1000)?1:0);
		gpiod_set_value(lcd->gpiod_data[PIN_DB13], (buff[cnt] & 0x2000)?1:0);
		gpiod_set_value(lcd->gpiod_data[PIN_DB14], (buff[cnt] & 0x4000)?1:0);
		gpiod_set_value(lcd->gpiod_data[PIN_DB15], (buff[cnt] & 0x8000)?1:0);
		//dev_info(&lcd->dev, "data: 0x%04x", buff[cnt]);
		gpiod_set_value(lcd->gpiod_wr, 1);
		ndelay(99999);
	}
}

static ssize_t power_show(struct device *child, struct device_attribute *attr, char *buf)
{
	struct ili9486_data *lcd = container_of(attr, struct ili9486_data,
		dev_attr_power);

	return sprintf(buf, "%d\n", lcd->statePower);
}
static ssize_t power_store(struct device *child, struct device_attribute *attr,
	const char *buf, size_t len)
{
	struct ili9486_data *lcd = container_of(attr, struct ili9486_data,
		dev_attr_power);

	sscanf(buf, "%hhd", &lcd->statePower);

	if (lcd->statePower == 1) {
		dev_info(&lcd->dev, "ILI9486_DISPON\n");
		ili9486_write_command(lcd, ILI9486_DISPON);
	} else {
		dev_info(&lcd->dev, "ILI9486_DISPOFF\n");
		ili9486_write_command(lcd, ILI9486_DISPOFF);
	}

	return len;
}

static void ili9486_execute_command_list(struct ili9486_data *lcd, const u16 *addr)
{
	u16 numCommands, numArgs;
	u16 ms;

	numCommands = *addr++;
	while (numCommands--) {
		u8 cmd = *addr++;
		ili9486_write_command(lcd, cmd);
		numArgs = *addr++;
		ms = numArgs & DELAY;
		numArgs &= ~DELAY;
		if (numArgs) {
			ili9486_write_data(lcd, (u16 *)addr, numArgs);
			addr += numArgs;
		}

		if (ms) {
			ms = *addr++;
			if (ms == 255)
				ms = 500;
			mdelay(ms);
			//dev_info(&lcd->dev, "delay: 0x%04x", ms);
		}
	}
}

static void ili9486_set_address_window(struct ili9486_data *lcd, u16 x0, u16 y0,
	u16 x1, u16 y1)
{
	u16 data[] = {0x00, x0 + ILI9486_XSTART, 0x00, x1 + ILI9486_XSTART};

	ili9486_write_command(lcd, ILI9486_CASET);
	ili9486_write_data(lcd, data, sizeof(data));

	// row address set
	ili9486_write_command(lcd, ILI9486_RASET);
	data[1] = y0 + ILI9486_YSTART;
	data[3] = y1 + ILI9486_YSTART;
	ili9486_write_data(lcd, data, sizeof(data));

	// write to RAM
	ili9486_write_command(lcd, ILI9486_RAMWR);
}

static void ili9486_update_screen(struct ili9486_data *lcd)
{
	//u16 *vmem;
	//int i;

	//vmem = (u16 *)lcd->lcd_info->screen_base;

	//u16 *vmem16 = (u16 *)lcd->lcd_info->screen_base;
	//vmem = lcd->ssbuf;

	//for (i = 0; i < ILI9486_WIDTH * ILI9486_HEIGHT * BPP / 8 / 2; i++)
		//vmem[i] = swab16(vmem16[i]);

	mutex_lock(&(lcd->io_lock));

	/* Set row/column data window */
	ili9486_set_address_window(lcd, 0, 0, ILI9486_WIDTH - 1,
						ILI9486_HEIGHT - 1);

	/* Blast framebuffer to ILI9486 internal display RAM */
	ili9486_write_data(lcd, (u16*)lcd->lcd_info->screen_base,
		ILI9486_WIDTH * ILI9486_HEIGHT * BPP / 16);

	mutex_unlock(&(lcd->io_lock));

	//dev_info(&lcd->dev, "ili9486_update_screen\n");
}

static void ili9486_load_image(struct ili9486_data *lcd, const u8 *image)
{
	//u16 *vmem;
	//int i;

	//u16 *vmem16 = (u16 *)image;
	//vmem = lcd->ssbuf;

	//for (i = 0; i < ILI9486_WIDTH * ILI9486_HEIGHT * BPP / 8 / 2; i++)
	//	vmem[i] = swab16(vmem16[i]);

	//memcpy(lcd->lcd_info->screen_base, (u8 *)vmem, ILI9486_WIDTH *
	//	ILI9486_HEIGHT * BPP / 8);

	//memcpy(lcd->ssbuf, (u8 *)image, 32768);

	//ili9486_update_screen(lcd);

	mutex_lock(&(lcd->io_lock));

	/* Set row/column data window */
	ili9486_set_address_window(lcd, 0, 0, ILI9486_WIDTH - 1,
						ILI9486_HEIGHT - 1);

	/* Blast framebuffer to ILI9486 internal display RAM */
	ili9486_write_data(lcd, (u16 *)image, 32768 / 2);

	mutex_unlock(&(lcd->io_lock));
}

static void ili9486_fill_rectangle(struct ili9486_data *lcd, u16 x, u16 y, u16 w, u16 h,
						u16 color)
{
	u16 i = 0;
	u16 j = 0;

	if ((x >= ILI9486_WIDTH) || (y >= ILI9486_HEIGHT))
		return;

	if ((x + w - 1) > ILI9486_WIDTH)
		w = ILI9486_WIDTH - x;

	if ((y + h - 1) > ILI9486_HEIGHT)
		h = ILI9486_HEIGHT - y;

	for (j = 0; j < h; ++j) {
		for (i = 0; i < w; ++i) {
			lcd->lcd_info->screen_base[(x + ILI9486_WIDTH * y) +
		(i + ILI9486_WIDTH * j)] = color;
		}
	}

	ili9486_update_screen(lcd);
}

static ssize_t ili9486fb_write(struct fb_info *info, const char __user *buf,
		size_t count, loff_t *ppos)
{
	struct fb_deferred_io *fbdefio = info->fbdefio;
	unsigned long total_size;
	unsigned long p = *ppos;
	u8 __iomem *dst;

	total_size = info->fix.smem_len;

	if (p > total_size)
		return -EINVAL;

	if (count + p > total_size)
		count = total_size - p;

	if (!count)
		return -EINVAL;

	dst = (void __force *) (info->screen_base + p);

	if (copy_from_user(dst, buf, count))
		return -EFAULT;

	schedule_delayed_work(&info->deferred_work, fbdefio->delay);

	*ppos += count;

	return count;
}

static int ili9486fb_blank(int blank_mode, struct fb_info *info)
{
	struct ili9486_data *lcd = info->par;

	// dev_info(info->dev, "%s(blank=%d)\n",
	// 	__func__, blank_mode);

	switch (blank_mode) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		ili9486_write_command(lcd, ILI9486_DISPOFF);
		break;
	case FB_BLANK_UNBLANK:
		ili9486_write_command(lcd, ILI9486_DISPON);
		break;
	}

	return 0;
}

static void ili9486fb_fillrect(struct fb_info *info,
			const struct fb_fillrect *rect)
{
	struct fb_deferred_io *fbdefio = info->fbdefio;

	// dev_info(info->dev,
	// 	"%s: dx=%d, dy=%d, width=%d, height=%d\n",
	// 	__func__, rect->dx, rect->dy, rect->width, rect->height);

	sys_fillrect(info, rect);
	schedule_delayed_work(&info->deferred_work, fbdefio->delay);
}

static void ili9486fb_copyarea(struct fb_info *info,
			const struct fb_copyarea *area)
{
	struct fb_deferred_io *fbdefio = info->fbdefio;

	// dev_info(info->dev,
	// 	"%s: dx=%d, dy=%d, width=%d, height=%d\n",
	// 	__func__,  area->dx, area->dy, area->width, area->height);

	sys_copyarea(info, area);
	schedule_delayed_work(&info->deferred_work, fbdefio->delay);
}

static void ili9486fb_imageblit(struct fb_info *info,
			const struct fb_image *image)
{
	struct fb_deferred_io *fbdefio = info->fbdefio;

	// dev_info(info->dev,
	// 	"%s: dx=%d, dy=%d, width=%d, height=%d\n",
	// 	__func__,  image->dx, image->dy, image->width, image->height);

	sys_imageblit(info, image);
	schedule_delayed_work(&info->deferred_work, fbdefio->delay);
}

static unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int ili9486fb_setcolreg(unsigned int regno, unsigned int red,
			unsigned int green, unsigned int blue,
			unsigned int transp, struct fb_info *info)
{
	unsigned int val;
	int ret = 1;

	// dev_info(info->dev,
	// 	"%s(regno=%u, red=0x%X, green=0x%X, blue=0x%X, trans=0x%X)\n",
	// 	__func__, regno, red, green, blue, transp);

	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;

			val  = chan_to_field(red,   &info->var.red);
			val |= chan_to_field(green, &info->var.green);
			val |= chan_to_field(blue,  &info->var.blue);

			pal[regno] = val;
			ret = 0;
		}
		break;
	}
	return ret;
}

static struct fb_fix_screeninfo ili9486fb_fix = {
	.id = "ili9486",
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
	.accel = FB_ACCEL_NONE,
};

static struct fb_var_screeninfo ili9486fb_var = {
	.activate = FB_ACTIVATE_NOW,
	.vmode = FB_VMODE_NONINTERLACED,
	.red.length = 5,
	.red.offset = 11,
	.green.length = 6,
	.green.offset = 5,
	.blue.length = 5,
	.blue.offset = 0,
	.bits_per_pixel = 16,
	.nonstd = 1,
	.transp.offset = 0,
	.transp.length = 0
};

static struct fb_ops ili9486fb_ops = {
	.owner 		= THIS_MODULE,
	.fb_read 	= fb_sys_read,
	.fb_write 	= ili9486fb_write,
	.fb_blank 	= ili9486fb_blank,
	.fb_fillrect 	= ili9486fb_fillrect,
	.fb_copyarea 	= ili9486fb_copyarea,
	.fb_imageblit 	= ili9486fb_imageblit,
	.fb_setcolreg	= ili9486fb_setcolreg,
};

static void ili9486fb_deferred_io(struct fb_info *info,
				struct list_head *pagelist)
{
	ili9486_update_screen(info->par);
}

static int ili9486_probe(struct platform_device *pdev)
{
	struct ili9486_data *lcd;
	int ret = 0, gpiocnt = 0, pins;
	struct fb_info *info;
	u32 vmem_size = 0;
	u8 *vmem;
	struct fb_deferred_io *ili9486fb_defio;

	lcd = devm_kzalloc(&pdev->dev, sizeof(struct ili9486_data), GFP_KERNEL);
	if (IS_ERR(lcd)) {
		dev_err(&pdev->dev, "error mem <devm_kzalloc>\n");
		return -ENOMEM;
	}

	lcd->dev = pdev->dev;

	gpiocnt = gpiod_count(&pdev->dev, "db");
	if (gpiocnt < 0) {
		dev_err(&pdev->dev, "error get count gpio! \n");
		devm_kfree(&pdev->dev, lcd);
		return -EIO;
	}
	dev_info(&pdev->dev, "Gpio data count: %d\n", gpiocnt);

	/*
	 * If devm_gpiod_get_index is used,
	 * gpiod_put is called automatically when unloading the module
	 * */
	for (pins = 0; pins < gpiocnt; pins++) {
		lcd->gpiod_data[pins] = devm_gpiod_get_index(&pdev->dev, "db", pins,
							  GPIOD_OUT_LOW);
		if (IS_ERR(lcd->gpiod_data[pins])) {
			ret = PTR_ERR(lcd->gpiod_data[pins]);
			dev_err(&pdev->dev, "Error get index gpio %d, err %d! \n", pins, ret);
			devm_kfree(&pdev->dev, lcd);
			return -EIO;
		}
		ret = gpiod_direction_output(lcd->gpiod_data[pins], 0);
		if(ret < 0) {
			dev_err(&pdev->dev, "Error direction output gpio %d, err %d! \n", pins, ret);
			devm_kfree(&pdev->dev, lcd);
			return -EIO;
		}
	}
	dev_info(&pdev->dev, "Gpio DB[0-15] loaded");

	/* The WR gets a GPIO pin number */
	lcd->gpiod_wr = devm_gpiod_get_optional(&pdev->dev, "wr", GPIOD_OUT_HIGH);
	if (IS_ERR(lcd->gpiod_wr)) {
		ret = PTR_ERR(lcd->gpiod_wr);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to get %s GPIO: %d\n",
				"wr", ret);
		devm_kfree(&pdev->dev, lcd);
		return ret;
	}
	ret = gpiod_direction_output(lcd->gpiod_wr, 0);
	dev_info(&pdev->dev, "Gpio WR loaded");

	/* The RS gets a GPIO pin number */
	lcd->gpiod_rs = devm_gpiod_get_optional(&pdev->dev, "rs", GPIOD_OUT_LOW);
	if (IS_ERR(lcd->gpiod_rs)) {
		ret = PTR_ERR(lcd->gpiod_rs);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to get %s GPIO: %d\n",
				"rs", ret);
		devm_kfree(&pdev->dev, lcd);
		return ret;
	}
	ret = gpiod_direction_output(lcd->gpiod_rs, 0);
	dev_info(&pdev->dev, "Gpio RS loaded");

	/* The RST gets a GPIO pin number */
	lcd->gpiod_reset = devm_gpiod_get_optional(&pdev->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(lcd->gpiod_reset)) {
		ret = PTR_ERR(lcd->gpiod_reset);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to get %s GPIO: %d\n",
				"reset", ret);
		devm_kfree(&pdev->dev, lcd);
		return ret;
	}
	ret = gpiod_direction_output(lcd->gpiod_reset, 0);
	dev_info(&pdev->dev, "Gpio RESET loaded");

	ili9486_reset(lcd);
	ili9486_execute_command_list(lcd, init_cmds);
	ili9486_set_address_window(lcd, 0, 0, ILI9486_WIDTH - 1,
						ILI9486_HEIGHT - 1);
	dev_info(&pdev->dev, "device init completed\n");

	// Init fb
	info = framebuffer_alloc(sizeof(struct ili9486_data), &pdev->dev);
	if (!info) {
		devm_kfree(&pdev->dev, lcd);
		return -ENOMEM;
	}

	dev_info(&pdev->dev, "frame buffer allocated\n");

	lcd->lcd_info = info;
	lcd->width  = ILI9486_WIDTH;
	lcd->height = ILI9486_HEIGHT;
	vmem_size = lcd->width * lcd->height * BPP / 8;

	vmem = vmalloc(vmem_size);
	if (!vmem) {
		dev_err(&pdev->dev, "Couldn't allocate graphical memory\n");
		devm_kfree(&pdev->dev, lcd);
		return -ENOMEM;
	}

	mutex_init(&lcd->io_lock);

	ili9486fb_defio = devm_kzalloc(&pdev->dev, sizeof(struct fb_deferred_io),
				GFP_KERNEL);
	if (!ili9486fb_defio) {
		dev_err(&pdev->dev, "Couldn't allocate deferred io.\n");
		vfree(vmem);
		devm_kfree(&pdev->dev, lcd);
		return -ENOMEM;
	}

	ili9486fb_defio->delay = HZ / refreshrate;
	ili9486fb_defio->deferred_io = ili9486fb_deferred_io;
	info->fbdefio = ili9486fb_defio;

	info->fbops = &ili9486fb_ops;
	info->fix = ili9486fb_fix;
	info->fix.line_length = lcd->width * BPP / 8;

	info->var = ili9486fb_var;
	info->var.xres = lcd->width;
	info->var.xres_virtual = lcd->width;
	info->var.yres = lcd->height;
	info->var.yres_virtual = lcd->height;
	info->var.width = lcd->width;
	info->var.height = lcd->height;

	info->screen_base = (u8 __force __iomem *)vmem;
	info->fix.smem_start = __pa(vmem);;
	info->fix.smem_len = vmem_size;
	info->flags = FBINFO_FLAG_DEFAULT | FBINFO_VIRTFB;
	info->par = lcd;
	info->pseudo_palette = kmalloc(MAX_PALETTE * sizeof(u32), GFP_KERNEL);

	ret = fb_alloc_cmap(&info->cmap, MAX_PALETTE, 0);
	if (ret < 0) {
		kfree(info->pseudo_palette);
		vfree(vmem);
		devm_kfree(&pdev->dev, lcd);
		return -ENOMEM;
	}
	info->cmap.len = MAX_PALETTE;

	fb_deferred_io_init(info);

	dev_info(&pdev->dev, "info.fix.smem_start=%lu\ninfo.fix.smem_len=%d\ninfo.screen_size=%lu\n",
		info->fix.smem_start, info->fix.smem_len, info->screen_size);

	platform_set_drvdata(pdev, lcd);

	/*It allocated swapped shadow buffer */
	// lcd->ssbuf = kmalloc(vmem_size, GFP_KERNEL);
	// if (!lcd->ssbuf) {
	// 	fb_deferred_io_cleanup(info);
	// 	fb_dealloc_cmap(&info->cmap);
	// 	kfree(info->pseudo_palette);
	// 	vfree(vmem);
	// 	devm_kfree(&pdev->dev, lcd);
	// 	return -ENOMEM;
	// }

	// Test load image 480x320
	ili9486_load_image(lcd, lcd_image);
	//ili9486_fill_rectangle(lcd, 0, 0, ILI9486_WIDTH, ILI9486_HEIGHT, 0x07A5);
	msleep(5000);

	/*ret = register_framebuffer(info);
	if (ret) {
		fb_deferred_io_cleanup(info);
		fb_dealloc_cmap(&info->cmap);
		kfree(info->pseudo_palette);
		vfree(vmem);
		devm_kfree(&pdev->dev, lcd);
		dev_err(&pdev->dev, "Error: ret = %d\n", ret);
		return ret;
	}
	dev_info(&pdev->dev, "The frame buffer registered\n");*/

	lcd->dev_attr_power.attr.name = "powerOnOff";
	lcd->dev_attr_power.attr.mode = S_IRUGO | S_IWUSR;
	lcd->dev_attr_power.show = &power_show;
	lcd->dev_attr_power.store = &power_store;
	ret = device_create_file(&pdev->dev, &lcd->dev_attr_power);
	if(ret != 0){
		dev_err(&pdev->dev, "Error create attribute file %s!",
			lcd->dev_attr_power.attr.name);
		unregister_framebuffer(lcd->lcd_info);
		fb_deferred_io_cleanup(info);
		fb_dealloc_cmap(&info->cmap);
		kfree(info->pseudo_palette);
		vfree(vmem);
		devm_kfree(&pdev->dev, lcd);
		dev_err(&pdev->dev, "Error: ret = %d\n", ret);
		return ret;
	}

	dev_info(&pdev->dev, "The ILI9486 driver probed\n");
	return 0;
}

static int ili9486_remove(struct platform_device *pdev)
{
	struct ili9486_data *lcd = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &lcd->dev_attr_power);

	ili9486_write_command(lcd, ILI9486_DISPOFF);

	//unregister_framebuffer(lcd->lcd_info);
	fb_deferred_io_cleanup(lcd->lcd_info);
	fb_dealloc_cmap(&lcd->lcd_info->cmap);
	kfree(lcd->lcd_info->pseudo_palette);
	vfree(lcd->lcd_info->screen_base);

	framebuffer_release(lcd->lcd_info);

	dev_info(&pdev->dev, "The ILI9486 driver removed\n");
	return 0;
}

static const struct of_device_id ili9486_ids[] = {
	{ .compatible = "ilitek, ili9486", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ili9486_ids);

static struct platform_driver ili9486_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_NAME,
		.of_match_table = of_match_ptr(ili9486_ids),
	},
	.probe = ili9486_probe,
	.remove = ili9486_remove,
};

module_platform_driver(ili9486_driver);

MODULE_AUTHOR("Dmitry Domnin");
MODULE_DESCRIPTION("A parallel ILI9486 lcd module");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
