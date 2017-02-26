/*
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/*
 * Copyright (c) 2015 Exar Corporation, Inc.
 *
 * This driver will work with any USB UART function in these Exar devices:
 *	XR21V1410/1412/1414
 *	XR21B1411
 *	XR21B1420/1422/1424
 *	XR22801/802/804
 *
 * The driver has been tested on various kernel versions from 3.6.x to 3.17.x.
 * This driver may work on newer versions as well.  There is a different driver
 * available from www.exar.com that will work with kernel versions 2.6.18 to
 * 3.4.x.
 *
 * ChangeLog:
 *            Version 1B - Initial released version.
 */

//#undef DEBUG
#undef VERBOSE_DEBUG

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/usb/cdc.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>
#include <linux/list.h>
#include "linux/version.h"

#include "exar.h"


#define DRIVER_AUTHOR "<uarttechsupport@exar.com>"
#define DRIVER_DESC "Exar USB UART (serial port) driver"

static struct usb_driver xr_usb_serial_driver;
static struct tty_driver *xr_usb_serial_tty_driver;
static struct xr_usb_serial *xr_usb_serial_table[XR_USB_SERIAL_TTY_MINORS];

static DEFINE_MUTEX(xr_usb_serial_table_lock);

/*
 * xr_usb_serial_table accessors
 */

/*
 * Look up an XR_USB_SERIAL structure by index. If found and not disconnected,
 * increment its refcount and return it with its mutex held.
 */
static struct xr_usb_serial *xr_usb_serial_get_by_index(unsigned int index)
{
	struct xr_usb_serial *xr;

	mutex_lock(&xr_usb_serial_table_lock);
	xr = xr_usb_serial_table[index];
	if (xr) {
		mutex_lock(&xr->mutex);
		if (xr->disconnected) {
			mutex_unlock(&xr->mutex);
			xr = NULL;
		} else {
			tty_port_get(&xr->port);
			mutex_unlock(&xr->mutex);
		}
	}
	mutex_unlock(&xr_usb_serial_table_lock);
	return xr;
}

/*
 * Try to find an available minor number and if found, associate it with
 * 'xr_usb_serial'.
 */
static int xr_usb_serial_alloc_minor(struct xr_usb_serial *xr)
{
	int minor;

	mutex_lock(&xr_usb_serial_table_lock);
	for (minor = 0; minor < XR_USB_SERIAL_TTY_MINORS; minor++) {
		if (!xr_usb_serial_table[minor]) {
			xr_usb_serial_table[minor] = xr;
			break;
		}
	}
	mutex_unlock(&xr_usb_serial_table_lock);

	return minor;
}

/* Release the minor number associated with 'xr_usb_serial'.  */
static void xr_usb_serial_release_minor(struct xr_usb_serial *xr)
{
	mutex_lock(&xr_usb_serial_table_lock);
	xr_usb_serial_table[xr->minor] = NULL;
	mutex_unlock(&xr_usb_serial_table_lock);
}

/*
 * Functions for XR_USB_SERIAL control messages.
 */
static int xr_usb_serial_ctrl_msg(struct xr_usb_serial *xr,
				  int request, int value,
				  void *buf, int len)
{
	int if_num = xr->control->altsetting[0].desc.bInterfaceNumber;
	int retval = usb_control_msg(xr->dev, usb_sndctrlpipe(xr->dev, 0),
				     request, USB_RT_XR_USB_SERIAL, value,
				     if_num, buf, len, 5000);

	dev_dbg(&xr->control->dev,
		"%s - rq 0x%02x, val %#x, len %#x, result %d\n",
		__func__, request, value, len, retval);
	return retval < 0 ? retval : 0;
}

#define XR_SET_MAP_XR2280X              5
#define XR_GET_MAP_XR2280X              5

#define XR_SET_MAP_XR21B142X             0
#define XR_GET_MAP_XR21B142X             0

#define XR_SET_MAP_XR21V141X             0
#define XR_GET_MAP_XR21V141X             1

#define XR_SET_MAP_XR21B1411             0
#define XR_GET_MAP_XR21B1411             1


static int set_reg(struct xr_usb_serial *xr, int regnum, int value)
{
	int result;
	int channel = 0;

	dev_dbg(&xr->control->dev, "%s Channel:%d 0x%02x = 0x%02x\n",
		__func__, channel, regnum, value);
	if ((xr->DeviceProduct & 0xfff0) == 0x1400) {
		int XR2280xaddr = XR2280x_FUNC_MGR_OFFSET + regnum;

		result = usb_control_msg(xr->dev,
					 usb_sndctrlpipe(xr->dev, 0),
					 XR_SET_MAP_XR2280X,
					 USB_DIR_OUT | USB_TYPE_VENDOR,
					 value,
					 XR2280xaddr,
					 NULL,
					 0,
					 5000);
	} else if ((xr->DeviceProduct == 0x1410) ||
		   (xr->DeviceProduct == 0x1412) ||
		   (xr->DeviceProduct == 0x1414)) {
		if (xr->channel)
			channel = xr->channel - 1;
		result = usb_control_msg(xr->dev,
					 usb_sndctrlpipe(xr->dev, 0),
					 XR_SET_MAP_XR21V141X,
					 USB_DIR_OUT | USB_TYPE_VENDOR,
					 value,
					 regnum | (channel << 8),
					 NULL,
					 0,
					 5000);
	} else if (xr->DeviceProduct == 0x1411) {
		result = usb_control_msg(xr->dev,
					 usb_sndctrlpipe(xr->dev, 0),
					 XR_SET_MAP_XR21B1411,
					 USB_DIR_OUT | USB_TYPE_VENDOR,
					 value,
					 regnum,
					 NULL,
					 0,
					 5000);
	} else if ((xr->DeviceProduct == 0x1420) ||
		   (xr->DeviceProduct == 0x1422) ||
		   (xr->DeviceProduct == 0x1424)) {
		channel = (xr->channel - 4) * 2;
		result = usb_control_msg(xr->dev,
					 usb_sndctrlpipe(xr->dev, 0),
					 XR_SET_MAP_XR21B142X,
					 USB_DIR_OUT | USB_TYPE_VENDOR | 1,
					 value,
					 regnum | (channel << 8),
					 NULL,
					 0,
					 5000);
	} else {
		result = -1;
	}
	if (result < 0)
		dev_dbg(&xr->control->dev, "%s Error:%d\n", __func__, result);
	return result;
}
static int set_reg_ext(struct xr_usb_serial *xr, int channel, int regnum,
		       int value)
{
	int result;
	int XR2280xaddr = XR2280x_FUNC_MGR_OFFSET + regnum;

	dev_dbg(&xr->control->dev, "%s channel:%d 0x%02x = 0x%02x\n",
		__func__, channel, regnum, value);
	if ((xr->DeviceProduct & 0xfff0) == 0x1400) {
		result = usb_control_msg(xr->dev,
					 usb_sndctrlpipe(xr->dev, 0),
					 XR_SET_MAP_XR2280X,
					 USB_DIR_OUT | USB_TYPE_VENDOR,
					 value,
					 XR2280xaddr,
					 NULL,
					 0,
					 5000);
	} else if ((xr->DeviceProduct == 0x1410) ||
		   (xr->DeviceProduct == 0x1412) ||
		   (xr->DeviceProduct == 0x1414)) {
		result = usb_control_msg(xr->dev,
					 usb_sndctrlpipe(xr->dev, 0),
					 XR_SET_MAP_XR21V141X,
					 USB_DIR_OUT | USB_TYPE_VENDOR,
					 value,
					 regnum | (channel << 8),
					 NULL,
					 0,
					 5000);
	} else if (xr->DeviceProduct == 0x1411) {
		result = usb_control_msg(xr->dev,
					 usb_sndctrlpipe(xr->dev, 0),
					 XR_SET_MAP_XR21B1411,
					 USB_DIR_OUT | USB_TYPE_VENDOR,
					 value,
					 regnum,
					 NULL,
					 0,
					 5000);
	} else if ((xr->DeviceProduct == 0x1420) ||
		   (xr->DeviceProduct == 0x1422) ||
		   (xr->DeviceProduct == 0x1424)) {
		result = usb_control_msg(xr->dev,
					 usb_sndctrlpipe(xr->dev, 0),
					 XR_SET_MAP_XR21B142X,
					 USB_DIR_OUT | USB_TYPE_VENDOR | 1,
					 value,
					 regnum | (channel << 8),
					 NULL,
					 0,
					 5000);
	} else {
		result = -1;
	}
	if (result < 0)
		dev_dbg(&xr->control->dev, "%s Error:%d\n", __func__, result);
	return result;
}

static int get_reg(struct xr_usb_serial *xr, int regnum, short *value)
{
	int result;
	int channel = 0;

	if ((xr->DeviceProduct & 0xfff0) == 0x1400) {
		int XR2280xaddr = XR2280x_FUNC_MGR_OFFSET + regnum;

		result = usb_control_msg(xr->dev,
					 usb_rcvctrlpipe(xr->dev, 0),
					 XR_GET_MAP_XR2280X,
					 USB_DIR_IN | USB_TYPE_VENDOR,
					 0,
					 XR2280xaddr,
					 value,
					 2,
					 5000);
	} else if ((xr->DeviceProduct == 0x1410) ||
		   (xr->DeviceProduct == 0x1412) ||
		   (xr->DeviceProduct == 0x1414)) {
		if (xr->channel)
			channel = xr->channel - 1;
		result = usb_control_msg(xr->dev,
					 usb_rcvctrlpipe(xr->dev, 0),
					 XR_GET_MAP_XR21V141X,
					 USB_DIR_IN | USB_TYPE_VENDOR,
					 0,
					 regnum | (channel << 8),
					 value,
					 1,
					 5000);
	} else if (xr->DeviceProduct == 0x1411) {
		result = usb_control_msg(xr->dev,
					 usb_rcvctrlpipe(xr->dev, 0),
					 XR_GET_MAP_XR21B1411,
					 USB_DIR_IN | USB_TYPE_VENDOR,
					 0,
					 regnum,
					 value,
					 2,
					 5000);
	} else if ((xr->DeviceProduct == 0x1420) ||
		   (xr->DeviceProduct == 0x1422) ||
		   (xr->DeviceProduct == 0x1424)) {
		channel = (xr->channel - 4) * 2;
		result = usb_control_msg(xr->dev,
					 usb_rcvctrlpipe(xr->dev, 0),
					 XR_GET_MAP_XR21B142X,
					 USB_DIR_IN | USB_TYPE_VENDOR | 1,
					 0,
					 regnum | (channel << 8),
					 value,
					 2,
					 5000);
	} else {
		result = -1;
	}

	if (result < 0)
		dev_dbg(&xr->control->dev, "%s channel:%d Reg 0x%x Error:%d\n",
			__func__, channel, regnum, result);
	else
		dev_dbg(&xr->control->dev, "%s channel:%d 0x%x = 0x%04x\n",
			__func__, channel, regnum, *value);

	return result;
}


static int get_reg_ext(struct xr_usb_serial *xr, int channel, int regnum,
		       short *value)
{
	int result;
	int XR2280xaddr = XR2280x_FUNC_MGR_OFFSET + regnum;

	if ((xr->DeviceProduct & 0xfff0) == 0x1400) {
		result = usb_control_msg(xr->dev,
					 usb_rcvctrlpipe(xr->dev, 0),
					 XR_GET_MAP_XR2280X,
					 USB_DIR_IN | USB_TYPE_VENDOR,
					 0,
					 XR2280xaddr,
					 value,
					 2,
					 5000);
	} else if ((xr->DeviceProduct == 0x1410) ||
		   (xr->DeviceProduct == 0x1412) ||
		   (xr->DeviceProduct == 0x1414)) {
		unsigned char reg_value = 0;

		result = usb_control_msg(xr->dev,
					 usb_rcvctrlpipe(xr->dev, 0),
					 XR_GET_MAP_XR21V141X,
					 USB_DIR_IN | USB_TYPE_VENDOR,
					 0,
					 regnum | (channel << 8),
					 &reg_value,
					 1,
					 5000);
		dev_dbg(&xr->control->dev, "get_reg_ext reg:%x\n", reg_value);
		*value = reg_value;
	} else if (xr->DeviceProduct == 0x1411) {
		result = usb_control_msg(xr->dev,
					 usb_rcvctrlpipe(xr->dev, 0),
					 XR_GET_MAP_XR21B1411,
					 USB_DIR_IN | USB_TYPE_VENDOR,
					 0,
					 regnum | (channel << 8),
					 value,
					 2,
					 5000);
	} else if ((xr->DeviceProduct == 0x1420) ||
		   (xr->DeviceProduct == 0x1422) ||
		   (xr->DeviceProduct == 0x1424)) {
		result = usb_control_msg(xr->dev,
					 usb_rcvctrlpipe(xr->dev, 0),
					 XR_GET_MAP_XR21B142X,
					 USB_DIR_IN | USB_TYPE_VENDOR | 1,
					 0,
					 regnum | (channel << 8),
					 value,
					 2,
					 5000);
	} else {
		result = -1;
	}

	if (result < 0)
		dev_dbg(&xr->control->dev, "%s Error:%d\n", __func__, result);
	else
		dev_dbg(&xr->control->dev, "%s channel:%d 0x%x = 0x%04x\n",
			__func__, channel, regnum, *value);

	return result;
}

struct xr21v141x_baud_rate {
	unsigned int	tx;
	unsigned int	rx0;
	unsigned int	rx1;
};

static struct xr21v141x_baud_rate xr21v141x_baud_rates[] = {
	{ 0x000, 0x000, 0x000 },
	{ 0x000, 0x000, 0x000 },
	{ 0x100, 0x000, 0x100 },
	{ 0x020, 0x400, 0x020 },
	{ 0x010, 0x100, 0x010 },
	{ 0x208, 0x040, 0x208 },
	{ 0x104, 0x820, 0x108 },
	{ 0x844, 0x210, 0x884 },
	{ 0x444, 0x110, 0x444 },
	{ 0x122, 0x888, 0x224 },
	{ 0x912, 0x448, 0x924 },
	{ 0x492, 0x248, 0x492 },
	{ 0x252, 0x928, 0x292 },
	{ 0X94A, 0X4A4, 0XA52 },
	{ 0X52A, 0XAA4, 0X54A },
	{ 0XAAA, 0x954, 0X4AA },
	{ 0XAAA, 0x554, 0XAAA },
	{ 0x555, 0XAD4, 0X5AA },
	{ 0XB55, 0XAB4, 0X55A },
	{ 0X6B5, 0X5AC, 0XB56 },
	{ 0X5B5, 0XD6C, 0X6D6 },
	{ 0XB6D, 0XB6A, 0XDB6 },
	{ 0X76D, 0X6DA, 0XBB6 },
	{ 0XEDD, 0XDDA, 0X76E },
	{ 0XDDD, 0XBBA, 0XEEE },
	{ 0X7BB, 0XF7A, 0XDDE },
	{ 0XF7B, 0XEF6, 0X7DE },
	{ 0XDF7, 0XBF6, 0XF7E },
	{ 0X7F7, 0XFEE, 0XEFE },
	{ 0XFDF, 0XFBE, 0X7FE },
	{ 0XF7F, 0XEFE, 0XFFE },
	{ 0XFFF, 0XFFE, 0XFFD },
};
#define UART_CLOCK_DIVISOR_0                               0x004
#define UART_CLOCK_DIVISOR_1                               0x005
#define UART_CLOCK_DIVISOR_2                               0x006
#define UART_TX_CLOCK_MASK_0                               0x007
#define UART_TX_CLOCK_MASK_1                               0x008
#define UART_RX_CLOCK_MASK_0                               0x009
#define UART_RX_CLOCK_MASK_1                               0x00a

static int xr21v141x_set_baud_rate(struct xr_usb_serial *xr, unsigned int rate)
{
	unsigned int divisor = 48000000 / rate;
	unsigned int i = ((32 * 48000000) / rate) & 0x1f;
	unsigned int tx_mask = xr21v141x_baud_rates[i].tx;
	unsigned int rx_mask = (divisor & 1) ? xr21v141x_baud_rates[i].rx1 :
					       xr21v141x_baud_rates[i].rx0;

	dev_dbg(&xr->control->dev, "Setting baud rate to %d: i=%u div=%u tx=%03x rx=%03x\n",
		rate, i, divisor, tx_mask, rx_mask);

	set_reg(xr, UART_CLOCK_DIVISOR_0, (divisor >> 0) & 0xff);
	set_reg(xr, UART_CLOCK_DIVISOR_1, (divisor >> 8) & 0xff);
	set_reg(xr, UART_CLOCK_DIVISOR_2, (divisor >> 16) & 0xff);
	set_reg(xr, UART_TX_CLOCK_MASK_0, (tx_mask >> 0) & 0xff);
	set_reg(xr, UART_TX_CLOCK_MASK_1, (tx_mask >> 8) & 0xff);
	set_reg(xr, UART_RX_CLOCK_MASK_0, (rx_mask >> 0) & 0xff);
	set_reg(xr, UART_RX_CLOCK_MASK_1, (rx_mask >> 8) & 0xff);

	return 0;
}
/* devices aren't required to support these requests.
 * the cdc xr_usb_serial descriptor tells whether they do...
 */
static int set_control(struct xr_usb_serial *xr, unsigned int control)
{
	int ret = 0;

	if ((xr->DeviceProduct == 0x1410) ||
	    (xr->DeviceProduct == 0x1412) ||
	    (xr->DeviceProduct == 0x1414)) {
		if (control & XR_USB_SERIAL_CTRL_DTR)
			set_reg(xr, xr->reg_map.uart_gpio_clr_addr, 0x08);
		else
			set_reg(xr, xr->reg_map.uart_gpio_set_addr, 0x08);

		if (control & XR_USB_SERIAL_CTRL_RTS)
			set_reg(xr, xr->reg_map.uart_gpio_clr_addr, 0x20);
		else
			set_reg(xr, xr->reg_map.uart_gpio_set_addr, 0x20);
	} else {
		ret = xr_usb_serial_ctrl_msg(xr,
					     USB_CDC_REQ_SET_CONTROL_LINE_STATE,
					     control, NULL, 0);
	}

	return ret;
}

static int set_line(struct xr_usb_serial *xr, struct usb_cdc_line_coding *line)
{
	int ret = 0;
	unsigned int format_size;
	unsigned int format_parity;
	unsigned int format_stop;

	if ((xr->DeviceProduct == 0x1410) ||
	    (xr->DeviceProduct == 0x1412) ||
	    (xr->DeviceProduct == 0x1414)) {
		xr21v141x_set_baud_rate(xr, line->dwDTERate);
		format_size = line->bDataBits;
		format_parity = line->bParityType;
		format_stop = line->bCharFormat;
		set_reg(xr, xr->reg_map.uart_format_addr,
			(format_size << 0) |
			(format_parity << 4) |
			(format_stop << 7));
	} else {
		ret = xr_usb_serial_ctrl_msg(xr, USB_CDC_REQ_SET_LINE_CODING, 0,
					     line, sizeof *(line));
	}
	return ret;
}
static int set_flow_mode(struct xr_usb_serial *xr, struct tty_struct *tty,
			 unsigned int cflag)
{
	unsigned int flow;
	unsigned int gpio_mode;

	if (cflag & CRTSCTS) {
		dev_dbg(&xr->control->dev, "xr_usb_serial_set_flow_mode:hardware\n");
		flow = UART_FLOW_MODE_HW;
		gpio_mode = UART_GPIO_MODE_SEL_RTS_CTS;
	} else if (I_IXOFF(tty) || I_IXON(tty)) {
		unsigned char start_char = START_CHAR(tty);
		unsigned char stop_char = STOP_CHAR(tty);

		dev_dbg(&xr->control->dev, "xr_usb_serial_set_flow_mode:software\n");
		flow = UART_FLOW_MODE_SW;
		gpio_mode = UART_GPIO_MODE_SEL_GPIO;

		set_reg(xr, xr->reg_map.uart_xon_char_addr, start_char);
		set_reg(xr, xr->reg_map.uart_xoff_char_addr, stop_char);
	} else {
		dev_dbg(&xr->control->dev, "xr_usb_serial_set_flow_mode:none\n");
		flow = UART_FLOW_MODE_NONE;
		gpio_mode = UART_GPIO_MODE_SEL_GPIO;
	}

	// Add support for the TXT and RXT function for 0x1420, 0x1422, 0x1424,
	// by setting GPIO_MODE [9:8] = '11'
	if ((xr->DeviceProduct == 0x1420) ||
	    (xr->DeviceProduct == 0x1422) ||
	    (xr->DeviceProduct == 0x1424))
		gpio_mode |= 0x300;

	set_reg(xr, xr->reg_map.uart_flow_addr, flow);
	set_reg(xr, xr->reg_map.uart_gpio_mode_addr, gpio_mode);
	return 0;
}

static int send_break(struct xr_usb_serial *xr, int state)
{
	int ret = 0;

	if ((xr->DeviceProduct == 0x1410) ||
	    (xr->DeviceProduct == 0x1412) ||
	    (xr->DeviceProduct == 0x1414)) {
		if (state)
			ret = set_reg(xr, xr->reg_map.tx_break_addr, 0xffff);
		else
			ret = set_reg(xr, xr->reg_map.tx_break_addr, 0);
	} else {
		ret = xr_usb_serial_ctrl_msg(xr, USB_CDC_REQ_SEND_BREAK, state,
					     NULL, 0);
	}
	return ret;
}

#define URM_REG_BLOCK           4
#define URM_ENABLE_BASE        0x010
#define URM_ENABLE_0_TX        0x001
#define URM_ENABLE_0_RX        0x002
#define URM_RESET_RX_FIFO_BASE        0x018
#define URM_RESET_TX_FIFO_BASE        0x01C



static int device_enable(struct xr_usb_serial *xr)
{
	int ret = 0;
	int channel = xr->channel;

	if ((xr->DeviceProduct == 0x1410) ||
	    (xr->DeviceProduct == 0x1412) ||
	    (xr->DeviceProduct == 0x1414)) {
		ret = set_reg_ext(xr, URM_REG_BLOCK, URM_ENABLE_BASE + channel,
				  URM_ENABLE_0_TX);
		ret = set_reg(xr, xr->reg_map.uart_enable_addr,
			      UART_ENABLE_TX | UART_ENABLE_RX);
		ret = set_reg_ext(xr, URM_REG_BLOCK, URM_ENABLE_BASE + channel,
				  URM_ENABLE_0_TX | URM_ENABLE_0_RX);
	} else {
		ret = set_reg(xr, xr->reg_map.uart_enable_addr,
			      UART_ENABLE_TX | UART_ENABLE_RX);
	}

	return ret;
}
static int fifo_reset(struct xr_usb_serial *xr)
{
	int ret = 0;
	int channel = xr->channel;

	if (channel)
		channel--;
	if ((xr->DeviceProduct == 0x1410) ||
	    (xr->DeviceProduct == 0x1412) ||
	    (xr->DeviceProduct == 0x1414)) {
		ret = set_reg_ext(xr, URM_REG_BLOCK,
				  URM_RESET_RX_FIFO_BASE + channel, 0xff);
		ret |= set_reg_ext(xr, URM_REG_BLOCK,
				   URM_RESET_TX_FIFO_BASE + channel, 0xff);
	}
	return ret;
}


static int device_disable(struct xr_usb_serial *xr)
{
	int ret = 0;
	int channel = xr->channel;

	ret = set_reg(xr, xr->reg_map.uart_enable_addr, 0);
	if ((xr->DeviceProduct == 0x1410) ||
	    (xr->DeviceProduct == 0x1412) ||
	    (xr->DeviceProduct == 0x1414)) {
		ret = set_reg_ext(xr, URM_REG_BLOCK, URM_ENABLE_BASE + channel,
				  URM_ENABLE_0_TX);
	}

	return ret;
}
static int set_loopback(struct xr_usb_serial *xr, int channel)
{
	int ret = 0;

	device_disable(xr);
	ret = set_reg_ext(xr, channel,
					xr->reg_map.uart_loopback_addr, 0x07);
	device_enable(xr);
	return ret;
}

static int tiocmget(struct xr_usb_serial *xr)
{
	short data;
	int result;

	result = get_reg(xr, xr->reg_map.uart_gpio_status_addr, &data);
	dev_dbg(&xr->control->dev, "xr_usb_serial_tiocmget uart_gpio_status_addr:0x%04x\n",
		data);
	if (result)
		return ((data & 0x8) ? 0 : TIOCM_DTR) |
		       ((data & 0x20) ? 0 : TIOCM_RTS) |
		       ((data & 0x4) ? 0 : TIOCM_DSR) |
		       ((data & 0x1) ? 0 : TIOCM_RI) |
		       ((data & 0x2) ? 0 : TIOCM_CD) |
		       ((data & 0x10) ? 0 : TIOCM_CTS);
	else
		return -EFAULT;
}
static int tiocmset(struct xr_usb_serial *xr, unsigned int set,
		    unsigned int clear)
{
	unsigned int newctrl = 0;

	newctrl = xr->ctrlout;

	set = (set & TIOCM_DTR ? XR_USB_SERIAL_CTRL_DTR : 0) |
	      (set & TIOCM_RTS ? XR_USB_SERIAL_CTRL_RTS : 0);

	clear = (clear & TIOCM_DTR ? XR_USB_SERIAL_CTRL_DTR : 0) |
		(clear & TIOCM_RTS ? XR_USB_SERIAL_CTRL_RTS : 0);

	newctrl = (newctrl & ~clear) | set;

	if (xr->ctrlout == newctrl)
		return 0;

	xr->ctrlout = newctrl;

	if (newctrl & XR_USB_SERIAL_CTRL_DTR)
		set_reg(xr, xr->reg_map.uart_gpio_clr_addr, 0x08);
	else
		set_reg(xr, xr->reg_map.uart_gpio_set_addr, 0x08);

	if (newctrl & XR_USB_SERIAL_CTRL_RTS)
		set_reg(xr, xr->reg_map.uart_gpio_clr_addr, 0x20);
	else
		set_reg(xr, xr->reg_map.uart_gpio_set_addr, 0x20);

	return 0;
}


static struct reg_addr_map xr21b140x_reg_map;
static struct reg_addr_map xr21b1411_reg_map;
static struct reg_addr_map xr21v141x_reg_map;
static struct reg_addr_map xr21b142x_reg_map;

static void init_xr21b140x_reg_map(void)
{
	xr21b140x_reg_map.uart_enable_addr = 0x00;
	xr21b140x_reg_map.uart_format_addr = 0x05;
	xr21b140x_reg_map.uart_flow_addr = 0x06;
	xr21b140x_reg_map.uart_loopback_addr = 0x16;
	xr21b140x_reg_map.uart_xon_char_addr = 0x07;
	xr21b140x_reg_map.uart_xoff_char_addr = 0x08;
	xr21b140x_reg_map.uart_gpio_mode_addr = 0x0c;
	xr21b140x_reg_map.uart_gpio_dir_addr = 0x0d;
	xr21b140x_reg_map.uart_gpio_set_addr = 0x0e;
	xr21b140x_reg_map.uart_gpio_clr_addr = 0x0f;
	xr21b140x_reg_map.uart_gpio_status_addr = 0x10;
	xr21b140x_reg_map.tx_break_addr = 0x0a;
	xr21b140x_reg_map.uart_custom_driver = 0x41;
}

static void init_xr21b1411_reg_map(void)
{
	xr21b1411_reg_map.uart_enable_addr = 0xc00;
	xr21b1411_reg_map.uart_flow_addr = 0xc06;
	xr21b1411_reg_map.uart_loopback_addr = 0xc16;
	xr21b1411_reg_map.uart_xon_char_addr = 0xc07;
	xr21b1411_reg_map.uart_xoff_char_addr = 0xc08;
	xr21b1411_reg_map.uart_gpio_mode_addr = 0xc0c;
	xr21b1411_reg_map.uart_gpio_dir_addr = 0xc0d;
	xr21b1411_reg_map.uart_gpio_set_addr = 0xc0e;
	xr21b1411_reg_map.uart_gpio_clr_addr = 0xc0f;
	xr21b1411_reg_map.uart_gpio_status_addr = 0xc10;
	xr21b1411_reg_map.tx_break_addr = 0xc0a;
	xr21b1411_reg_map.uart_custom_driver = 0x20d;
}

static void init_xr21v141x_reg_map(void)
{
	xr21v141x_reg_map.uart_enable_addr = 0x03;
	xr21v141x_reg_map.uart_format_addr = 0x0b;
	xr21v141x_reg_map.uart_flow_addr = 0x0c;
	xr21v141x_reg_map.uart_loopback_addr = 0x12;
	xr21v141x_reg_map.uart_xon_char_addr = 0x10;
	xr21v141x_reg_map.uart_xoff_char_addr = 0x11;
	xr21v141x_reg_map.uart_gpio_mode_addr = 0x1a;
	xr21v141x_reg_map.uart_gpio_dir_addr = 0x1b;
	xr21v141x_reg_map.uart_gpio_set_addr = 0x1d;
	xr21v141x_reg_map.uart_gpio_clr_addr = 0x1e;
	xr21v141x_reg_map.uart_gpio_status_addr = 0x1f;
	xr21v141x_reg_map.tx_break_addr = 0x14;
}
static void init_xr21b142x_reg_map(void)
{
	xr21b142x_reg_map.uart_enable_addr = 0x00;
	xr21b142x_reg_map.uart_flow_addr = 0x06;
	xr21b142x_reg_map.uart_loopback_addr = 0x16;
	xr21b142x_reg_map.uart_xon_char_addr = 0x07;
	xr21b142x_reg_map.uart_xoff_char_addr = 0x08;
	xr21b142x_reg_map.uart_gpio_mode_addr = 0x0c;
	xr21b142x_reg_map.uart_gpio_dir_addr = 0x0d;
	xr21b142x_reg_map.uart_gpio_set_addr = 0x0e;
	xr21b142x_reg_map.uart_gpio_clr_addr = 0x0f;
	xr21b142x_reg_map.uart_gpio_status_addr = 0x10;
	xr21b140x_reg_map.tx_break_addr = 0x0a;
	xr21b140x_reg_map.uart_custom_driver = 0x60;
	xr21b140x_reg_map.uart_low_latency = 0x46;
}

int xr_usb_serial_pre_setup(struct xr_usb_serial *xr)
{
	int ret = 0;

	init_xr21b140x_reg_map();
	init_xr21b1411_reg_map();
	init_xr21v141x_reg_map();
	init_xr21b142x_reg_map();
	if ((xr->DeviceProduct & 0xfff0) == 0x1400) {
		memcpy(&(xr->reg_map), &xr21b140x_reg_map,
		       sizeof(struct reg_addr_map));
	} else if (xr->DeviceProduct == 0x1411) {
		memcpy(&(xr->reg_map), &xr21b1411_reg_map,
		       sizeof(struct reg_addr_map));
	} else if ((xr->DeviceProduct == 0x1410) ||
		   (xr->DeviceProduct == 0x1412) ||
		   (xr->DeviceProduct == 0x1414)) {
		memcpy(&(xr->reg_map), &xr21v141x_reg_map,
		       sizeof(struct reg_addr_map));
	} else if ((xr->DeviceProduct == 0x1420) ||
		   (xr->DeviceProduct == 0x1422) ||
		   (xr->DeviceProduct == 0x1424)) {
		memcpy(&(xr->reg_map), &xr21b142x_reg_map,
		       sizeof(struct reg_addr_map));
	} else {
		ret = -1;
	}
	if (xr->reg_map.uart_custom_driver)
		set_reg(xr, xr->reg_map.uart_custom_driver, 1);

	set_reg(xr, xr->reg_map.uart_gpio_mode_addr, 0);
	set_reg(xr, xr->reg_map.uart_gpio_dir_addr, 0x28);
	set_reg(xr, xr->reg_map.uart_gpio_set_addr,
		UART_GPIO_SET_DTR | UART_GPIO_SET_RTS);

	return ret;
}

/*
 * Write buffer management.
 * All of these assume proper locks taken by the caller.
 */
static int xr_usb_serial_wb_alloc(struct xr_usb_serial *xr)
{
	int i, wbn;
	struct xr_usb_serial_wb *wb;

	wbn = 0;
	i = 0;
	for (;; ) {
		wb = &xr->wb[wbn];
		if (!wb->use) {
			wb->use = 1;
			return wbn;
		}
		wbn = (wbn + 1) % XR_USB_SERIAL_NW;
		if (++i >= XR_USB_SERIAL_NW)
			return -1;
	}
}

static int xr_usb_serial_wb_is_avail(struct xr_usb_serial *xr)
{
	int i, n;
	unsigned long flags;

	n = XR_USB_SERIAL_NW;
	spin_lock_irqsave(&xr->write_lock, flags);
	for (i = 0; i < XR_USB_SERIAL_NW; i++)
		n -= xr->wb[i].use;
	spin_unlock_irqrestore(&xr->write_lock, flags);
	return n;
}

/*
 * Finish write. Caller must hold xr->write_lock
 */
static void xr_usb_serial_write_done(struct xr_usb_serial *xr,
				     struct xr_usb_serial_wb *wb)
{
	wb->use = 0;
	xr->transmitting--;
	usb_autopm_put_interface_async(xr->control);
}

/*
 * Poke write.
 *
 * the caller is responsible for locking
 */
static int xr_usb_serial_start_wb(struct xr_usb_serial *xr,
				  struct xr_usb_serial_wb *wb)
{
	int rc;

	xr->transmitting++;

	wb->urb->transfer_buffer = wb->buf;
	wb->urb->transfer_dma = wb->dmah;
	wb->urb->transfer_buffer_length = wb->len;
	wb->urb->dev = xr->dev;

	rc = usb_submit_urb(wb->urb, GFP_ATOMIC);
	if (rc < 0) {
		dev_err(&xr->data->dev,
			"%s - usb_submit_urb(write bulk) failed: %d\n",
			__func__, rc);
		xr_usb_serial_write_done(xr, wb);
	}
	return rc;
}

/*
 * attributes exported through sysfs
 */
static ssize_t show_caps
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct xr_usb_serial *xr = usb_get_intfdata(intf);

	return sprintf(buf, "%d", xr->ctrl_caps);
}
static DEVICE_ATTR(bmCapabilities, 0444, show_caps, NULL);

static ssize_t show_country_codes
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct xr_usb_serial *xr = usb_get_intfdata(intf);

	memcpy(buf, xr->country_codes, xr->country_code_size);
	return xr->country_code_size;
}

static DEVICE_ATTR(wCountryCodes, 0444, show_country_codes, NULL);

static ssize_t show_country_rel_date
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct xr_usb_serial *xr = usb_get_intfdata(intf);

	return sprintf(buf, "%d", xr->country_rel_date);
}

static DEVICE_ATTR(iCountryCodeRelDate, 0444, show_country_rel_date, NULL);
/*
 * Interrupt handlers for various XR_USB_SERIAL device responses
 */

/* control interface reports status changes with "interrupt" transfers */
static void xr_usb_serial_ctrl_irq(struct urb *urb)
{
	struct xr_usb_serial *xr = urb->context;
	struct usb_cdc_notification *dr = urb->transfer_buffer;
	unsigned char *data;
	int newctrl;
	int retval;
	int status = urb->status;
	int i;
	unsigned char *p;

	switch (status) {
	case 0:
		p = (unsigned char *)(urb->transfer_buffer);
		for (i = 0; i < urb->actual_length; i++)
			dev_dbg(&xr->control->dev, "0x%02x\n", p[i]);
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dev_dbg(&xr->control->dev,
			"%s - urb shutting down with status: %d\n",
			__func__, status);
		return;
	default:
		dev_dbg(&xr->control->dev,
			"%s - nonzero urb status received: %d\n",
			__func__, status);
		goto exit;
	}

	usb_mark_last_busy(xr->dev);

	data = (unsigned char *)(dr + 1);
	switch (dr->bNotificationType) {
	case USB_CDC_NOTIFY_NETWORK_CONNECTION:
		dev_dbg(&xr->control->dev, "%s - network connection: %d\n",
			__func__, dr->wValue);
		break;

	case USB_CDC_NOTIFY_SERIAL_STATE:
		newctrl = get_unaligned_le16(data);
		if (!xr->clocal &&
		    (xr->ctrlin & ~newctrl & XR_USB_SERIAL_CTRL_DCD)) {
			dev_dbg(&xr->control->dev, "%s - calling hangup\n",
				__func__);
			tty_port_tty_hangup(&xr->port, false);
		}
		xr->ctrlin = newctrl;

		dev_dbg(&xr->control->dev,
			"%s - input control lines: dcd%c dsr%c break%c ring%c framing%c parity%c overrun%c\n",
			__func__,
			xr->ctrlin & XR_USB_SERIAL_CTRL_DCD ? '+' : '-',
			xr->ctrlin & XR_USB_SERIAL_CTRL_DSR ? '+' : '-',
			xr->ctrlin & XR_USB_SERIAL_CTRL_BRK ? '+' : '-',
			xr->ctrlin & XR_USB_SERIAL_CTRL_RI  ? '+' : '-',
			xr->ctrlin & XR_USB_SERIAL_CTRL_FRAMING ? '+' : '-',
			xr->ctrlin & XR_USB_SERIAL_CTRL_PARITY ? '+' : '-',
			xr->ctrlin & XR_USB_SERIAL_CTRL_OVERRUN ? '+' : '-');
		break;

	default:
		dev_dbg(&xr->control->dev,
			"%s - unknown notification %d received: index %d len %d data0 %d data1 %d\n",
			__func__,
			dr->bNotificationType, dr->wIndex,
			dr->wLength, data[0], data[1]);
		break;
	}
exit:
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval)
		dev_err(&xr->control->dev, "%s - usb_submit_urb failed: %d\n",
			__func__, retval);
}

static int xr_usb_serial_submit_read_urb(struct xr_usb_serial *xr,
					 int index, gfp_t mem_flags)
{
	int res;

	if (!test_and_clear_bit(index, &xr->read_urbs_free))
		return 0;

	dev_vdbg(&xr->data->dev, "%s - urb %d\n", __func__, index);

	res = usb_submit_urb(xr->read_urbs[index], mem_flags);
	if (res) {
		if (res != -EPERM) {
			dev_err(&xr->data->dev,
				"%s - usb_submit_urb failed: %d\n",
				__func__, res);
		}
		set_bit(index, &xr->read_urbs_free);
		return res;
	}

	return 0;
}

static int xr_usb_serial_submit_read_urbs(struct xr_usb_serial *xr,
					  gfp_t mem_flags)
{
	int res;
	int i;

	for (i = 0; i < xr->rx_buflimit; ++i) {
		res = xr_usb_serial_submit_read_urb(xr, i, mem_flags);
		if (res)
			return res;
	}

	return 0;
}
static void xr_usb_serial_process_read_urb(struct xr_usb_serial *xr,
					   struct urb *urb)
{
	if (!urb->actual_length)
		return;
	tty_insert_flip_string(&xr->port, urb->transfer_buffer,
			       urb->actual_length);
	tty_flip_buffer_push(&xr->port);
}

static void xr_usb_serial_read_bulk_callback(struct urb *urb)
{
	struct xr_usb_serial_rb *rb = urb->context;
	struct xr_usb_serial *xr = rb->instance;
	unsigned long flags;

	dev_vdbg(&xr->data->dev, "%s - urb %d, len %d\n", __func__,
		 rb->index, urb->actual_length);
	set_bit(rb->index, &xr->read_urbs_free);

	if (!xr->dev) {
		dev_dbg(&xr->data->dev, "%s - disconnected\n", __func__);
		return;
	}
	usb_mark_last_busy(xr->dev);

	if (urb->status) {
		dev_dbg(&xr->data->dev, "%s - non-zero urb status: %d\n",
			__func__, urb->status);
		return;
	}
	xr_usb_serial_process_read_urb(xr, urb);

	/* throttle device if requested by tty */
	spin_lock_irqsave(&xr->read_lock, flags);
	xr->throttled = xr->throttle_req;
	if (!xr->throttled && !xr->susp_count) {
		spin_unlock_irqrestore(&xr->read_lock, flags);
		xr_usb_serial_submit_read_urb(xr, rb->index, GFP_ATOMIC);
	} else {
		spin_unlock_irqrestore(&xr->read_lock, flags);
	}
}

/* data interface wrote those outgoing bytes */
static void xr_usb_serial_write_bulk(struct urb *urb)
{
	struct xr_usb_serial_wb *wb = urb->context;
	struct xr_usb_serial *xr = wb->instance;
	unsigned long flags;

	if (urb->status ||
	    (urb->actual_length != urb->transfer_buffer_length)) {
		dev_vdbg(&xr->data->dev, "%s - len %d/%d, status %d\n",
			 __func__,
			 urb->actual_length,
			 urb->transfer_buffer_length,
			 urb->status);
	}

	spin_lock_irqsave(&xr->write_lock, flags);
	xr_usb_serial_write_done(xr, wb);
	spin_unlock_irqrestore(&xr->write_lock, flags);
	schedule_work(&xr->work);
}

static void xr_usb_serial_softint(struct work_struct *work)
{
	struct xr_usb_serial *xr =
		container_of(work, struct xr_usb_serial, work);

	dev_vdbg(&xr->data->dev, "%s\n", __func__);
	tty_port_tty_wakeup(&xr->port);
}

/*
 * TTY handlers
 */
static int xr_usb_serial_tty_install(struct tty_driver *driver,
				     struct tty_struct *tty)
{
	struct xr_usb_serial *xr;
	int retval;

	dev_dbg(tty->dev, "%s\n", __func__);

	xr = xr_usb_serial_get_by_index(tty->index);
	if (!xr)
		return -ENODEV;

	retval = tty_standard_install(driver, tty);
	if (retval)
		goto error_init_termios;

	tty->driver_data = xr;

	return 0;

error_init_termios:
	tty_port_put(&xr->port);
	return retval;
}

static int xr_usb_serial_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct xr_usb_serial *xr = tty->driver_data;
	int result;

	result = fifo_reset(xr);
	dev_dbg(tty->dev, "%s\n", __func__);

	return tty_port_open(&xr->port, tty, filp);
}

static int xr_usb_serial_port_activate(struct tty_port *port,
				       struct tty_struct *tty)
{
	struct xr_usb_serial *xr =
		container_of(port, struct xr_usb_serial, port);
	int retval = -ENODEV;

	dev_dbg(&xr->control->dev, "%s\n", __func__);

	mutex_lock(&xr->mutex);
	if (xr->disconnected)
		goto disconnected;

	retval = usb_autopm_get_interface(xr->control);
	if (retval)
		goto error_get_interface;

	/*
	 * FIXME: Why do we need this? Allocating 64K of physically contiguous
	 * memory is really nasty...
	 */
	set_bit(TTY_NO_WRITE_SPLIT, &tty->flags);
	xr->control->needs_remote_wakeup = 1;

	xr->ctrlurb->dev = xr->dev;
	if (usb_submit_urb(xr->ctrlurb, GFP_KERNEL)) {
		dev_err(&xr->control->dev,
			"%s - usb_submit_urb(ctrl irq) failed\n", __func__);
		goto error_submit_urb;
	}

	xr->ctrlout = XR_USB_SERIAL_CTRL_DTR | XR_USB_SERIAL_CTRL_RTS;
	if (set_control(xr, xr->ctrlout) < 0 &&
	    (xr->ctrl_caps & USB_CDC_CAP_LINE))
		goto error_set_control;

	usb_autopm_put_interface(xr->control);

	/*
	 * Unthrottle device in case the TTY was closed while throttled.
	 */
	spin_lock_irq(&xr->read_lock);
	xr->throttled = 0;
	xr->throttle_req = 0;
	spin_unlock_irq(&xr->read_lock);

	if (xr_usb_serial_submit_read_urbs(xr, GFP_KERNEL))
		goto error_submit_read_urbs;

	mutex_unlock(&xr->mutex);

	return 0;

error_submit_read_urbs:
	xr->ctrlout = 0;
	set_control(xr, xr->ctrlout);
error_set_control:
	usb_kill_urb(xr->ctrlurb);
error_submit_urb:
	usb_autopm_put_interface(xr->control);
error_get_interface:
disconnected:
	mutex_unlock(&xr->mutex);
	return retval;
}

static void xr_usb_serial_port_destruct(struct tty_port *port)
{
	struct xr_usb_serial *xr =
		container_of(port, struct xr_usb_serial, port);

	dev_dbg(&xr->control->dev, "%s\n", __func__);
	xr_usb_serial_release_minor(xr);
	usb_put_intf(xr->control);
	kfree(xr->country_codes);
	kfree(xr);
}

static void xr_usb_serial_port_shutdown(struct tty_port *port)
{
	struct xr_usb_serial *xr =
		container_of(port, struct xr_usb_serial, port);
	int i;

	dev_dbg(&xr->control->dev, "%s\n", __func__);

	mutex_lock(&xr->mutex);
	if (!xr->disconnected) {
		usb_autopm_get_interface(xr->control);
		set_control(xr, xr->ctrlout = 0);
		usb_kill_urb(xr->ctrlurb);
		for (i = 0; i < XR_USB_SERIAL_NW; i++)
			usb_kill_urb(xr->wb[i].urb);
		for (i = 0; i < xr->rx_buflimit; i++)
			usb_kill_urb(xr->read_urbs[i]);
		xr->control->needs_remote_wakeup = 0;
		usb_autopm_put_interface(xr->control);
	}
	mutex_unlock(&xr->mutex);
}

static void xr_usb_serial_tty_cleanup(struct tty_struct *tty)
{
	struct xr_usb_serial *xr = tty->driver_data;

	dev_dbg(&xr->control->dev, "%s\n", __func__);
	tty_port_put(&xr->port);
}

static void xr_usb_serial_tty_hangup(struct tty_struct *tty)
{
	struct xr_usb_serial *xr = tty->driver_data;

	dev_dbg(&xr->control->dev, "%s\n", __func__);
	tty_port_hangup(&xr->port);
}

static void xr_usb_serial_tty_close(struct tty_struct *tty, struct file *filp)
{
	struct xr_usb_serial *xr = tty->driver_data;

	dev_dbg(&xr->control->dev, "%s\n", __func__);
	tty_port_close(&xr->port, tty, filp);
}

static int xr_usb_serial_tty_write(struct tty_struct *tty,
				   const unsigned char *buf, int count)
{
	struct xr_usb_serial *xr = tty->driver_data;
	int stat;
	unsigned long flags;
	int wbn;
	struct xr_usb_serial_wb *wb;

	if (!count)
		return 0;

	dev_vdbg(&xr->data->dev, "%s - count %d\n", __func__, count);

	spin_lock_irqsave(&xr->write_lock, flags);
	wbn = xr_usb_serial_wb_alloc(xr);
	if (wbn < 0) {
		spin_unlock_irqrestore(&xr->write_lock, flags);
		return 0;
	}
	wb = &xr->wb[wbn];

	if (!xr->dev) {
		wb->use = 0;
		spin_unlock_irqrestore(&xr->write_lock, flags);
		return -ENODEV;
	}

	count = (count > xr->writesize) ? xr->writesize : count;
	dev_vdbg(&xr->data->dev, "%s - write %d\n", __func__, count);
	memcpy(wb->buf, buf, count);
	wb->len = count;

	usb_autopm_get_interface_async(xr->control);
	if (xr->susp_count) {
		if (!xr->delayed_wb)
			xr->delayed_wb = wb;
		else
			usb_autopm_put_interface_async(xr->control);
		spin_unlock_irqrestore(&xr->write_lock, flags);
		return count;   /* A white lie */
	}
	usb_mark_last_busy(xr->dev);

	stat = xr_usb_serial_start_wb(xr, wb);
	spin_unlock_irqrestore(&xr->write_lock, flags);

	if (stat < 0)
		return stat;
	return count;
}

static int xr_usb_serial_tty_write_room(struct tty_struct *tty)
{
	struct xr_usb_serial *xr = tty->driver_data;

	/*
	 * Do not let the line discipline to know that we have a reserve,
	 * or it might get too enthusiastic.
	 */
	return xr_usb_serial_wb_is_avail(xr) ? xr->writesize : 0;
}

static int xr_usb_serial_tty_chars_in_buffer(struct tty_struct *tty)
{
	struct xr_usb_serial *xr = tty->driver_data;

	/*
	 * if the device was unplugged then any remaining characters fell out
	 * of the connector ;)
	 */
	if (xr->disconnected)
		return 0;
	/*
	 * This is inaccurate (overcounts), but it works.
	 */
	return (XR_USB_SERIAL_NW - xr_usb_serial_wb_is_avail(xr)) *
	       xr->writesize;
}

static void xr_usb_serial_tty_throttle(struct tty_struct *tty)
{
	struct xr_usb_serial *xr = tty->driver_data;

	spin_lock_irq(&xr->read_lock);
	xr->throttle_req = 1;
	spin_unlock_irq(&xr->read_lock);
}

static void xr_usb_serial_tty_unthrottle(struct tty_struct *tty)
{
	struct xr_usb_serial *xr = tty->driver_data;
	unsigned int was_throttled;

	spin_lock_irq(&xr->read_lock);
	was_throttled = xr->throttled;
	xr->throttled = 0;
	xr->throttle_req = 0;
	spin_unlock_irq(&xr->read_lock);

	if (was_throttled)
		xr_usb_serial_submit_read_urbs(xr, GFP_KERNEL);
}

static int xr_usb_serial_tty_break_ctl(struct tty_struct *tty, int state)
{
	struct xr_usb_serial *xr = tty->driver_data;
	int retval;

	retval = send_break(xr, state ? 0xffff : 0);
	if (retval < 0)
		dev_dbg(&xr->control->dev, "%s - send break failed\n",
			__func__);
	return retval;
}

static int xr_usb_serial_tty_tiocmget(struct tty_struct *tty)
{
	struct xr_usb_serial *xr = tty->driver_data;

	dev_dbg(&xr->control->dev, "xr_usb_serial_tty_tiocmget\n");
	return tiocmget(xr);
}

static int xr_usb_serial_tty_tiocmset(struct tty_struct *tty,
				      unsigned int set, unsigned int clear)
{
	struct xr_usb_serial *xr = tty->driver_data;

	dev_dbg(&xr->control->dev, "xr_usb_serial_tty_tiocmset set=0x%x clear=0x%x\n",
		set, clear);
	return tiocmset(xr, set, clear);
}

static int get_serial_info(struct xr_usb_serial *xr,
			   struct serial_struct __user *info)
{
	struct serial_struct tmp;

	if (!info)
		return -EINVAL;

	memset(&tmp, 0, sizeof(tmp));
	tmp.flags = ASYNC_LOW_LATENCY;
	tmp.xmit_fifo_size = xr->writesize;
	tmp.baud_base = le32_to_cpu(xr->line.dwDTERate);
	tmp.close_delay = xr->port.close_delay / 10;
	tmp.closing_wait = xr->port.closing_wait == ASYNC_CLOSING_WAIT_NONE ?
			   ASYNC_CLOSING_WAIT_NONE :
			   xr->port.closing_wait / 10;

	if (copy_to_user(info, &tmp, sizeof(tmp)))
		return -EFAULT;
	else
		return 0;
}

static int set_serial_info(struct xr_usb_serial *xr,
			   struct serial_struct __user *newinfo)
{
	struct serial_struct new_serial;
	unsigned int closing_wait, close_delay;
	int retval = 0;

	if (copy_from_user(&new_serial, newinfo, sizeof(new_serial)))
		return -EFAULT;

	close_delay = new_serial.close_delay * 10;
	closing_wait = new_serial.closing_wait == ASYNC_CLOSING_WAIT_NONE ?
		       ASYNC_CLOSING_WAIT_NONE : new_serial.closing_wait * 10;

	mutex_lock(&xr->port.mutex);

	if (!capable(CAP_SYS_ADMIN)) {
		if ((close_delay != xr->port.close_delay) ||
		    (closing_wait != xr->port.closing_wait))
			retval = -EPERM;
		else
			retval = -EOPNOTSUPP;
	} else {
		xr->port.close_delay = close_delay;
		xr->port.closing_wait = closing_wait;
	}

	mutex_unlock(&xr->port.mutex);
	return retval;
}

static int xr_usb_serial_tty_ioctl(struct tty_struct *tty,
				   unsigned int cmd, unsigned long arg)
{
	struct xr_usb_serial *xr = tty->driver_data;
	int rv = -ENOIOCTLCMD;
	unsigned int channel, reg, val;
	int baud_rate = 0;
	struct usb_cdc_line_coding newline;
	short *data;

	switch (cmd) {
	case TIOCGSERIAL: /* gets serial port data */
		rv = get_serial_info(xr, (struct serial_struct __user *)arg);
		break;
	case TIOCSSERIAL:
		rv = set_serial_info(xr, (struct serial_struct __user *)arg);
		break;
	case XR_USB_SERIAL_GET_REG:
		if (get_user(channel, (int __user *)arg))
			return -EFAULT;
		if (get_user(reg, (int __user *)(arg + sizeof(int))))
			return -EFAULT;

		data = kmalloc(2, GFP_KERNEL);
		if (data == NULL)
			return -ENOMEM;

		if (channel == -1)
			rv = get_reg(xr, reg, data);
		else
			rv = get_reg_ext(xr, channel, reg, data);
		if (rv < 0) {
			dev_err(&xr->control->dev, "Cannot get register (%d)\n",
				rv);
			kfree(data);
			return -EFAULT;
		}
		if (put_user(le16_to_cpu(*data),
			     (int __user *)(arg + 2 * sizeof(int)))) {
			dev_err(&xr->control->dev, "Cannot put user result\n");
			kfree(data);
			return -EFAULT;
		}
		rv = 0;
		kfree(data);
		break;

	case XR_USB_SERIAL_SET_REG:
		if (get_user(channel, (int __user *)arg))
			return -EFAULT;
		if (get_user(reg, (int __user *)(arg + sizeof(int))))
			return -EFAULT;
		if (get_user(val, (int __user *)(arg + 2 * sizeof(int))))
			return -EFAULT;

		if (channel == -1)
			rv = set_reg(xr, reg, val);
		else
			rv = set_reg_ext(xr, channel, reg, val);

		if (rv < 0)
			return -EFAULT;
		rv = 0;
		break;
	case XR_USB_SERIAL_LOOPBACK:
		if (get_user(channel, (int __user *)arg))
			return -EFAULT;
		if (channel == -1)
			channel = xr->channel;
		rv = set_loopback(xr, channel);
		if (rv < 0)
			return -EFAULT;
		rv = 0;
		break;
	case XR_USB_SERIAL_SET_GPIO_MODE_REG:
		device_disable(xr);
		if (get_user(channel, (int __user *)arg))
			return -EFAULT;
		if (get_user(val, (int __user *)(arg + sizeof(int))))
			return -EFAULT;
		if (channel == -1) {
			//block = portdata->block;
			rv = set_reg(xr,
				     xr->reg_map.uart_gpio_mode_addr, val);
		} else {
			rv = set_reg_ext(xr, channel,
					 xr->reg_map.uart_gpio_mode_addr, val);
		}

		dev_dbg(&xr->control->dev, "XR_USB_SERIAL_SET_GPIO_MODE_REG 0x%x val:0x%x\n",
			xr->reg_map.uart_gpio_mode_addr, val);
		device_enable(xr);
		if (rv < 0)
			return -EFAULT;
		break;
	case XR_USB_SERIAL_GET_GPIO_MODE_REG:
		device_disable(xr);
		if (get_user(channel, (int __user *)arg))
			return -EFAULT;

		data = kmalloc(2, GFP_KERNEL);
		if (data == NULL)
			return -ENOMEM;

		if (channel == -1) {
			rv = get_reg(xr, xr->reg_map.uart_gpio_mode_addr, data);
		} else {
			rv = get_reg_ext(xr, channel,
					 xr->reg_map.uart_gpio_mode_addr, data);
		}

		device_enable(xr);

		dev_dbg(&xr->control->dev, "XR_USB_SERIAL_GET_GPIO_MODE_REG 0x%x val:0x%x\n",
			xr->reg_map.uart_gpio_mode_addr, *data);

		if (rv < 0) {
			dev_err(&xr->control->dev, "Cannot get register (%d) channel=%d\n",
				rv, channel);
			kfree(data);
			return -EFAULT;
		}

		if (put_user(data[0], (int __user *)(arg + sizeof(int)))) {
			dev_err(&xr->control->dev, "Cannot put user result\n");
			kfree(data);
			return -EFAULT;
		}

		kfree(data);
		break;
	case XRIOC_SET_ANY_BAUD_RATE:

		if (get_user(baud_rate, (int __user *)arg)) {
			dev_dbg(&xr->control->dev, "get_user error\n");
			return -EFAULT;
		}
		xr->line.dwDTERate = baud_rate;
		memcpy(&newline, &(xr->line),
		       sizeof(struct usb_cdc_line_coding));

		device_disable(xr);
		rv = set_line(xr, &newline);
		device_enable(xr);
		dev_dbg(&xr->control->dev, "XRIOC_SET_ANY_BAUD_RATE set baud_rate:%d ret=%d\n",
			baud_rate, rv);
		break;
	}

	return rv;
}

static void xr_usb_serial_tty_set_termios(struct tty_struct *tty,
					  struct ktermios *termios_old)
{
	struct xr_usb_serial *xr = tty->driver_data;

	struct ktermios *termios = &tty->termios;
	unsigned int cflag = termios->c_cflag;
	struct usb_cdc_line_coding newline;
	int newctrl = xr->ctrlout;

	device_disable(xr);
	newline.dwDTERate = cpu_to_le32(tty_get_baud_rate(tty));
	newline.bCharFormat = termios->c_cflag & CSTOPB ? 1 : 0;
	newline.bParityType = termios->c_cflag & PARENB ?
			      (termios->c_cflag & PARODD ? 1 : 2) +
			      (termios->c_cflag & CMSPAR ? 2 : 0) : 0;
	switch (termios->c_cflag & CSIZE) {
	case CS5:/*using CS5 replace of the 9 bit data mode*/
		newline.bDataBits = 9;
		break;
	case CS6:
		newline.bDataBits = 6;
		break;
	case CS7:
		newline.bDataBits = 7;
		break;
	case CS8:
	default:
		newline.bDataBits = 8;
		break;
	}
	/* FIXME: Needs to clear unsupported bits in the termios */
	xr->clocal = ((termios->c_cflag & CLOCAL) != 0);

	if (!newline.dwDTERate) {
		newline.dwDTERate = xr->line.dwDTERate;
		newctrl &= ~XR_USB_SERIAL_CTRL_DTR;
	} else {
		newctrl |= XR_USB_SERIAL_CTRL_DTR;
	}

	if (newctrl != xr->ctrlout)
		set_control(xr, xr->ctrlout = newctrl);

	set_flow_mode(xr, tty, cflag);/*set the serial flow mode*/

	if (memcmp(&xr->line, &newline, sizeof(newline))) {
		memcpy(&xr->line, &newline, sizeof(newline));
		dev_dbg(&xr->control->dev, "%s - set line: %d %d %d %d\n",
			__func__,
			le32_to_cpu(newline.dwDTERate),
			newline.bCharFormat, newline.bParityType,
			newline.bDataBits);
		set_line(xr, &xr->line);
	}
	device_enable(xr);
}

static const struct tty_port_operations xr_usb_serial_port_ops = {
	.shutdown	= xr_usb_serial_port_shutdown,
	.activate	= xr_usb_serial_port_activate,
	.destruct	= xr_usb_serial_port_destruct,
};

/*
 * USB probe and disconnect routines.
 */

/* Little helpers: write/read buffers free */
static void xr_usb_serial_write_buffers_free(struct xr_usb_serial *xr)
{
	int i;
	struct xr_usb_serial_wb *wb;
	struct usb_device *usb_dev = interface_to_usbdev(xr->control);

	for (wb = &xr->wb[0], i = 0; i < XR_USB_SERIAL_NW; i++, wb++)
		usb_free_coherent(usb_dev, xr->writesize, wb->buf, wb->dmah);
}

static void xr_usb_serial_read_buffers_free(struct xr_usb_serial *xr)
{
	struct usb_device *usb_dev = interface_to_usbdev(xr->control);
	int i;

	for (i = 0; i < xr->rx_buflimit; i++) {
		usb_free_coherent(usb_dev, xr->readsize,
				  xr->read_buffers[i].base,
				  xr->read_buffers[i].dma);
	}
}

/* Little helper: write buffers allocate */
static int xr_usb_serial_write_buffers_alloc(struct xr_usb_serial *xr)
{
	int i;
	struct xr_usb_serial_wb *wb;

	for (wb = &xr->wb[0], i = 0; i < XR_USB_SERIAL_NW; i++, wb++) {
		wb->buf = usb_alloc_coherent(xr->dev, xr->writesize, GFP_KERNEL,
					     &wb->dmah);
		if (!wb->buf) {
			while (i != 0) {
				--i;
				--wb;
				usb_free_coherent(xr->dev, xr->writesize,
						  wb->buf, wb->dmah);
			}
			return -ENOMEM;
		}
	}
	return 0;
}

static int xr_usb_serial_probe(struct usb_interface *intf,
			       const struct usb_device_id *id)
{
	struct usb_cdc_union_desc *union_header = NULL;
	struct usb_cdc_country_functional_desc *cfd = NULL;
	unsigned char *buffer = intf->altsetting->extra;
	int buflen = intf->altsetting->extralen;
	struct usb_interface *control_if;
	struct usb_interface *data_if;
	struct usb_endpoint_descriptor *epctrl = NULL;
	struct usb_endpoint_descriptor *epread = NULL;
	struct usb_endpoint_descriptor *epwrite = NULL;
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	struct xr_usb_serial *xr;
	int minor;
	int ctrlsize, readsize;
	u8 *buf;
	u8 ac_mgmt_func = 0;
	u8 call_mgmt_func = 0;
	int call_if_num = -1;
	int data_if_num = -1;
	unsigned long quirks;
	int num_rx_buf;
	int i;
	int combined_interfaces = 0;
	struct device *tty_dev;
	int rv = -ENOMEM;

	/* normal quirks */
	quirks = (unsigned long)id->driver_info;

	if (quirks == IGNORE_DEVICE)
		return -ENODEV;

	num_rx_buf = (quirks == SINGLE_RX_URB) ? 1 : XR_USB_SERIAL_NR;

	dev_dbg(&intf->dev, "USB_device_id idVendor:%04x, idProduct %04x\n",
		id->idVendor, id->idProduct);

	/* handle quirks deadly to normal probing*/
	if (quirks == NO_UNION_NORMAL) {
		data_if = usb_ifnum_to_if(usb_dev, 1);
		control_if = usb_ifnum_to_if(usb_dev, 0);
		goto skip_normal_probe;
	}

	/* normal probing*/
	if (!buffer) {
		dev_err(&intf->dev, "Weird descriptor references\n");
		return -EINVAL;
	}

	if (!buflen) {
		if (intf->cur_altsetting->endpoint &&
		    intf->cur_altsetting->endpoint->extralen &&
		    intf->cur_altsetting->endpoint->extra) {
			dev_dbg(&intf->dev,
				"Seeking extra descriptors on endpoint\n");
			buflen = intf->cur_altsetting->endpoint->extralen;
			buffer = intf->cur_altsetting->endpoint->extra;
		} else {
			dev_err(&intf->dev,
				"Zero length descriptor references\n");
			return -EINVAL;
		}
	}

	while (buflen > 0) {
		if (buffer[1] != USB_DT_CS_INTERFACE) {
			dev_err(&intf->dev, "skipping garbage\n");
			goto next_desc;
		}

		switch (buffer[2]) {
		case USB_CDC_UNION_TYPE: /* we've found it */
			if (union_header) {
				dev_err(&intf->dev, "More than one union descriptor, skipping ...\n");
				goto next_desc;
			}
			union_header = (struct usb_cdc_union_desc *)buffer;
			break;
		case USB_CDC_COUNTRY_TYPE: /* export through sysfs*/
			cfd = (struct usb_cdc_country_functional_desc *)buffer;
			break;
		case USB_CDC_HEADER_TYPE:       /* maybe check version */
			break;                  /* for now we ignore it */
		case USB_CDC_ACM_TYPE:
			ac_mgmt_func = buffer[3];
			break;
		case USB_CDC_CALL_MANAGEMENT_TYPE:
			call_mgmt_func = buffer[3];
			call_if_num = buffer[4];
			if ((quirks & NOT_A_MODEM) == 0 &&
			    (call_mgmt_func & 3) != 3)
				dev_err(&intf->dev, "This device cannot do calls on its own. It is not a modem.\n");
			break;
		default:
			/* there are LOTS more CDC descriptors that
			 * could legitimately be found here.
			 */
			dev_dbg(&intf->dev, "Ignoring descriptor: type %02x, length %d\n",
				buffer[2], buffer[0]);
			break;
		}
next_desc:
		buflen -= buffer[0];
		buffer += buffer[0];
	}

	if (!union_header) {
		if (call_if_num > 0) {
			dev_dbg(&intf->dev, "No union descriptor, using call management descriptor\n");
			/* quirks for Droids MuIn LCD */
			if (quirks & NO_DATA_INTERFACE) {
				data_if = usb_ifnum_to_if(usb_dev, 0);
			} else {
				data_if = usb_ifnum_to_if(usb_dev,
						(data_if_num = call_if_num));
			}
			control_if = intf;
		} else {
			if (intf->cur_altsetting->desc.bNumEndpoints != 3) {
				dev_dbg(&intf->dev, "No union descriptor, giving up\n");
				return -ENODEV;
			}

			dev_warn(&intf->dev, "No union descriptor, testing for castrated device\n");
			combined_interfaces = 1;
			control_if = data_if = intf;
			goto look_for_collapsed_interface;
		}
	} else {
		control_if = usb_ifnum_to_if(usb_dev,
					     union_header->bMasterInterface0);
		data_if = usb_ifnum_to_if(usb_dev,
				(data_if_num = union_header->bSlaveInterface0));
		if (!control_if || !data_if) {
			dev_dbg(&intf->dev, "no interfaces\n");
			return -ENODEV;
		}
	}

	if (data_if_num != call_if_num)
		dev_dbg(&intf->dev, "Separate call control interface. That is not fully supported.\n");

	if (control_if == data_if) {
		/* some broken devices designed for windows work this way */
		dev_warn(&intf->dev, "Control and data interfaces are not separated!\n");
		combined_interfaces = 1;
		/* a popular other OS doesn't use it */
		quirks |= NO_CAP_LINE;
		if (data_if->cur_altsetting->desc.bNumEndpoints != 3) {
			dev_err(&intf->dev, "This needs exactly 3 endpoints\n");
			return -EINVAL;
		}
look_for_collapsed_interface:
		for (i = 0; i < 3; i++) {
			struct usb_endpoint_descriptor *ep;

			ep = &data_if->cur_altsetting->endpoint[i].desc;

			if (usb_endpoint_is_int_in(ep))
				epctrl = ep;
			else if (usb_endpoint_is_bulk_out(ep))
				epwrite = ep;
			else if (usb_endpoint_is_bulk_in(ep))
				epread = ep;
			else
				return -EINVAL;
		}
		if (!epctrl || !epread || !epwrite)
			return -ENODEV;

		goto made_compressed_probe;
	}

skip_normal_probe:

	/*workaround for switched interfaces */
	if (data_if->cur_altsetting->desc.bInterfaceClass
	    != CDC_DATA_INTERFACE_TYPE) {
		if (control_if->cur_altsetting->desc.bInterfaceClass
		    == CDC_DATA_INTERFACE_TYPE) {
			struct usb_interface *t;

			dev_dbg(&intf->dev,
				"Your device has switched interfaces.\n");
			t = control_if;
			control_if = data_if;
			data_if = t;
		} else {
			return -EINVAL;
		}
	}

	/* Accept probe requests only for the control interface */
	if (!combined_interfaces && intf != control_if)
		return -ENODEV;

	if (!combined_interfaces && usb_interface_claimed(data_if)) {
		/* valid in this context */
		dev_dbg(&intf->dev, "The data interface isn't available\n");
		return -EBUSY;
	}


	if (data_if->cur_altsetting->desc.bNumEndpoints < 2 ||
	    control_if->cur_altsetting->desc.bNumEndpoints == 0)
		return -EINVAL;

	epctrl = &control_if->cur_altsetting->endpoint[0].desc;
	epread = &data_if->cur_altsetting->endpoint[0].desc;
	epwrite = &data_if->cur_altsetting->endpoint[1].desc;


	/* workaround for switched endpoints */
	if (!usb_endpoint_dir_in(epread)) {
		/* descriptors are swapped */
		struct usb_endpoint_descriptor *t;

		dev_dbg(&intf->dev,
			"The data interface has switched endpoints\n");
		t = epread;
		epread = epwrite;
		epwrite = t;
	}
made_compressed_probe:
	dev_dbg(&intf->dev, "interfaces are valid\n");

	xr = kzalloc(sizeof(struct xr_usb_serial), GFP_KERNEL);
	if (xr == NULL)
		goto alloc_fail;

	minor = xr_usb_serial_alloc_minor(xr);
	if (minor == XR_USB_SERIAL_TTY_MINORS) {
		dev_err(&intf->dev, "no more free xr_usb_serial devices\n");
		kfree(xr);
		return -ENODEV;
	}

	ctrlsize = usb_endpoint_maxp(epctrl);
	readsize = usb_endpoint_maxp(epread) *
		   (quirks == SINGLE_RX_URB ? 1 : 2);
	xr->combined_interfaces = combined_interfaces;
	xr->writesize = usb_endpoint_maxp(epwrite) * 20;
	xr->control = control_if;
	xr->data = data_if;
	xr->minor = minor;
	xr->dev = usb_dev;
	xr->ctrl_caps = ac_mgmt_func;
	if (quirks & NO_CAP_LINE)
		xr->ctrl_caps &= ~USB_CDC_CAP_LINE;
	xr->ctrlsize = ctrlsize;
	xr->readsize = readsize;
	xr->rx_buflimit = num_rx_buf;
	INIT_WORK(&xr->work, xr_usb_serial_softint);
	spin_lock_init(&xr->write_lock);
	spin_lock_init(&xr->read_lock);
	mutex_init(&xr->mutex);
	xr->rx_endpoint = usb_rcvbulkpipe(usb_dev, epread->bEndpointAddress);
	xr->is_int_ep = usb_endpoint_xfer_int(epread);
	if (xr->is_int_ep)
		xr->bInterval = epread->bInterval;
	tty_port_init(&xr->port);
	xr->port.ops = &xr_usb_serial_port_ops;
	xr->DeviceVendor = id->idVendor;
	xr->DeviceProduct = id->idProduct;
#if 0
	//map the serial port A B C D to blocknum 0 1 2 3 for xr21v141x
	if ((xr->DeviceProduct & 0xfff0) == 0x1410)
		xr->channel = epwrite->bEndpointAddress - 1;
	//map the serial port A B C D to blocknum 0 2 4 6 for xr21B142x
	else if ((xr->DeviceProduct & 0xfff0) == 0x1420)
		xr->channel = (epwrite->bEndpointAddress - 4) * 2;
	else
		xr->channel = epwrite->bEndpointAddress;

#else
	xr->channel = epwrite->bEndpointAddress;
	dev_dbg(&intf->dev, "epwrite->bEndpointAddress =%d\n",
		epwrite->bEndpointAddress);
#endif
	buf = usb_alloc_coherent(usb_dev, ctrlsize, GFP_KERNEL, &xr->ctrl_dma);
	if (!buf)
		goto alloc_fail2;

	xr->ctrl_buffer = buf;

	if (xr_usb_serial_write_buffers_alloc(xr) < 0)
		goto alloc_fail4;

	xr->ctrlurb = usb_alloc_urb(0, GFP_KERNEL);
	if (!xr->ctrlurb)
		goto alloc_fail5;

	for (i = 0; i < num_rx_buf; i++) {
		struct xr_usb_serial_rb *rb = &(xr->read_buffers[i]);
		struct urb *urb;

		rb->base = usb_alloc_coherent(xr->dev, readsize, GFP_KERNEL,
					      &rb->dma);
		if (!rb->base)
			goto alloc_fail6;

		rb->index = i;
		rb->instance = xr;

		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb)
			goto alloc_fail6;

		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		urb->transfer_dma = rb->dma;
		if (xr->is_int_ep) {
			usb_fill_int_urb(urb, xr->dev,
					 xr->rx_endpoint,
					 rb->base,
					 xr->readsize,
					 xr_usb_serial_read_bulk_callback, rb,
					 xr->bInterval);
		} else {
			usb_fill_bulk_urb(urb, xr->dev,
					  xr->rx_endpoint,
					  rb->base,
					  xr->readsize,
					  xr_usb_serial_read_bulk_callback, rb);
		}

		xr->read_urbs[i] = urb;
		__set_bit(i, &xr->read_urbs_free);
	}
	for (i = 0; i < XR_USB_SERIAL_NW; i++) {
		struct xr_usb_serial_wb *snd = &(xr->wb[i]);

		snd->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (snd->urb == NULL)
			goto alloc_fail7;

		if (usb_endpoint_xfer_int(epwrite)) {
			usb_fill_int_urb(snd->urb, usb_dev,
					 usb_sndintpipe(usb_dev,
						 epwrite->bEndpointAddress),
					 NULL, xr->writesize,
					 xr_usb_serial_write_bulk, snd,
					 epwrite->bInterval);
		} else {
			usb_fill_bulk_urb(snd->urb, usb_dev,
					  usb_sndbulkpipe(usb_dev,
						  epwrite->bEndpointAddress),
					  NULL, xr->writesize,
					  xr_usb_serial_write_bulk, snd);
		}
		snd->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		snd->instance = xr;
	}

	usb_set_intfdata(intf, xr);

	i = device_create_file(&intf->dev, &dev_attr_bmCapabilities);
	if (i < 0)
		goto alloc_fail7;

	if (cfd) { /* export the country data */
		xr->country_codes = kmalloc(cfd->bLength - 4, GFP_KERNEL);
		if (!xr->country_codes)
			goto skip_countries;
		xr->country_code_size = cfd->bLength - 4;
		memcpy(xr->country_codes, (u8 *)&cfd->wCountyCode0,
		       cfd->bLength - 4);
		xr->country_rel_date = cfd->iCountryCodeRelDate;

		i = device_create_file(&intf->dev, &dev_attr_wCountryCodes);
		if (i < 0) {
			kfree(xr->country_codes);
			xr->country_codes = NULL;
			xr->country_code_size = 0;
			goto skip_countries;
		}

		i = device_create_file(&intf->dev,
				       &dev_attr_iCountryCodeRelDate);
		if (i < 0) {
			device_remove_file(&intf->dev, &dev_attr_wCountryCodes);
			kfree(xr->country_codes);
			xr->country_codes = NULL;
			xr->country_code_size = 0;
			goto skip_countries;
		}
	}

skip_countries:
	usb_fill_int_urb(xr->ctrlurb, usb_dev,
			 usb_rcvintpipe(usb_dev, epctrl->bEndpointAddress),
			 xr->ctrl_buffer, ctrlsize, xr_usb_serial_ctrl_irq, xr,
			 /* works around buggy devices */
			 epctrl->bInterval ? epctrl->bInterval : 0xff);
	xr->ctrlurb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	xr->ctrlurb->transfer_dma = xr->ctrl_dma;

	dev_info(&intf->dev, "ttyXR_USB_SERIAL%d: USB XR_USB_SERIAL device\n",
		 minor);

	xr_usb_serial_pre_setup(xr);

	set_control(xr, xr->ctrlout);

	xr->line.dwDTERate = cpu_to_le32(9600);
	xr->line.bDataBits = 8;
	set_line(xr, &xr->line);

	usb_driver_claim_interface(&xr_usb_serial_driver, data_if, xr);
	usb_set_intfdata(data_if, xr);

	usb_get_intf(control_if);
	tty_dev = tty_port_register_device(&xr->port, xr_usb_serial_tty_driver,
					   minor, &control_if->dev);
	if (IS_ERR(tty_dev)) {
		rv = PTR_ERR(tty_dev);
		goto alloc_fail8;
	}

	return 0;
alloc_fail8:
	if (xr->country_codes) {
		device_remove_file(&xr->control->dev,
				   &dev_attr_wCountryCodes);
		device_remove_file(&xr->control->dev,
				   &dev_attr_iCountryCodeRelDate);
	}
	device_remove_file(&xr->control->dev, &dev_attr_bmCapabilities);
alloc_fail7:
	usb_set_intfdata(intf, NULL);
	for (i = 0; i < XR_USB_SERIAL_NW; i++)
		usb_free_urb(xr->wb[i].urb);
alloc_fail6:
	for (i = 0; i < num_rx_buf; i++)
		usb_free_urb(xr->read_urbs[i]);
	xr_usb_serial_read_buffers_free(xr);
	usb_free_urb(xr->ctrlurb);
alloc_fail5:
	xr_usb_serial_write_buffers_free(xr);
alloc_fail4:
	usb_free_coherent(usb_dev, ctrlsize, xr->ctrl_buffer, xr->ctrl_dma);
alloc_fail2:
	xr_usb_serial_release_minor(xr);
	kfree(xr);
alloc_fail:
	return rv;
}

static void stop_data_traffic(struct xr_usb_serial *xr)
{
	int i;

	dev_dbg(&xr->control->dev, "%s\n", __func__);

	usb_kill_urb(xr->ctrlurb);
	for (i = 0; i < XR_USB_SERIAL_NW; i++)
		usb_kill_urb(xr->wb[i].urb);
	for (i = 0; i < xr->rx_buflimit; i++)
		usb_kill_urb(xr->read_urbs[i]);

	cancel_work_sync(&xr->work);
}

static void xr_usb_serial_disconnect(struct usb_interface *intf)
{
	struct xr_usb_serial *xr = usb_get_intfdata(intf);
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	struct tty_struct *tty;
	int i;

	dev_dbg(&intf->dev, "%s\n", __func__);

	/* sibling interface is already cleaning up */
	if (!xr)
		return;

	mutex_lock(&xr->mutex);
	xr->disconnected = true;
	if (xr->country_codes) {
		device_remove_file(&xr->control->dev,
				   &dev_attr_wCountryCodes);
		device_remove_file(&xr->control->dev,
				   &dev_attr_iCountryCodeRelDate);
	}
	device_remove_file(&xr->control->dev, &dev_attr_bmCapabilities);
	usb_set_intfdata(xr->control, NULL);
	usb_set_intfdata(xr->data, NULL);
	mutex_unlock(&xr->mutex);

	tty = tty_port_tty_get(&xr->port);
	if (tty) {
		tty_vhangup(tty);
		tty_kref_put(tty);
	}
	stop_data_traffic(xr);
	tty_unregister_device(xr_usb_serial_tty_driver, xr->minor);

	usb_free_urb(xr->ctrlurb);
	for (i = 0; i < XR_USB_SERIAL_NW; i++)
		usb_free_urb(xr->wb[i].urb);
	for (i = 0; i < xr->rx_buflimit; i++)
		usb_free_urb(xr->read_urbs[i]);
	xr_usb_serial_write_buffers_free(xr);
	usb_free_coherent(usb_dev, xr->ctrlsize, xr->ctrl_buffer, xr->ctrl_dma);
	xr_usb_serial_read_buffers_free(xr);

	if (!xr->combined_interfaces) {
		usb_driver_release_interface(&xr_usb_serial_driver,
					     intf == xr->control ?
					     xr->data : xr->control);
	}

	tty_port_put(&xr->port);
}

#ifdef CONFIG_PM
static int xr_usb_serial_suspend(struct usb_interface *intf,
				 pm_message_t message)
{
	struct xr_usb_serial *xr = usb_get_intfdata(intf);
	int cnt;

	if (PMSG_IS_AUTO(message)) {
		int b;

		spin_lock_irq(&xr->write_lock);
		b = xr->transmitting;
		spin_unlock_irq(&xr->write_lock);
		if (b)
			return -EBUSY;
	}

	spin_lock_irq(&xr->read_lock);
	spin_lock(&xr->write_lock);
	cnt = xr->susp_count++;
	spin_unlock(&xr->write_lock);
	spin_unlock_irq(&xr->read_lock);

	if (cnt)
		return 0;

	if (test_bit(ASYNCB_INITIALIZED, &xr->port.flags))
		stop_data_traffic(xr);

	return 0;
}

static int xr_usb_serial_resume(struct usb_interface *intf)
{
	struct xr_usb_serial *xr = usb_get_intfdata(intf);
	struct xr_usb_serial_wb *wb;
	int rv = 0;
	int cnt;

	spin_lock_irq(&xr->read_lock);
	xr->susp_count -= 1;
	cnt = xr->susp_count;
	spin_unlock_irq(&xr->read_lock);

	if (cnt)
		return 0;

	if (test_bit(ASYNCB_INITIALIZED, &xr->port.flags)) {
		rv = usb_submit_urb(xr->ctrlurb, GFP_NOIO);

		spin_lock_irq(&xr->write_lock);
		if (xr->delayed_wb) {
			wb = xr->delayed_wb;
			xr->delayed_wb = NULL;
			spin_unlock_irq(&xr->write_lock);
			xr_usb_serial_start_wb(xr, wb);
		} else {
			spin_unlock_irq(&xr->write_lock);
		}

		/*
		 * delayed error checking because we must
		 * do the write path at all cost
		 */
		if (rv < 0)
			goto err_out;

		rv = xr_usb_serial_submit_read_urbs(xr, GFP_NOIO);
	}

err_out:
	return rv;
}

static int xr_usb_serial_reset_resume(struct usb_interface *intf)
{
	struct xr_usb_serial *xr = usb_get_intfdata(intf);

	if (test_bit(ASYNCB_INITIALIZED, &xr->port.flags))
		tty_port_tty_hangup(&xr->port, false);

	return xr_usb_serial_resume(intf);
}

#endif /* CONFIG_PM */

/*
 * USB driver structure.
 */
static const struct usb_device_id xr_usb_serial_ids[] = {
	{ USB_DEVICE(0x04e2, 0x1410) },
	{ USB_DEVICE(0x04e2, 0x1411) },
	{ USB_DEVICE(0x04e2, 0x1412) },
	{ USB_DEVICE(0x04e2, 0x1414) },
	{ USB_DEVICE(0x04e2, 0x1420) },
	{ USB_DEVICE(0x04e2, 0x1421) },
	{ USB_DEVICE(0x04e2, 0x1422) },
	{ USB_DEVICE(0x04e2, 0x1424) },
	{ USB_DEVICE(0x04e2, 0x1400) },
	{ USB_DEVICE(0x04e2, 0x1401) },
	{ USB_DEVICE(0x04e2, 0x1402) },
	{ USB_DEVICE(0x04e2, 0x1403) },
	{ }
};

MODULE_DEVICE_TABLE(usb, xr_usb_serial_ids);

static struct usb_driver xr_usb_serial_driver = {
	.name				= "cdc_xr_usb_serial",
	.probe				= xr_usb_serial_probe,
	.disconnect			= xr_usb_serial_disconnect,
#ifdef CONFIG_PM
	.suspend			= xr_usb_serial_suspend,
	.resume				= xr_usb_serial_resume,
	.reset_resume			= xr_usb_serial_reset_resume,
#endif
	.id_table			= xr_usb_serial_ids,
#ifdef CONFIG_PM
	.supports_autosuspend		= 1,
#endif
	.disable_hub_initiated_lpm	= 1,
};

/*
 * TTY driver structures.
 */

static const struct tty_operations xr_usb_serial_ops = {
	.install		= xr_usb_serial_tty_install,
	.open			= xr_usb_serial_tty_open,
	.close			= xr_usb_serial_tty_close,
	.cleanup		= xr_usb_serial_tty_cleanup,
	.hangup			= xr_usb_serial_tty_hangup,
	.write			= xr_usb_serial_tty_write,
	.write_room		= xr_usb_serial_tty_write_room,
	.ioctl			= xr_usb_serial_tty_ioctl,
	.throttle		= xr_usb_serial_tty_throttle,
	.unthrottle		= xr_usb_serial_tty_unthrottle,
	.chars_in_buffer	= xr_usb_serial_tty_chars_in_buffer,
	.break_ctl		= xr_usb_serial_tty_break_ctl,
	.set_termios		= xr_usb_serial_tty_set_termios,
	.tiocmget		= xr_usb_serial_tty_tiocmget,
	.tiocmset		= xr_usb_serial_tty_tiocmset,
};

/*
 * Init / exit.
 */
static int __init xr_usb_serial_init(void)
{
	int retval;

	xr_usb_serial_tty_driver = alloc_tty_driver(XR_USB_SERIAL_TTY_MINORS);
	if (!xr_usb_serial_tty_driver)
		return -ENOMEM;
	xr_usb_serial_tty_driver->name = "ttyXRUSB",
	xr_usb_serial_tty_driver->driver_name = "exar-usb",
	xr_usb_serial_tty_driver->major = XR_USB_SERIAL_TTY_MAJOR,
	xr_usb_serial_tty_driver->minor_start = 0,
	xr_usb_serial_tty_driver->type = TTY_DRIVER_TYPE_SERIAL,
	xr_usb_serial_tty_driver->subtype = SERIAL_TYPE_NORMAL,
	xr_usb_serial_tty_driver->flags = TTY_DRIVER_REAL_RAW |
					  TTY_DRIVER_DYNAMIC_DEV;
	xr_usb_serial_tty_driver->init_termios = tty_std_termios;
	xr_usb_serial_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD |
							 HUPCL | CLOCAL;
	tty_set_operations(xr_usb_serial_tty_driver, &xr_usb_serial_ops);

	retval = tty_register_driver(xr_usb_serial_tty_driver);
	if (retval) {
		put_tty_driver(xr_usb_serial_tty_driver);
		return retval;
	}

	retval = usb_register(&xr_usb_serial_driver);
	if (retval) {
		tty_unregister_driver(xr_usb_serial_tty_driver);
		put_tty_driver(xr_usb_serial_tty_driver);
		return retval;
	}

	pr_info(KBUILD_MODNAME ": " DRIVER_DESC "\n");

	return 0;
}

static void __exit xr_usb_serial_exit(void)
{
	usb_deregister(&xr_usb_serial_driver);
	tty_unregister_driver(xr_usb_serial_tty_driver);
	put_tty_driver(xr_usb_serial_tty_driver);
}

module_init(xr_usb_serial_init);
module_exit(xr_usb_serial_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_ALIAS_CHARDEV_MAJOR(XR_USB_SERIAL_TTY_MAJOR);

