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
 * The driver has been tested on various kernel versions from 2.6.18 to 3.4.x.
 * There is a different driver available from www.exar.com for kernel versions 3.6.x and newer.
 *
 * ChangeLog:
 *            Version 1A - Initial released version.
 */
 
#define DRIVER_VERSION "1A"
#define DRIVER_AUTHOR "<uarttechsupport@exar.com>"
#define DRIVER_DESC "Exar USB UART (serial port) driver"

#undef XR21B142X_IWA


#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/errno.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <linux/serial.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <asm/uaccess.h>

#include <linux/usb/cdc.h>
#ifndef CDC_DATA_INTERFACE_TYPE
#define CDC_DATA_INTERFACE_TYPE 0x0a
#endif
#ifndef USB_RT_ACM
#define USB_RT_ACM      (USB_TYPE_CLASS | USB_RECIP_INTERFACE)
#define ACM_CTRL_DTR            0x01
#define ACM_CTRL_RTS            0x02
#define ACM_CTRL_DCD            0x01
#define ACM_CTRL_DSR            0x02
#define ACM_CTRL_BRK            0x04
#define ACM_CTRL_RI             0x08
#define ACM_CTRL_FRAMING        0x10
#define ACM_CTRL_PARITY         0x20
#define ACM_CTRL_OVERRUN        0x40
#endif

#include "linux/version.h"

#include "xr_usb_serial_common.h"

#define N_IN_URB    4
#define N_OUT_URB   12
#define IN_BUFLEN   4096

static int debug = 0;


/* -------------------------------------------------------------------------- */

#if defined(RHEL_RELEASE_CODE)
#if RHEL_RELEASE_CODE < RHEL_RELEASE_VERSION(5, 2)
#define true 1

static inline int usb_endpoint_dir_in(const struct usb_endpoint_descriptor *epd)
{
        return ((epd->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN);
}

static inline int usb_endpoint_dir_out(const struct usb_endpoint_descriptor *epd)
{
        return ((epd->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT);
}

static inline int usb_endpoint_xfer_bulk(const struct usb_endpoint_descriptor *epd)
{
        return ((epd->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
                USB_ENDPOINT_XFER_BULK);
}

static inline int usb_endpoint_is_bulk_in(const struct usb_endpoint_descriptor *epd)
{
        return (usb_endpoint_xfer_bulk(epd) && usb_endpoint_dir_in(epd));
}

static inline int usb_endpoint_is_bulk_out(const struct usb_endpoint_descriptor *epd)
{
        return (usb_endpoint_xfer_bulk(epd) && usb_endpoint_dir_out(epd));
}
#endif
#endif

/* -------------------------------------------------------------------------- */

#include "xr_usbserial_ioctl.h"

/* -------------------------------------------------------------------------- */
static struct usb_device_id id_table [] = {
        { USB_DEVICE(0x04e2, 0x1410)},
        { USB_DEVICE(0x04e2, 0x1411)},
	    { USB_DEVICE(0x04e2, 0x1412)},
	    { USB_DEVICE(0x04e2, 0x1414)},
        { USB_DEVICE(0x04e2, 0x1420)},
        { USB_DEVICE(0x04e2, 0x1422)},
        { USB_DEVICE(0x04e2, 0x1424)},
        { USB_DEVICE(0x04e2, 0x1400)},
        { USB_DEVICE(0x04e2, 0x1401)},
        { USB_DEVICE(0x04e2, 0x1402)},
        { USB_DEVICE(0x04e2, 0x1403)},
        
        { }
};
MODULE_DEVICE_TABLE(usb, id_table);


static void xr_usb_serial_disconnect(struct usb_interface *interface);

static struct usb_driver xr_usb_serial_driver = {
        .name          = "xr_usb_serial",
        .probe         = usb_serial_probe,
        .disconnect    = xr_usb_serial_disconnect,
        .id_table      = id_table,
        .no_dynamic_id = 1,
};


/* -------------------------------------------------------------------------- */

struct xr_usb_serial_serial_private
{
        struct usb_interface *data_interface;
		unsigned short DeviceVendor;
        unsigned short DeviceProduct;
};


struct xr_usb_serial_port_private {
        spinlock_t    lock;     /* lock the structure */
        int           outstanding_urbs; /* number of out urbs in flight */

        struct urb   *in_urbs[N_IN_URB];
        char         *in_buffer[N_IN_URB];

        int           ctrlin;
        int           ctrlout;
        int           clocal;

        int           block;
		struct reg_addr_map reg_map;
		struct usb_cdc_line_coding  line_coding;
		
        int           preciseflags; /* USB: wide mode, TTY: flags per character */
        int           trans9;   /* USB: wide mode, serial 9N1 */
        unsigned int  baud_base; /* setserial: used to hack in non-standard baud rates */
        int           have_extra_byte;
        int           extra_byte;

	    int           bcd_device;
        unsigned int  throttled:1;	   /* actually throttled */
	    unsigned int  throttle_req:1;   /* throttle requested */
#ifdef XR21B142X_IWA
        int           iwa;
#endif
        unsigned short DeviceVendor;
        unsigned short DeviceProduct;
};


/* -------------------------------------------------------------------------- */

static int xr_usb_serial_rev_a(struct usb_serial_port *port)
{
        struct xr_usb_serial_port_private *portdata = usb_get_serial_port_data(port);
	return portdata->bcd_device == 0;
}
static int xr_usb_serial_ctrl_msg(struct usb_serial_port *port,
                        int request, int value, void *buf, int len)
{
        struct usb_serial *serial = port->serial;
        int retval = usb_control_msg(serial->dev,
                                     usb_sndctrlpipe(serial->dev, 0),
                                     request,
                                     USB_RT_ACM,
                                     value,
                                     serial->interface->cur_altsetting->desc.bInterfaceNumber,
                                     buf,
                                     len,
                                     5000);
		if (debug)
          dev_dbg(&port->serial->dev->dev, "xr_usb_serial_control_msg: rq: 0x%02x val: %#x len: %#x result: %d\n", request, value, len, retval);
        return retval < 0 ? retval : 0;
}

#include "xr_usb_serial_hal.c"

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
static void xr_usb_serial_set_termios(struct usb_serial_port *port,
                                struct termios *old_termios)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
static void xr_usb_serial_set_termios(struct usb_serial_port *port,
                                struct ktermios *old_termios)
#else
static void xr_usb_serial_set_termios(struct tty_struct *tty_param,
                                struct usb_serial_port *port,
                                struct ktermios *old_termios)
#endif
{
        struct xr_usb_serial_port_private *portdata = usb_get_serial_port_data(port);

        unsigned int             cflag, block;
			
        unsigned int             data_bits, parity_type, char_format;
		int result;
		
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
        struct tty_struct       *tty = port->tty;
#else
        struct tty_struct       *tty = port->port.tty;
#endif

        if (debug) dev_dbg(&port->serial->dev->dev, "%s\n", __func__);
      
/*  mutex_lock(&config_mutex); */

        cflag = tty->termios->c_cflag;
        portdata->clocal = ((cflag & CLOCAL) != 0);
        block = portdata->block;
        xr_usb_serial_disable(port);

		if ((cflag & CSIZE) == CS7) {
					   data_bits = 7;
			   } else if ((cflag & CSIZE) == CS5) {
					   /* Enabling 5-bit mode is really 9-bit mode! */
					   data_bits = 9;
			   } else {
					   data_bits = 8;
			   }
			   portdata->trans9 = (data_bits == 9);

			   if (cflag & PARENB) {
					   if (cflag & PARODD) {
							   if (cflag & CMSPAR) {
									   parity_type = USB_CDC_MARK_PARITY;
							   } else {
									   parity_type = USB_CDC_ODD_PARITY;
							   }
					   } else {
							   if (cflag & CMSPAR) {
									   parity_type = USB_CDC_SPACE_PARITY;
							   } else {
									   parity_type = USB_CDC_EVEN_PARITY;
							   }
					   }
			   } else {
					   parity_type = USB_CDC_NO_PARITY;
			   }

			   if (cflag & CSTOPB) {
					   char_format = USB_CDC_2_STOP_BITS;
			   } else {
					   char_format = USB_CDC_1_STOP_BITS;
			   }
			   
#ifdef XR21B142X_IWA
        if (data_bits == UART_FORMAT_SIZE_8)
		{
            portdata->iwa = parity_type;
            if (portdata->iwa != USB_CDC_NO_PARITY)
			{
                    data_bits = 9;
                    parity_type = USB_CDC_NO_PARITY;
            }
        } 
		else 
		{
            portdata->iwa = USB_CDC_NO_PARITY;
        }
#endif	   
               
        xr_usb_serial_set_flow_mode(port,tty,cflag);   
		        
        if (portdata->trans9) 
		{
                /* Turn on wide mode if we're 9-bit transparent. */
             	xr_usb_serial_set_wide_mode(port,1);
#ifdef XR21B142X_IWA
        } 
		else if (portdata->iwa != USB_CDC_NO_PARITY)
		{
				xr_usb_serial_set_wide_mode(port,1);
#endif
        } 
		else if (!portdata->preciseflags) 
		{
              	xr_usb_serial_set_wide_mode(port,0);
        }
				
        portdata->line_coding.dwDTERate   = cpu_to_le32(tty_get_baud_rate(tty));
        portdata->line_coding.bCharFormat = char_format;
        portdata->line_coding.bParityType = parity_type;
        portdata->line_coding.bDataBits   = data_bits;
		
        if (debug) 
        dev_dbg(&port->serial->dev->dev, "%s: line coding: rate=%d format=%d parity=%d bits=%d\n", __func__,
                           cpu_to_le32(tty_get_baud_rate(tty)),
                           char_format,
                           parity_type,
                           data_bits);
		
		
        result = xr_usb_serial_set_line(port, &portdata->line_coding);
        if (result < 0) {
              dev_dbg(&port->serial->dev->dev, "%s: cannot set line coding: rate=%d format=%d parity=%d bits=%d\n", __func__,
                        tty_get_baud_rate(tty),
                        portdata->line_coding.bCharFormat,
                        portdata->line_coding.bParityType,
                        portdata->line_coding.bDataBits);
        }
		
        xr_usb_serial_enable(port);

/*  mutex_unlock(&config_mutex); */
}

#define UART_TX_BREAK	0x0A
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
static void xr_usb_serial_break_ctl(struct usb_serial_port *port, int break_state)
#else
static void xr_usb_serial_break_ctl(struct tty_struct *tty, int break_state)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
#else
        struct usb_serial_port *port = tty->driver_data;
#endif

        if (debug) dev_dbg(&port->serial->dev->dev, "BREAK %d\n", break_state);
		
        xr_usb_serial_send_break(port,break_state);
		
}



#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
static int xr_usb_serial_tiocmget(struct usb_serial_port *port, struct file *file)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 39)
static int xr_usb_serial_tiocmget(struct tty_struct *tty, struct file *file)
#else
static int xr_usb_serial_tiocmget(struct tty_struct *tty)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
#else
        struct usb_serial_port *port = tty->driver_data;
#endif
		short data;
		int result;
        result = xr_usb_serial_get_gpio_status(port,&data);
		if (result)
			return ((data & 0x8) ? 0: TIOCM_DTR) | ((data & 0x20) ? 0:TIOCM_RTS ) | ((data & 0x4) ? 0:TIOCM_DSR) | ((data & 0x1) ? 0 : TIOCM_RI) | ((data & 0x2) ? 0:TIOCM_CD) | ((data & 0x10) ? 0 : TIOCM_CTS); 
		else
			return -EFAULT;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
static int xr_usb_serial_tiocmset(struct usb_serial_port *port, struct file *file,
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 39)
static int xr_usb_serial_tiocmset(struct tty_struct *tty, struct file *file,
#else
static int xr_usb_serial_tiocmset(struct tty_struct *tty,
#endif
                            unsigned int set, unsigned int clear)

{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
#else
        struct usb_serial_port *port = tty->driver_data;
#endif
        struct xr_usb_serial_port_private *portdata = usb_get_serial_port_data(port);
        unsigned int newctrl;

        newctrl = portdata->ctrlout;
        set = (set & TIOCM_DTR ? ACM_CTRL_DTR : 0) | (set & TIOCM_RTS ? ACM_CTRL_RTS : 0);
        clear = (clear & TIOCM_DTR ? ACM_CTRL_DTR : 0) | (clear & TIOCM_RTS ? ACM_CTRL_RTS : 0);

        newctrl = (newctrl & ~clear) | set;

        if (portdata->ctrlout == newctrl)
                return 0;

		portdata->ctrlout = newctrl;
		if (newctrl & ACM_CTRL_DTR) 
		    xr_usb_serial_ctrl_dtr(port,0);
		else
			xr_usb_serial_ctrl_dtr(port,1);

		if (newctrl & ACM_CTRL_RTS) 
			xr_usb_serial_ctrl_rts(port,0);
		else
            xr_usb_serial_ctrl_rts(port,1);
        return 0;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
static int xr_usb_serial_ioctl(struct usb_serial_port *port, struct file *file, unsigned int cmd, unsigned long arg)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 39)
static int xr_usb_serial_ioctl(struct tty_struct *tty, struct file *file, unsigned int cmd, unsigned long arg)
#else
static int xr_usb_serial_ioctl(struct tty_struct *tty, unsigned int cmd, unsigned long arg)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
#else
        struct usb_serial_port *port = tty->driver_data;
#endif
        struct xr_usb_serial_port_private *portdata = usb_get_serial_port_data(port);

        unsigned int             block, reg, val, match, preciseflags, unicast, broadcast, flow, selector;
        short                    *data;
        int                      result;
        struct serial_struct     ss;

        if (debug) dev_dbg(&port->serial->dev->dev, "%s %08x\n", __func__, cmd);

        switch (cmd) {
        case TIOCGSERIAL:
                if (!arg)
                        return -EFAULT;
                memset(&ss, 0, sizeof(ss));
                ss.baud_base = portdata->baud_base;
                if (copy_to_user((void __user *)arg, &ss, sizeof(ss)))
                        return -EFAULT;
                break;

        case TIOCSSERIAL:
                if (!arg)
                        return -EFAULT;
                if (copy_from_user(&ss, (void __user *)arg, sizeof(ss)))
                        return -EFAULT;
                portdata->baud_base = ss.baud_base;
				portdata->line_coding.dwDTERate = cpu_to_le32(ss.baud_base);
                if (debug) dev_dbg(&port->serial->dev->dev, "baud_base=%d\n", portdata->baud_base);

/*      mutex_lock(&config_mutex); */
                xr_usb_serial_disable(port);
                if (debug)
		        dev_dbg(&port->serial->dev->dev, "%s: line coding: rate=%d format=%d parity=%d bits=%d\n", __func__,
                                   ss.baud_base,
                                   portdata->line_coding.bCharFormat,
                                   portdata->line_coding.bParityType,
                                   portdata->line_coding.bDataBits);
                result = xr_usb_serial_set_line(port, &portdata->line_coding);
                if (result < 0) {
                        //dev_dbg(&port->dev, "%d != %ld\n", result, sizeof(portdata->line_coding));
                        if (debug)
                        dev_dbg(&port->serial->dev->dev, "%s: cannot set line coding: rate=%d format=%d parity=%d bits=%d\n", __func__,
                                ss.baud_base,
                                portdata->line_coding.bCharFormat,
                                portdata->line_coding.bParityType,
                                portdata->line_coding.bDataBits);
                        return -EINVAL;
                }
                xr_usb_serial_enable(port);
/*      mutex_unlock(&config_mutex); */
                break;
        		
        case XRIOC_GET_REG:
                if (get_user(block, (int __user *)arg))
                        return -EFAULT;
                if (get_user(reg, (int __user *)(arg + sizeof(int))))
                        return -EFAULT;

                data = kmalloc(2, GFP_KERNEL);
                if (data == NULL) {
                        dev_err(&port->serial->dev->dev, "%s - Cannot allocate USB buffer.\n", __func__);
                        return -ENOMEM;
				}

				if (block == -1)
				{
                    result = xr_usb_serial_get_reg(port,reg, data);
				}
				else
				{
				    result = xr_usb_serial_get_reg_ext(port,block,reg,data);
				}
				
                if (result < 0 ) {
                        dev_err(&port->serial->dev->dev, "Cannot get register (%d) block=%d \n", result,block);
                        kfree(data);
                        return -EFAULT;
                }
				if (debug)
                   dev_dbg(&port->serial->dev->dev,"xr_usb_serial_get_reg<%d> value =%x\n",reg,*data);
				
                if (put_user(data[0], (int __user *)(arg + 2 * sizeof(int)))) {
                        dev_err(&port->serial->dev->dev, "Cannot put user result\n");
                        kfree(data);
                        return -EFAULT;
                }

                kfree(data);
                break;

        case XRIOC_SET_REG:
                if (get_user(block, (int __user *)arg))
                        return -EFAULT;
                if (get_user(reg, (int __user *)(arg + sizeof(int))))
                        return -EFAULT;
                if (get_user(val, (int __user *)(arg + 2 * sizeof(int))))
                        return -EFAULT;

				if (block == -1)
				{
                    result = xr_usb_serial_set_reg(port,reg, val);
				}
				else
				{
				   result = xr_usb_serial_set_reg_ext(port,block,reg, val);
				}
                if (result < 0)
                        return -EFAULT;
                break;
        case XRIOC_SET_ADDRESS_MATCH:
                match = arg;

                if (debug) dev_dbg(&port->serial->dev->dev, "%s VIOC_SET_ADDRESS_MATCH %d\n", __func__, match);

                xr_usb_serial_disable(port);

                if (match & XR_ADDRESS_MATCH_DISABLE) {
                        flow      = UART_FLOW_MODE_NONE;
                } else {
                        flow      = UART_FLOW_MODE_ADDR_MATCH_TX;
                        unicast   = (match >> XR_ADDRESS_UNICAST_S) & 0xff;
                        broadcast = (match >> XR_ADDRESS_BROADCAST_S) & 0xff;
                }

                if (debug) dev_dbg(&port->dev, "address match: flow=%d ucast=%d bcast=%u\n",
                                   flow, unicast, broadcast);
				            
                xr_usb_serial_enable(port);
                break;
        case XRIOC_SET_PRECISE_FLAGS:
                preciseflags = arg;

                if (debug) dev_dbg(&port->serial->dev->dev, "%s VIOC_SET_PRECISE_FLAGS %d\n", __func__, preciseflags);

                xr_usb_serial_disable(port);

                if (preciseflags) {
                        portdata->preciseflags = 1;
                } else {
                        portdata->preciseflags = 0;
                }
				xr_usb_serial_set_wide_mode(port,portdata->preciseflags);
              					
                xr_usb_serial_enable(port);
                break;

       

	case XRIOC_LOOPBACK:
		selector = arg;
		dev_dbg(&port->serial->dev->dev, "VIOC_LOOPBACK 0x%02x\n", selector);
		xr_usb_serial_set_loopback(port);
		break;

        default:
                return -ENOIOCTLCMD;
        }

        return 0;
}


/* -------------------------------------------------------------------------- */


#ifdef XR21B142X_IWA
static const int xr_usb_serial_parity[] =
{
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0
};
#endif



#if LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 18)
static void xr_usb_serial_out_callback(struct urb *urb, struct pt_regs *regs)
#else
static void xr_usb_serial_out_callback(struct urb *urb)
#endif
{
        struct usb_serial_port          *port     = urb->context;
        struct xr_usb_serial_port_private     *portdata = usb_get_serial_port_data(port);
        int                              status   = urb->status;
        unsigned long                    flags;

        if (debug) dev_dbg(&port->serial->dev->dev, "%s - port %d\n", __func__, port->number);

        /* free up the transfer buffer, as usb_free_urb() does not do this */
        kfree(urb->transfer_buffer);

        if (status)
                if (debug) dev_dbg(&port->serial->dev->dev, "%s - nonzero write bulk status received: %d\n",
                                   __func__, status);

        spin_lock_irqsave(&portdata->lock, flags);
        --portdata->outstanding_urbs;
        spin_unlock_irqrestore(&portdata->lock, flags);

        usb_serial_port_softint(port);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
static int xr_usb_serial_write_room(struct usb_serial_port *port)
#else
static int xr_usb_serial_write_room(struct tty_struct *tty)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
#else
        struct usb_serial_port *port = tty->driver_data;
#endif
		struct xr_usb_serial_port_private     *portdata = usb_get_serial_port_data(port);
        unsigned long                    flags;

        if (debug) 
			dev_dbg(&port->dev, "%s - port %d\n", __func__, port->number);

        /* try to give a good number back based on if we have any free urbs at
         * this point in time */
        spin_lock_irqsave(&portdata->lock, flags);
        if (portdata->outstanding_urbs > N_OUT_URB * 2 / 3) 
		{
                spin_unlock_irqrestore(&portdata->lock, flags);
                if (debug)
					dev_dbg(&port->dev, "%s - write limit hit\n", __func__);
                return 0;
        }
        spin_unlock_irqrestore(&portdata->lock, flags);

        return 2048;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
static int xr_usb_serial_write(struct usb_serial_port *port,
#else
static int xr_usb_serial_write(struct tty_struct *tty, struct usb_serial_port *port,
#endif
                         const unsigned char *buf, int count)
{
        struct xr_usb_serial_port_private     *portdata = usb_get_serial_port_data(port);
        struct usb_serial               *serial   = port->serial;
        int                              bufsize  = count;
        unsigned long                    flags;
        unsigned char                   *buffer;
        struct urb                      *urb;
        int                              status;

        portdata = usb_get_serial_port_data(port);

        if (debug) dev_dbg(&port->dev, "%s: write (%d chars)\n", __func__, count);

        spin_lock_irqsave(&portdata->lock, flags);
        if (portdata->outstanding_urbs > N_OUT_URB) {
                spin_unlock_irqrestore(&portdata->lock, flags);
                if (debug) dev_dbg(&port->dev, "%s - write limit hit\n", __func__);
                return 0;
        }
        portdata->outstanding_urbs++;
        spin_unlock_irqrestore(&portdata->lock, flags);

#ifdef XR21B142X_IWA
        if (portdata->iwa != UART_FORMAT_PARITY_NONE)
                bufsize = count * 2;
#endif
        buffer = kmalloc(bufsize, GFP_ATOMIC);

        if (!buffer) {
                dev_err(&port->dev, "out of memory\n");
                count = -ENOMEM;
                goto error_no_buffer;
        }

        urb = usb_alloc_urb(0, GFP_ATOMIC);
        if (!urb) {
                dev_err(&port->dev, "no more free urbs\n");
                count = -ENOMEM;
                goto error_no_urb;
        }

#ifdef XR21B142X_IWA
        if (portdata->iwa != UART_FORMAT_PARITY_NONE) {
                int i;
                char *b = buffer;
                for (i = 0; i < count; ++i) {
                        int c, p = 0;
                        c = buf[i];
                        switch (portdata->iwa) {
                        case UART_FORMAT_PARITY_ODD:    p = !xr_usb_serial_parity[c]; break;
                        case UART_FORMAT_PARITY_EVEN:   p = xr_usb_serial_parity[c];  break;
                        case UART_FORMAT_PARITY_1:  p = 1;          break;
                        case UART_FORMAT_PARITY_0:  p = 0;          break;
                        }
                        *b++ = c;
                        *b++ = p;
                }
        } else
#endif
                memcpy(buffer, buf, count);

/*         usb_serial_debug_data(debug, &port->dev, __func__, bufsize, buffer); */

        usb_fill_bulk_urb(urb, serial->dev,
                          usb_sndbulkpipe(serial->dev,
                                          port->bulk_out_endpointAddress),
                          buffer, bufsize, xr_usb_serial_out_callback, port);

        /* send it down the pipe */
        status = usb_submit_urb(urb, GFP_ATOMIC);
        if (status) {
                dev_err(&port->dev, "%s - usb_submit_urb(write bulk) failed "
                        "with status = %d\n", __func__, status);
                count = status;
                goto error;
        }

        /* we are done with this urb, so let the host driver
         * really free it when it is finished with it */
        usb_free_urb(urb);

        return count;
error:
        usb_free_urb(urb);
error_no_urb:
        kfree(buffer);
error_no_buffer:
        spin_lock_irqsave(&portdata->lock, flags);
        --portdata->outstanding_urbs;
        spin_unlock_irqrestore(&portdata->lock, flags);
        return count;
}



/* -------------------------------------------------------------------------- */

#if LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 18)
static void xr_usb_serial_in_callback(struct urb *urb, struct pt_regs *regs)
#else
static void xr_usb_serial_in_callback(struct urb *urb)
#endif
{
        int                              endpoint        = usb_pipeendpoint(urb->pipe);
        struct usb_serial_port          *port            = urb->context;
        struct xr_usb_serial_port_private     *portdata        = usb_get_serial_port_data(port);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
        struct tty_struct               *tty             = port->tty;
#else
        struct tty_struct               *tty             = port->port.tty;
#endif
		int                              preciseflags    = portdata->preciseflags;
        char                            *transfer_buffer = urb->transfer_buffer;
        int                              length, room, have_extra_byte;
        int                              err;
        unsigned long                    flags;
        if (debug) dev_dbg(&port->dev, "%s: %p\n", __func__, urb);

        if (urb->status) {
                if (debug) dev_dbg(&port->dev, "%s: nonzero status: %d on endpoint %02x.\n",
                                   __func__, urb->status, endpoint);
                return;
        }

#ifdef XR21B142X_IWA
        if (portdata->iwa != UART_FORMAT_PARITY_NONE) {
                preciseflags = true;
        }
#endif

        length = urb->actual_length;
        if (length == 0) {
                if (debug) dev_dbg(&port->dev, "%s: empty read urb received\n", __func__);
                err = usb_submit_urb(urb, GFP_ATOMIC);
                if (err)
                        dev_err(&port->dev, "resubmit read urb failed. (%d)\n", err);
                return;
        }

        length      = length + (portdata->have_extra_byte ? 1 : 0);
        have_extra_byte = (preciseflags && (length & 1));
        length      = (preciseflags) ? (length / 2) : length;

        room = tty_buffer_request_room(tty, length);
        if (room != length) {
                if (debug) dev_dbg(&port->dev, "Not enough room in TTY buf, dropped %d chars.\n", length - room);
        }

        if (room) {
                if (preciseflags) {
                        char *dp = transfer_buffer;
                        int i, ch, ch_flags;

                        for (i = 0; i < room; ++i) {
                                char tty_flag;

                                if (i == 0) {
                                        if (portdata->have_extra_byte) {
                                                ch = portdata->extra_byte;
                                        } else {
                                                ch = *dp++;
                                        }
                                } else {
                                        ch = *dp++;
                                }
                                ch_flags = *dp++;

#ifdef XR21B142X_IWA
                                {
                                        int p;
                                        switch (portdata->iwa) {
                                        case UART_FORMAT_PARITY_ODD:    p = !xr_usb_serial_parity[ch]; break;
                                        case UART_FORMAT_PARITY_EVEN:   p = xr_usb_serial_parity[ch];  break;
                                        case UART_FORMAT_PARITY_1:  p = 1;           break;
                                        case UART_FORMAT_PARITY_0:  p = 0;           break;
                                        default:                        p = 0;           break;
                                        }
                                        ch_flags ^= p;
                                }
#endif
                              
                                if (ch_flags & RAMCTL_BUFFER_PARITY)
                                        tty_flag = TTY_PARITY;
                                else if (ch_flags & RAMCTL_BUFFER_BREAK)
                                        tty_flag = TTY_BREAK;
                                else if (ch_flags & RAMCTL_BUFFER_FRAME)
                                        tty_flag = TTY_FRAME;
                                else if (ch_flags & RAMCTL_BUFFER_OVERRUN)
                                        tty_flag = TTY_OVERRUN;
                                else
                                        tty_flag = TTY_NORMAL;
							
                                tty_insert_flip_char(tty, ch, tty_flag);
                        }
                } else {
                        tty_insert_flip_string(tty, transfer_buffer, room);
                }

                tty_flip_buffer_push(tty);
        }

        portdata->have_extra_byte = have_extra_byte;
        if (have_extra_byte) {
                portdata->extra_byte = transfer_buffer[urb->actual_length - 1];
        }
		spin_lock_irqsave(&portdata->lock, flags);
		portdata->throttled = portdata->throttle_req;
		spin_unlock_irqrestore(&portdata->lock, flags);
        if(!(portdata->throttled))
        {
           err = usb_submit_urb(urb, GFP_ATOMIC);
           if (err)
                  dev_err(&port->dev, "resubmit read urb failed. (%d)\n", err);
        }
}


/* -------------------------------------------------------------------------- */
#if LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 18)
static void xr_usb_serial_int_callback(struct urb *urb, struct pt_regs *regs)
#else
static void xr_usb_serial_int_callback(struct urb *urb)
#endif
{
        struct usb_serial_port          *port     = urb->context;
        struct xr_usb_serial_port_private     *portdata = usb_get_serial_port_data(port);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
        struct tty_struct               *tty      = port->tty;
#else
        struct tty_struct               *tty      = port->port.tty;
#endif
        struct usb_cdc_notification     *dr       = urb->transfer_buffer;
        unsigned char                   *data;
        int                              newctrl;
        int                              status;

        switch (urb->status) {
        case 0:
                /* success */
                break;
        case -ECONNRESET:
        case -ENOENT:
        case -ESHUTDOWN:
                /* this urb is terminated, clean up */
                if (debug) dev_dbg(&port->dev, "urb shutting down with status: %d\n", urb->status);
                return;
        default:
                if (debug) dev_dbg(&port->dev, "nonzero urb status received: %d\n", urb->status);
                goto exit;
        }


        data = (unsigned char *)(dr + 1);
        switch (dr->bNotificationType) {

        case USB_CDC_NOTIFY_NETWORK_CONNECTION:
                if (debug) dev_dbg(&port->dev, "%s network\n", dr->wValue ? "connected to" : "disconnected from");
                break;

        case USB_CDC_NOTIFY_SERIAL_STATE:
                newctrl = le16_to_cpu(get_unaligned((__le16 *)data));

                if (!portdata->clocal && (portdata->ctrlin & ~newctrl & ACM_CTRL_DCD)) {
                        if (debug) dev_dbg(&port->dev, "calling hangup\n");
                        tty_hangup(tty);
                }

                portdata->ctrlin = newctrl;

                if (debug) dev_dbg(&port->dev, "input control lines: dcd%c dsr%c break%c ring%c framing%c parity%c overrun%c\n",
                                   portdata->ctrlin & ACM_CTRL_DCD ? '+' : '-',
                                   portdata->ctrlin & ACM_CTRL_DSR ? '+' : '-',
                                   portdata->ctrlin & ACM_CTRL_BRK ? '+' : '-',
                                   portdata->ctrlin & ACM_CTRL_RI  ? '+' : '-',
                                   portdata->ctrlin & ACM_CTRL_FRAMING ? '+' : '-',
                                   portdata->ctrlin & ACM_CTRL_PARITY ? '+' : '-',
                                   portdata->ctrlin & ACM_CTRL_OVERRUN ? '+' : '-');
                break;

        default:
                if (debug) dev_dbg(&port->dev, "unknown notification %d received: index %d len %d data0 %d data1 %d\n",
                                   dr->bNotificationType, dr->wIndex,
                                   dr->wLength, data[0], data[1]);
                break;
        }
exit:
        if (debug) dev_dbg(&port->dev, "Resubmitting interrupt IN urb %p\n", urb);
        status = usb_submit_urb(urb, GFP_ATOMIC);
        if (status)
                dev_err(&port->dev, "usb_submit_urb failed with result %d", status);
}


/* -------------------------------------------------------------------------- */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
static int xr_usb_serial_open(struct usb_serial_port *port, struct file *filp)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
static int xr_usb_serial_open(struct tty_struct *tty_param,
                        struct usb_serial_port *port, struct file *filp)
#else
static int xr_usb_serial_open(struct tty_struct *tty_param, struct usb_serial_port *port)
#endif
{
        struct xr_usb_serial_port_private     *portdata;
        struct usb_serial               *serial = port->serial;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
        struct tty_struct               *tty    = port->tty;
#else
        struct tty_struct               *tty    = port->port.tty;
#endif
        int                              i;
        struct urb                      *urb;
        int                              result;

        portdata = usb_get_serial_port_data(port);

        if (debug) dev_dbg(&port->dev, "%s\n", __func__);

		portdata->ctrlout = ACM_CTRL_DTR | ACM_CTRL_RTS;
		if (portdata->ctrlout & ACM_CTRL_DTR) 
		    xr_usb_serial_ctrl_dtr(port,0); 
		else
			xr_usb_serial_ctrl_dtr(port,1);

		if (portdata->ctrlout & ACM_CTRL_RTS) 
			 xr_usb_serial_ctrl_rts(port,0); 
		else
		    xr_usb_serial_ctrl_rts(port,1); 

        /* Reset low level data toggle and start reading from endpoints */
        for (i = 0; i < N_IN_URB; i++) {
                if (debug) dev_dbg(&port->serial->dev->dev, "%s urb %d\n", __func__, i);

                urb = portdata->in_urbs[i];
                if (!urb)
                        continue;
                if (urb->dev != serial->dev) {
                        if (debug) dev_dbg(&port->serial->dev->dev, "%s: dev %p != %p\n", __func__,
                                           urb->dev, serial->dev);
                        continue;
                }

                /*
                 * make sure endpoint data toggle is synchronized with the
                 * device
                 */
/*      if (debug) dev_dbg(&port->dev, "%s clearing halt on %x\n", __func__, urb->pipe); */
/*      usb_clear_halt(urb->dev, urb->pipe); */

                if (debug) dev_dbg(&port->serial->dev->dev, "%s submitting urb %p\n", __func__, urb);
                result = usb_submit_urb(urb, GFP_KERNEL);
                if (result) {
                        dev_err(&port->serial->dev->dev, "submit urb %d failed (%d) %d\n",
                                i, result, urb->transfer_buffer_length);
                }
        }

        tty->low_latency = 0;
        
        /* start up the interrupt endpoint if we have one */
        if (port->interrupt_in_urb) {
                result = usb_submit_urb(port->interrupt_in_urb, GFP_KERNEL);
                if (result)
                        dev_err(&port->serial->dev->dev, "submit irq_in urb failed %d\n",
                                result);
        }
        return 0;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
static void xr_usb_serial_close(struct usb_serial_port *port, struct file *filp)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
static void xr_usb_serial_close(struct tty_struct *tty_param,
                          struct usb_serial_port *port, struct file *filp)
#else
static void xr_usb_serial_close(struct usb_serial_port *port)
#endif
{
        int                              i;
        struct usb_serial               *serial = port->serial;
        struct xr_usb_serial_port_private     *portdata;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
        struct tty_struct               *tty    = port->tty;
#else
        struct tty_struct               *tty    = port->port.tty;
#endif
        if (debug) dev_dbg(&port->serial->dev->dev, "%s\n", __func__);
        portdata = usb_get_serial_port_data(port);

		portdata->ctrlout = 0;
		if (portdata->ctrlout & ACM_CTRL_DTR) 
			xr_usb_serial_ctrl_dtr(port,0); 
		else
			xr_usb_serial_ctrl_dtr(port,1); 

		if (portdata->ctrlout & ACM_CTRL_RTS) 
			xr_usb_serial_ctrl_rts(port,0);
		else
			xr_usb_serial_ctrl_rts(port,1);

        if (serial->dev) {
                /* Stop reading/writing urbs */
                for (i = 0; i < N_IN_URB; i++)
                        usb_kill_urb(portdata->in_urbs[i]);
        }

        usb_kill_urb(port->interrupt_in_urb);

        tty = NULL; /* FIXME */
}


static int xr_usb_serial_attach(struct usb_serial *serial)
{
        struct xr_usb_serial_serial_private   *serial_priv       = usb_get_serial_data(serial);
        struct usb_interface            *interface         = serial_priv->data_interface;
        struct usb_host_interface       *iface_desc;
        struct usb_endpoint_descriptor  *endpoint;
        struct usb_endpoint_descriptor  *bulk_in_endpoint  = NULL;
        struct usb_endpoint_descriptor  *bulk_out_endpoint = NULL;

        struct usb_serial_port          *port;
        struct xr_usb_serial_port_private     *portdata;
        struct urb                      *urb;
        int                              i, j;

        /* Assume that there's exactly one serial port. */
        port = serial->port[0];

        if (debug) dev_dbg(&port->dev, "%s\n", __func__);

        /* The usb_serial is now fully set up, but we want to make a
         * couple of modifications.  Namely, it was configured based
         * upon the control interface and not the data interface, so
         * it has no notion of the bulk in and out endpoints.  So we
         * essentially do some of the same allocations and
         * configurations that the usb-serial core would have done if
         * it had not made any faulty assumptions about the
         * endpoints. */

        iface_desc = interface->cur_altsetting;
        for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
                endpoint = &iface_desc->endpoint[i].desc;

                if (usb_endpoint_is_bulk_in(endpoint)) {
                        bulk_in_endpoint = endpoint;
                }

                if (usb_endpoint_is_bulk_out(endpoint)) {
                        bulk_out_endpoint = endpoint;
                }
        }

        if (!bulk_out_endpoint || !bulk_in_endpoint) {
                if (debug) dev_dbg(&port->dev, "Missing endpoint!\n");
                return -EINVAL;
        }

        port->bulk_out_endpointAddress = bulk_out_endpoint->bEndpointAddress;
        port->bulk_in_endpointAddress = bulk_in_endpoint->bEndpointAddress;

        portdata = kzalloc(sizeof(*portdata), GFP_KERNEL);
        if (!portdata) {
                if (debug) dev_dbg(&port->dev, "%s: kmalloc for xr_usb_serial_port_private (%d) failed!.\n",
                                   __func__, i);
                return -ENOMEM;
        }
        spin_lock_init(&portdata->lock);
        for (j = 0; j < N_IN_URB; j++) {
                portdata->in_buffer[j] = kmalloc(IN_BUFLEN, GFP_KERNEL);
                if (!portdata->in_buffer[j]) {
                        for (--j; j >= 0; j--)
                                kfree(portdata->in_buffer[j]);
                        kfree(portdata);
                        return -ENOMEM;
                }
        }

        
        portdata->block = port->bulk_out_endpointAddress;
		
        dev_info(&port->serial->dev->dev, "xr_usb_serial_attach  portdata->block=%d\n", portdata->block); 
		
        usb_set_serial_port_data(port, portdata);

	    portdata->bcd_device = le16_to_cpu(serial->dev->descriptor.bcdDevice);
		
	    dev_info(&port->serial->dev->dev, "xr_usb_serial_attach  portdata->bcd_device=%d\n", portdata->bcd_device); 
		
		
	if (xr_usb_serial_rev_a(port))
		dev_info(&port->dev, "Adapting to revA silicon\n");

        /* initialize the in urbs */
        for (j = 0; j < N_IN_URB; ++j) 
		{
                urb = usb_alloc_urb(0, GFP_KERNEL);
                if (urb == NULL) {
                        if (debug) dev_dbg(&port->dev, "%s: alloc for in port failed.\n",
                                           __func__);
                        continue;
                }
                /* Fill URB using supplied data. */
                if (debug) dev_dbg(&port->dev, "Filling URB %p, EP=%d buf=%p len=%d\n", urb, port->bulk_in_endpointAddress, portdata->in_buffer[j], IN_BUFLEN);
                usb_fill_bulk_urb(urb, serial->dev,
                                  usb_rcvbulkpipe(serial->dev,
                                                  port->bulk_in_endpointAddress),
                                  portdata->in_buffer[j], IN_BUFLEN,
                                  xr_usb_serial_in_callback, port);
                portdata->in_urbs[j] = urb;
        }
		xr_usb_serial_pre_setup(port,serial_priv->DeviceVendor,serial_priv->DeviceProduct);
			
        return 0;
}


static void xr_usb_serial_serial_disconnect(struct usb_serial *serial)
{
        struct usb_serial_port          *port;
        struct xr_usb_serial_port_private     *portdata;
        int                              i, j;

        if (debug) dev_dbg(&serial->dev->dev, "%s %p\n", __func__, serial);

        for (i = 0; i < serial->num_ports; ++i) {
                port = serial->port[i];
                if (!port)
                        continue;
                portdata = usb_get_serial_port_data(port);
                if (!portdata)
                        continue;
                xr_usb_serial_set_custom_mode(port,0);//Disable the custom driver mode
                for (j = 0; j < N_IN_URB; j++) {
                        usb_kill_urb(portdata->in_urbs[j]);
                        usb_free_urb(portdata->in_urbs[j]);
                }
        }
}


static void xr_usb_serial_serial_release(struct usb_serial *serial)
{
        struct usb_serial_port          *port;
        struct xr_usb_serial_port_private     *portdata;
        int                              i, j;

        if (debug) dev_dbg(&serial->dev->dev, "%s %p\n", __func__, serial);

        for (i = 0; i < serial->num_ports; ++i) {
                port = serial->port[i];
                if (!port)
                        continue;
                portdata = usb_get_serial_port_data(port);
                if (!portdata)
                        continue;

                for (j = 0; j < N_IN_URB; j++) {
                        kfree(portdata->in_buffer[j]);
                }
                kfree(portdata);
                usb_set_serial_port_data(port, NULL);
        }
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
static void xr_usb_serial_shutdown(struct usb_serial *serial)
{
        xr_usb_serial_serial_disconnect(serial);
        xr_usb_serial_serial_release(serial);
}
#endif


/* -------------------------------------------------------------------------- */

static int xr_usb_serial_calc_num_ports(struct usb_serial *serial)
{
        return 1;
}
	 

static int xr_usb_serial_probe(struct usb_serial *serial,
                         const struct usb_device_id *id)
{
        struct usb_interface                    *intf                     = serial->interface;
        unsigned char                           *buffer                   = intf->altsetting->extra;
        int                                      buflen                   = intf->altsetting->extralen;
        struct usb_device                       *usb_dev                  = interface_to_usbdev(intf);
        struct usb_cdc_union_desc               *union_header             = NULL;
        struct usb_cdc_country_functional_desc  *cfd                      = NULL;
        u8                                       ac_management_function   = 0;
        u8                                       call_management_function = 0;
        int                                      call_interface_num       = -1;
        int                                      data_interface_num;
        struct usb_interface                    *control_interface;
        struct usb_interface                    *data_interface;
        struct usb_endpoint_descriptor          *epctrl;
        struct usb_endpoint_descriptor          *epread;
        struct usb_endpoint_descriptor          *epwrite;
		struct usb_serial_port			*port;
        struct xr_usb_serial_serial_private           *serial_priv;
		
         /* Assume that there's exactly one serial port. */
	    port = serial->port[0];
		 
        if (!buffer) {
                dev_err(&intf->dev, "Weird descriptor references\n");
                return -EINVAL;
        }

        if (!buflen) {
                if (intf->cur_altsetting->endpoint->extralen && intf->cur_altsetting->endpoint->extra) {
                        if (debug) dev_dbg(&intf->dev, "Seeking extra descriptors on endpoint\n");
                        buflen = intf->cur_altsetting->endpoint->extralen;
                        buffer = intf->cur_altsetting->endpoint->extra;
                } else {
                        dev_err(&intf->dev, "Zero length descriptor references\n");
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
                case USB_CDC_COUNTRY_TYPE: /* export through sysfs */
                        cfd = (struct usb_cdc_country_functional_desc *)buffer;
                        break;
                case USB_CDC_HEADER_TYPE: /* maybe check version */
                        break; /* for now we ignore it */
                case USB_CDC_ACM_TYPE:
                        ac_management_function = buffer[3];
                        break;
                case USB_CDC_CALL_MANAGEMENT_TYPE:
                        call_management_function = buffer[3];
                        call_interface_num = buffer[4];
                        if ((call_management_function & 3) != 3) {
/*              dev_err(&intf->dev, "This device cannot do calls on its own. It is no modem.\n"); */
                        }
                        break;
                default:
                        /* there are LOTS more CDC descriptors that
                         * could legitimately be found here.
                         */
                        if (debug) dev_dbg(&intf->dev, "Ignoring descriptor: "
                                           "type %02x, length %d\n",
                                           buffer[2], buffer[0]);
                        break;
                }
        next_desc:
                buflen -= buffer[0];
                buffer += buffer[0];
        }

        if (!union_header) {
                if (call_interface_num > 0) {
                        if (debug) dev_dbg(&intf->dev, "No union descriptor, using call management descriptor\n");
                        data_interface = usb_ifnum_to_if(usb_dev, (data_interface_num = call_interface_num));
                        control_interface = intf;
                } else {
                        if (debug) dev_dbg(&intf->dev, "No union descriptor, giving up\n");
                        return -ENODEV;
                }
        } else {
                control_interface = usb_ifnum_to_if(usb_dev, union_header->bMasterInterface0);
                data_interface    = usb_ifnum_to_if(usb_dev, (data_interface_num = union_header->bSlaveInterface0));
                if (!control_interface || !data_interface) {
                        if (debug) dev_dbg(&intf->dev, "no interfaces\n");
                        return -ENODEV;
                }
        }

        if (data_interface_num != call_interface_num)
                if (debug) dev_dbg(&intf->dev, "Separate call control interface. That is not fully supported.\n");

        /* workaround for switched interfaces */
        if (data_interface->cur_altsetting->desc.bInterfaceClass != CDC_DATA_INTERFACE_TYPE) {
                if (control_interface->cur_altsetting->desc.bInterfaceClass == CDC_DATA_INTERFACE_TYPE) {
                        struct usb_interface *t;
/*          if (debug) dev_dbg(&intf->dev, "Your device has switched interfaces.\n"); */

                        t = control_interface;
                        control_interface = data_interface;
                        data_interface = t;
                } else {
                        return -EINVAL;
                }
        }

        /* Accept probe requests only for the control interface */
        if (intf != control_interface) {
/*      if (debug) dev_dbg(&intf->dev, "Skipping data interface %p\n", intf); */
                return -ENODEV;
        }
/*  if (debug) dev_dbg(&intf->dev, "Grabbing control interface %p\n", intf); */

        if (usb_interface_claimed(data_interface)) { /* valid in this context */
                if (debug) dev_dbg(&intf->dev, "The data interface isn't available\n");
                return -EBUSY;
        }

        if (data_interface->cur_altsetting->desc.bNumEndpoints < 2)
                return -EINVAL;

        epctrl  = &control_interface->cur_altsetting->endpoint[0].desc;
        epread  = &data_interface->cur_altsetting->endpoint[0].desc;
        epwrite = &data_interface->cur_altsetting->endpoint[1].desc;
        if (!usb_endpoint_dir_in(epread)) {
                struct usb_endpoint_descriptor *t;
                t   = epread;
                epread  = epwrite;
                epwrite = t;
        }

        /* The documentation suggests that we allocate private storage
         * with the attach() entry point, but we can't allow the data
         * interface to remain unclaimed until then; so we need
         * somewhere to save the claimed interface now. */
        if (!(serial_priv = kzalloc(sizeof(struct xr_usb_serial_serial_private), GFP_KERNEL))) {
                if (debug) dev_dbg(&intf->dev, "out of memory\n");
                goto alloc_fail;
        }
		
        usb_set_serial_data(serial, serial_priv);

/*  if (debug) dev_dbg(&intf->dev, "Claiming data interface %p\n", data_interface); */
        usb_driver_claim_interface(&xr_usb_serial_driver, data_interface, NULL);

        /* Don't set the data interface private data.  When we
         * disconnect we test this field against NULL to discover
         * whether we're dealing with the control or data
         * interface. */
        serial_priv->data_interface = data_interface;
     	serial_priv->DeviceVendor = id->idVendor;
		serial_priv->DeviceProduct = id->idProduct;
        return 0;

alloc_fail:
        return -ENOMEM;
}

static void xr_usb_serial_disconnect(struct usb_interface *interface)
{
        struct usb_serial               *serial = usb_get_intfdata(interface);
        struct xr_usb_serial_serial_private   *serial_priv;

        if (debug) dev_dbg(&interface->dev, "%s %p\n", __func__, interface);

        if (!serial) {
                /* NULL interface private data means that we're
                 * dealing with the data interface and not the control
                 * interface.  So we just bail and let the real clean
                 * up happen later when the control interface is
                 * disconnected. */
                return;
        }

        serial_priv = usb_get_serial_data(serial);

/*  if (debug) dev_dbg(&interface->dev, "Releasing data interface %p.\n", serial_priv->data_interface); */
        usb_driver_release_interface(&xr_usb_serial_driver, serial_priv->data_interface);

        kfree(serial_priv);
        usb_set_serial_data(serial, NULL);

/*  if (debug) dev_dbg(&interface->dev, "Disconnecting control interface\n"); */
        usb_serial_disconnect(interface);
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
static void xr_usb_serial_tty_throttle(struct usb_serial_port *port)
#else
static void xr_usb_serial_tty_throttle(struct tty_struct *tty)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
#else
        struct usb_serial_port *port = tty->driver_data;
#endif
    struct xr_usb_serial_port_private     *portdata = usb_get_serial_port_data(port);

	spin_lock_irq(&portdata->lock);
	portdata->throttle_req = 1;
	spin_unlock_irq(&portdata->lock);
}




#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
static void xr_usb_serial_tty_unthrottle(struct usb_serial_port *port)
#else
static void xr_usb_serial_tty_unthrottle(struct tty_struct *tty)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
#else
        struct usb_serial_port *port = tty->driver_data;
#endif
    struct xr_usb_serial_port_private     *portdata = usb_get_serial_port_data(port);
    unsigned int was_throttled;
	int j,err;
	struct urb *urb;
	spin_lock_irq(&portdata->lock);
	was_throttled = portdata->throttled;
	portdata->throttled = 0;
	portdata->throttle_req = 0;
	spin_unlock_irq(&portdata->lock);
	if (was_throttled)
	for (j = 0; j < N_IN_URB; ++j)
	{
	  urb = portdata->in_urbs[j];
	  err = usb_submit_urb(urb, GFP_ATOMIC);
      if (err)
         dev_err(&port->dev, "resubmit read urb failed. (%d)\n", err);
	}

}

static struct usb_serial_driver xr_usb_serial_device = {
        .driver = {
                .owner =    THIS_MODULE,
                .name =     "xr_usb_serial",
        },
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 37)
		.usb_driver	    = &xr_usb_serial_driver,
#endif
        .description        = "EXAR USB serial port",
        .id_table           = id_table,
        .calc_num_ports     = xr_usb_serial_calc_num_ports,
        .probe              = xr_usb_serial_probe,
        .open               = xr_usb_serial_open,
        .close              = xr_usb_serial_close,
        .write              = xr_usb_serial_write,
        .write_room         = xr_usb_serial_write_room,
        .ioctl              = xr_usb_serial_ioctl,
        .set_termios        = xr_usb_serial_set_termios,
        .break_ctl          = xr_usb_serial_break_ctl,
        .tiocmget           = xr_usb_serial_tiocmget,
        .tiocmset           = xr_usb_serial_tiocmset,
        .attach             = xr_usb_serial_attach,
        .throttle           = xr_usb_serial_tty_throttle,
        .unthrottle         = xr_usb_serial_tty_unthrottle,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
        .shutdown          = xr_usb_serial_shutdown,
#else
        .disconnect        = xr_usb_serial_serial_disconnect,
        .release           = xr_usb_serial_serial_release,
#endif
        .read_int_callback  = xr_usb_serial_int_callback,
};


/* Functions used by new usb-serial code. */
static int __init xr_usb_serial_init(void)
{
        int retval;
        retval = usb_serial_register(&xr_usb_serial_device);
        if (retval)
                goto failed_device_register;


        retval = usb_register(&xr_usb_serial_driver);
        if (retval)
                goto failed_driver_register;

        printk(KERN_INFO DRIVER_DESC ": " DRIVER_VERSION "\n");

        return 0;

failed_driver_register:
        usb_serial_deregister(&xr_usb_serial_device);
failed_device_register:
        return retval;
}


static void __exit xr_usb_serial_exit(void)
{
        usb_deregister(&xr_usb_serial_driver);
        usb_serial_deregister(&xr_usb_serial_device);
}

module_init(xr_usb_serial_init);
module_exit(xr_usb_serial_exit);


MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug messages");
