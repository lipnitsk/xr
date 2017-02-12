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

#define XR_SET_MAP_XR2280X              5
#define XR_GET_MAP_XR2280X              5

#define XR_SET_MAP_XR21B142X             0
#define XR_GET_MAP_XR21B142X             0

#define XR_SET_MAP_XR21V141X             0
#define XR_GET_MAP_XR21V141X             1

#define XR_SET_MAP_XR21B1411             0
#define XR_GET_MAP_XR21B1411             1

int xr_usb_serial_disable(struct usb_serial_port *port);
int xr_usb_serial_enable(struct usb_serial_port *port);

int xr_usb_serial_set_reg(struct usb_serial_port *port,int regnum, int value)
{
    struct usb_serial *serial = port->serial; 
	int result;
	struct xr_usb_serial_port_private *portdata = usb_get_serial_port_data(port);
	int block = 0;
	dev_dbg(&port->serial->dev->dev, "%s block:%d 0x%02x = 0x%02x\n", __func__,block,regnum, value);
	if((portdata->DeviceProduct&0xfff0) == 0x1400)
	{
	    int XR2280xaddr; 
	    XR2280xaddr = XR2280x_FUNC_MGR_OFFSET + regnum; 
		result = usb_control_msg(serial->dev,                     /* usb device */
	                             usb_sndctrlpipe(serial->dev, 0), /* endpoint pipe */
	                             XR_SET_MAP_XR2280X,                      /* request */
	                             USB_DIR_OUT | USB_TYPE_VENDOR,   /* request_type */
	                             value,                           /* request value */
	                             XR2280xaddr,           /* index */
	                             NULL,                            /* data */
	                             0,                               /* size */
	                             5000);                           /* timeout */

	}
	else if((portdata->DeviceProduct == 0x1410)||
		   (portdata->DeviceProduct == 0x1412)||
		   (portdata->DeviceProduct == 0x1414))
	{
	    /* Bulk OUT endpoints 0x1..0x4 map to register blocks 0..3 */
		if(portdata->block)
	      block = portdata->block - 1;
 		result = usb_control_msg(serial->dev,					  /* usb device */
										usb_sndctrlpipe(serial->dev, 0), /* endpoint pipe */
										XR_SET_MAP_XR21V141X, 					 /* request */
										USB_DIR_OUT | USB_TYPE_VENDOR,	 /* request_type */
										value,							 /* request value */
										regnum | (block << 8),			 /* index */
										NULL,							 /* data */
										0,								 /* size */
										5000);							 /* timeout */
	}
	else if(portdata->DeviceProduct == 0x1411)
	{
	  result = usb_control_msg(serial->dev,					  /* usb device */
										usb_sndctrlpipe(serial->dev, 0), /* endpoint pipe */
										XR_SET_MAP_XR21B1411, 					 /* request */
										USB_DIR_OUT | USB_TYPE_VENDOR,	 /* request_type */
										value,							 /* request value */
										regnum ,			             /* index */
										NULL,							 /* data */
										0,								 /* size */
										5000);							 /* timeout */
	}
		   
	else if((portdata->DeviceProduct == 0x1420)||
		   (portdata->DeviceProduct == 0x1422)||
		   (portdata->DeviceProduct == 0x1424))
	{
	    /* Bulk OUT endpoints 0x4..0x7 map to register blocks# 0 2 4 6*/
	    block = (portdata->block - 4)*2;
        result = usb_control_msg(serial->dev,                     /* usb device */
                                 usb_sndctrlpipe(serial->dev, 0), /* endpoint pipe */
                                 XR_SET_MAP_XR21B142X,                   /* request */
                                 USB_DIR_OUT | USB_TYPE_VENDOR | 1,   /* request_type */
                                 value,                           /* request value */
                                 regnum | (block << 8),           /* index */
                                 NULL,                            /* data */
                                 0,                               /* size */
                                 5000);                           /* timeout */
	}
	else
	{
	    result = -1;
	}
	if(result < 0)
		dev_dbg(&port->serial->dev->dev, "%s block(%d) Error:%d\n", __func__,block,result);
    return result;
	
       
}

int xr_usb_serial_set_reg_ext(struct usb_serial_port *port,int block,int regnum, int value)
{
    struct usb_serial *serial = port->serial; 
	int result;
	struct xr_usb_serial_port_private *portdata = usb_get_serial_port_data(port);
	dev_dbg(&port->serial->dev->dev, "%s block:%d 0x%02x = 0x%02x\n", __func__,block,regnum, value);
	if((portdata->DeviceProduct&0xfff0) == 0x1400)
	{
	    int XR2280xaddr; 
	    XR2280xaddr = XR2280x_FUNC_MGR_OFFSET + regnum; 
		result = usb_control_msg(serial->dev,                     /* usb device */
	                             usb_sndctrlpipe(serial->dev, 0), /* endpoint pipe */
	                             XR_SET_MAP_XR2280X,                      /* request */
	                             USB_DIR_OUT | USB_TYPE_VENDOR,   /* request_type */
	                             value,                           /* request value */
	                             XR2280xaddr,           /* index */
	                             NULL,                            /* data */
	                             0,                               /* size */
	                             5000);                           /* timeout */

	}
	else if((portdata->DeviceProduct == 0x1410)||
		   (portdata->DeviceProduct == 0x1412)||
		   (portdata->DeviceProduct == 0x1414))
	{
	    /* Bulk OUT endpoints 0x1..0x4 map to register blocks 0..3 */
 		result = usb_control_msg(serial->dev,					  /* usb device */
										usb_sndctrlpipe(serial->dev, 0), /* endpoint pipe */
										XR_SET_MAP_XR21V141X, 					 /* request */
										USB_DIR_OUT | USB_TYPE_VENDOR,	 /* request_type */
										value,							 /* request value */
										regnum | (block << 8),			 /* index */
										NULL,							 /* data */
										0,								 /* size */
										5000);							 /* timeout */
	}
	else if(portdata->DeviceProduct == 0x1411)
	{
	   result = usb_control_msg(serial->dev,					  /* usb device */
										usb_sndctrlpipe(serial->dev, 0), /* endpoint pipe */
										XR_SET_MAP_XR21B1411, 					 /* request */
										USB_DIR_OUT | USB_TYPE_VENDOR,	 /* request_type */
										value,							 /* request value */
										regnum ,			             /* index */
										NULL,							 /* data */
										0,								 /* size */
										5000);							 /* timeout */
	}
	else if((portdata->DeviceProduct == 0x1420)||
		   (portdata->DeviceProduct == 0x1422)||
		   (portdata->DeviceProduct == 0x1424))
	{
	    /* Bulk OUT endpoints 0x4..0x7 map to register blocks 0 2 4 6*/
	    block = block*2;
        result = usb_control_msg(serial->dev,                     /* usb device */
                                 usb_sndctrlpipe(serial->dev, 0), /* endpoint pipe */
                                 XR_SET_MAP_XR21B142X,                   /* request */
                                 USB_DIR_OUT | USB_TYPE_VENDOR | 1,   /* request_type */
                                 value,                           /* request value */
                                 regnum | (block << 8),           /* index */
                                 NULL,                            /* data */
                                 0,                               /* size */
                                 5000);                           /* timeout */
	}
	else
	{
	    result = -1;
	}
	if(result < 0)
		dev_dbg(&port->serial->dev->dev, "%s block(%d) Error:%d\n", __func__,block,result);
    return result;
	
       
}

int xr_usb_serial_get_reg(struct usb_serial_port *port,int regnum, short *value)
{
	int result;
    struct usb_serial *serial = port->serial;
	struct xr_usb_serial_port_private *portdata = usb_get_serial_port_data(port);
	int block = 0;
	char val = 0;
	if((portdata->DeviceProduct&0xfff0) == 0x1400)
	{
		int XR2280xaddr = XR2280x_FUNC_MGR_OFFSET + regnum; 
    	result = usb_control_msg(serial->dev,                     /* usb device */
                                 usb_rcvctrlpipe(serial->dev, 0), /* endpoint pipe */
                                 XR_GET_MAP_XR2280X,                     /* request */
                                 USB_DIR_IN | USB_TYPE_VENDOR ,    /* request_type */
                                 0,                               /* request value */
                                 XR2280xaddr,                    /* index */
                                 value,                           /* data */
                                 2,                               /* size */
                                 5000);                           /* timeout */
		
				
		
	}
	else if((portdata->DeviceProduct == 0x1410)||
		   (portdata->DeviceProduct == 0x1412)||
		   (portdata->DeviceProduct == 0x1414))
	{
	    /* Bulk OUT endpoints 0x1..0x4 map to register blocks 0..3 */
	   if(portdata->block)
	      block = portdata->block - 1;
       result = usb_control_msg(serial->dev,                     /* usb device */
                                 usb_rcvctrlpipe(serial->dev, 0), /* endpoint pipe */
                                 XR_GET_MAP_XR21V141X,                     /* request */
                                 USB_DIR_IN | USB_TYPE_VENDOR,    /* request_type */
                                 0,                               /* request value */
                                 regnum | (block << 8),              /* index */
                                 &val,                           /* data */
                                 1,                               /* size */
                                 5000);                           /* timeout */
	   *value = val;
	}
	else if(portdata->DeviceProduct == 0x1411)
	{
	  result = usb_control_msg(serial->dev,                     /* usb device */
                                 usb_rcvctrlpipe(serial->dev, 0), /* endpoint pipe */
                                 XR_GET_MAP_XR21B1411,                     /* request */
                                 USB_DIR_IN | USB_TYPE_VENDOR,    /* request_type */
                                 0,                               /* request value */
                                 regnum,                         /* index */
                                 &val,                           /* data */
                                 2,                               /* size */
                                 5000);                           /* timeout */
	   *value = val;
	}
	else if((portdata->DeviceProduct == 0x1420)||
		   (portdata->DeviceProduct == 0x1422)||
		   (portdata->DeviceProduct == 0x1424))
	{
	  /* Bulk OUT endpoints 0x4..0x7 map to register blocks# 0 2 4 6 */
	  block = (portdata->block - 4)*2; 
      result = usb_control_msg(serial->dev,                     /* usb device */
                                 usb_rcvctrlpipe(serial->dev, 0), /* endpoint pipe */
                                 XR_GET_MAP_XR21B142X,                     /* request */
                                 USB_DIR_IN | USB_TYPE_VENDOR | 1,    /* request_type */
                                 0,                               /* request value */
                                 regnum | (block << 8),              /* index */
                                 value,                           /* data */
                                 2,                               /* size */
                                 5000);                           /* timeout */
	}
	else
	{
	    result = -1;
	}
	
	if(result < 0)
		dev_dbg(&port->serial->dev->dev, "%s block:%d Reg 0x%x Error:%d\n", __func__,block,regnum,result);
	else
	    dev_dbg(&port->serial->dev->dev, "%s block:%d 0x%x = 0x%04x\n", __func__,block,regnum, *value);
	
	return result;

}
int xr_usb_serial_get_reg_ext(struct usb_serial_port *port,int block,int regnum, short *value)
{
	int result;
    struct usb_serial *serial = port->serial;
	struct xr_usb_serial_port_private *portdata = usb_get_serial_port_data(port);
	char val = 0;
	if((portdata->DeviceProduct&0xfff0) == 0x1400)
	{
		int XR2280xaddr = XR2280x_FUNC_MGR_OFFSET + regnum; 
    	result = usb_control_msg(serial->dev,                     /* usb device */
                                 usb_rcvctrlpipe(serial->dev, 0), /* endpoint pipe */
                                 XR_GET_MAP_XR2280X,                     /* request */
                                 USB_DIR_IN | USB_TYPE_VENDOR ,    /* request_type */
                                 0,                               /* request value */
                                 XR2280xaddr,                    /* index */
                                 value,                           /* data */
                                 2,                               /* size */
                                 5000);                           /* timeout */
		
				
		
	}
	else if((portdata->DeviceProduct == 0x1410)||
		   (portdata->DeviceProduct == 0x1412)||
		   (portdata->DeviceProduct == 0x1414))
	{
	    /* Bulk OUT endpoints 0x1..0x4 map to register blocks 0..3 */
	    result = usb_control_msg(serial->dev,                     /* usb device */
                                 usb_rcvctrlpipe(serial->dev, 0), /* endpoint pipe */
                                 XR_GET_MAP_XR21V141X,                     /* request */
                                 USB_DIR_IN | USB_TYPE_VENDOR,    /* request_type */
                                 0,                               /* request value */
                                 regnum | (block << 8),              /* index */
                                 &val,                           /* data */
                                 1,                               /* size */
                                 5000);                           /* timeout */
		*value = val;
		
	}
	else if(portdata->DeviceProduct == 0x1411)
	{
	   result = usb_control_msg(serial->dev,                     /* usb device */
                                 usb_rcvctrlpipe(serial->dev, 0), /* endpoint pipe */
                                 XR_GET_MAP_XR21B1411,                     /* request */
                                 USB_DIR_IN | USB_TYPE_VENDOR,    /* request_type */
                                 0,                               /* request value */
                                 regnum,                         /* index */
                                 &val,                           /* data */
                                 2,                               /* size */
                                 5000);                           /* timeout */
	   *value = val;
	}
	else if((portdata->DeviceProduct == 0x1420)||
		   (portdata->DeviceProduct == 0x1422)||
		   (portdata->DeviceProduct == 0x1424))
	{
	  /* Bulk OUT endpoints0 1 2 3 map to register blocks 0 2 4 6 */
	  block = block*2; 
      result = usb_control_msg(serial->dev,                     /* usb device */
                                 usb_rcvctrlpipe(serial->dev, 0), /* endpoint pipe */
                                 XR_GET_MAP_XR21B142X,                     /* request */
                                 USB_DIR_IN | USB_TYPE_VENDOR | 1,    /* request_type */
                                 0,                               /* request value */
                                 regnum | (block << 8),              /* index */
                                 value,                           /* data */
                                 2,                               /* size */
                                 5000);                           /* timeout */
	}
	
	else
	{
	    result = -1;
	}
	
	if(result < 0)
		dev_dbg(&port->serial->dev->dev, "%s block:%d Reg 0x%x Error:%d\n", __func__,block,regnum,result);
	else
	    dev_dbg(&port->serial->dev->dev, "%s block:%d 0x%x = 0x%04x\n", __func__,block,regnum, *value);
	
	return result;

}

struct xr21v141x_baud_rate
{
	unsigned int tx;
	unsigned int rx0;
	unsigned int rx1;
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

static int xr21v141x_set_baud_rate(struct usb_serial_port *port, unsigned int rate)
{
	unsigned int 	divisor = 48000000 / rate;
	unsigned int 	i 	= ((32 * 48000000) / rate) & 0x1f;
	unsigned int 	tx_mask = xr21v141x_baud_rates[i].tx;
	unsigned int 	rx_mask = (divisor & 1) ? xr21v141x_baud_rates[i].rx1 : xr21v141x_baud_rates[i].rx0;
	
	dev_dbg(&port->serial->dev->dev, "Setting baud rate to %d: i=%u div=%u tx=%03x rx=%03x\n", rate, i, divisor, tx_mask, rx_mask);

	xr_usb_serial_set_reg(port,UART_CLOCK_DIVISOR_0, (divisor >>  0) & 0xff);
	xr_usb_serial_set_reg(port,UART_CLOCK_DIVISOR_1, (divisor >>  8) & 0xff);
	xr_usb_serial_set_reg(port,UART_CLOCK_DIVISOR_2, (divisor >> 16) & 0xff);
	xr_usb_serial_set_reg(port,UART_TX_CLOCK_MASK_0, (tx_mask >>  0) & 0xff);
	xr_usb_serial_set_reg(port,UART_TX_CLOCK_MASK_1, (tx_mask >>  8) & 0xff);
	xr_usb_serial_set_reg(port,UART_RX_CLOCK_MASK_0, (rx_mask >>  0) & 0xff);
	xr_usb_serial_set_reg(port,UART_RX_CLOCK_MASK_1, (rx_mask >>  8) & 0xff);
	
	return 0;
}
/* devices aren't required to support these requests.
 * the cdc xr_usb_serial descriptor tells whether they do...
 */
int xr_usb_serial_set_control(struct usb_serial_port *port, unsigned int control)
{
    int ret = 0;
	struct xr_usb_serial_port_private *portdata = usb_get_serial_port_data(port);
	
    if((portdata->DeviceProduct&0xfff0) == 0x1400)
    {
	   ret = xr_usb_serial_ctrl_msg(port, USB_CDC_REQ_SET_CONTROL_LINE_STATE, control, NULL, 0);
    }
	else if((portdata->DeviceProduct == 0x1410)||
		   (portdata->DeviceProduct == 0x1412)||
		   (portdata->DeviceProduct == 0x1414))
	{
		if (control & XR_USB_SERIAL_CTRL_DTR) 
		   xr_usb_serial_set_reg(port,portdata->reg_map.uart_gpio_clr_addr, 0x08);
		else
		   xr_usb_serial_set_reg(port,portdata->reg_map.uart_gpio_set_addr, 0x08);

		if (control & XR_USB_SERIAL_CTRL_RTS) 
		   xr_usb_serial_set_reg(port,portdata->reg_map.uart_gpio_clr_addr, 0x20);
		else
		   xr_usb_serial_set_reg(port,portdata->reg_map.uart_gpio_set_addr, 0x20);
	}
	else if(portdata->DeviceProduct == 0x1411)
	{
	  ret = xr_usb_serial_ctrl_msg(port, USB_CDC_REQ_SET_CONTROL_LINE_STATE, control, NULL, 0);
	}
	else if((portdata->DeviceProduct == 0x1420)||
		   (portdata->DeviceProduct == 0x1422)||
		   (portdata->DeviceProduct == 0x1424))
	{
	  ret = xr_usb_serial_ctrl_msg(port, USB_CDC_REQ_SET_CONTROL_LINE_STATE, control, NULL, 0);
	}
	else
	{
	   ret =-1; 
	}
	return ret;
}

int xr_usb_serial_set_line(struct usb_serial_port *port, struct usb_cdc_line_coding* line)
 {
	 int ret = 0;
	 unsigned int format_size;
	 unsigned int format_parity;
	 unsigned int format_stop;
	 struct xr_usb_serial_port_private *portdata = usb_get_serial_port_data(port);
	 
	 if((portdata->DeviceProduct&0xfff0) == 0x1400)
	 {
		ret = xr_usb_serial_ctrl_msg(port, USB_CDC_REQ_SET_LINE_CODING, 0, line, sizeof *(line));
	 }
	 else if((portdata->DeviceProduct == 0x1410)||
		    (portdata->DeviceProduct == 0x1412)||
		    (portdata->DeviceProduct == 0x1414))
	 {
	    if(line->dwDTERate == 0) return -1;
		xr21v141x_set_baud_rate(port,line->dwDTERate);
		format_size = line->bDataBits;
		format_parity = line->bParityType;
		format_stop = line->bCharFormat;
		xr_usb_serial_set_reg(port,
			                  portdata->reg_map.uart_format_addr,
			                  (format_size << 0) | (format_parity << 4) | (format_stop << 7) );
		
	 }
	 else if(portdata->DeviceProduct == 0x1411)
	 {
	   ret = xr_usb_serial_ctrl_msg(port, USB_CDC_REQ_SET_LINE_CODING, 0, line, sizeof *(line));
	 }
	 else if((portdata->DeviceProduct == 0x1420)||
		    (portdata->DeviceProduct == 0x1422)||
		    (portdata->DeviceProduct == 0x1424))
	 {
	   ret = xr_usb_serial_ctrl_msg(port, USB_CDC_REQ_SET_LINE_CODING, 0, line, sizeof *(line));
	 }
	 return ret;
 }
 int xr_usb_serial_set_flow_mode(struct usb_serial_port *port, 
 	                                     struct tty_struct *tty, unsigned int cflag)
 {
	unsigned int flow;
	unsigned int gpio_mode;
	struct xr_usb_serial_port_private *portdata = usb_get_serial_port_data(port);
	
	if (cflag & CRTSCTS)
	{
	    dev_dbg(&port->serial->dev->dev, "xr_usb_serial_set_flow_mode:hardware\n");
	    flow      = UART_FLOW_MODE_HW;
	    gpio_mode = UART_GPIO_MODE_SEL_RTS_CTS;
	} 
	else if (I_IXOFF(tty) || I_IXON(tty))
	{
	    unsigned char   start_char = START_CHAR(tty);
	    unsigned char   stop_char  = STOP_CHAR(tty);
        dev_dbg(&port->serial->dev->dev, "xr_usb_serial_set_flow_mode:software\n");
	    flow      = UART_FLOW_MODE_SW;
	    gpio_mode = UART_GPIO_MODE_SEL_GPIO;

	    xr_usb_serial_set_reg(port,portdata->reg_map.uart_xon_char_addr, start_char);
	    xr_usb_serial_set_reg(port,portdata->reg_map.uart_xoff_char_addr, stop_char);
	}
	else
	{
	    dev_dbg(&port->serial->dev->dev, "xr_usb_serial_set_flow_mode:none\n");
	    flow      = UART_FLOW_MODE_NONE;
	    gpio_mode = UART_GPIO_MODE_SEL_GPIO;
	}
    xr_usb_serial_set_reg(port,portdata->reg_map.uart_flow_addr, flow);
    xr_usb_serial_set_reg(port,portdata->reg_map.uart_gpio_mode_addr, gpio_mode);
	return 0;
	 

 }
 
int xr_usb_serial_send_break(struct usb_serial_port *port, int state)
{
	int ret = 0;
	struct xr_usb_serial_port_private *portdata = usb_get_serial_port_data(port);
	if((portdata->DeviceProduct&0xfff0) == 0x1410)
	{
	  if(state)
	     ret = xr_usb_serial_set_reg(port,portdata->reg_map.tx_break_addr,0xffff);
	  else
	  	 ret = xr_usb_serial_set_reg(port,portdata->reg_map.tx_break_addr,0);
	}
	else 
	{
	     ret = xr_usb_serial_ctrl_msg(port,USB_CDC_REQ_SEND_BREAK, state, NULL, 0);
	}
	return ret;
}
#define XR21V1414_WIDE_MODE_OFFSET         3
#define XR21B142X_WIDE_MODE_TX_OFFSET     0x42
#define XR21B142X_WIDE_MODE_RX_OFFSET     0x45


int xr_usb_serial_set_wide_mode(struct usb_serial_port *port, int preciseflags)
{
    int ret = 0;
	struct xr_usb_serial_port_private *portdata = usb_get_serial_port_data(port);
	int block = 0;  
	xr_usb_serial_disable(port);
    if((portdata->DeviceProduct&0xfff0) == 0x1400)
	{
		
	}
	else if((portdata->DeviceProduct == 0x1410)||
		   (portdata->DeviceProduct == 0x1412)||
		   (portdata->DeviceProduct == 0x1414))
	{
	  if(portdata->block)
	      block = portdata->block - 1;
	  xr_usb_serial_set_reg_ext(port, 0x66, block*8 + XR21V1414_WIDE_MODE_OFFSET, preciseflags);
		
	}
	else if(portdata->DeviceProduct == 0x1411)
	{
	  xr_usb_serial_set_reg(port,0xd02, preciseflags);
	}
	else if((portdata->DeviceProduct == 0x1420)||
		   (portdata->DeviceProduct == 0x1422)||
		   (portdata->DeviceProduct == 0x1424))
	{
	  xr_usb_serial_set_reg(port, XR21B142X_WIDE_MODE_TX_OFFSET, preciseflags); 
	  xr_usb_serial_set_reg(port, XR21B142X_WIDE_MODE_RX_OFFSET, preciseflags); 
	}
	xr_usb_serial_enable(port);  
	return ret;
}

int xr_usb_serial_get_gpio_status(struct usb_serial_port *port,short *gpio_status)
{
    int ret = 0;
	struct xr_usb_serial_port_private *portdata = usb_get_serial_port_data(port);
    ret = xr_usb_serial_get_reg(port,portdata->reg_map.uart_gpio_status_addr,gpio_status); 
	return ret;
}
int xr_usb_serial_ctrl_dtr(struct usb_serial_port *port,int high)
{
 int ret = 0;
 struct xr_usb_serial_port_private *portdata = usb_get_serial_port_data(port);
 if(high)
    ret = xr_usb_serial_set_reg(port, portdata->reg_map.uart_gpio_set_addr, 0x08);
 else
 	 ret = xr_usb_serial_set_reg(port, portdata->reg_map.uart_gpio_clr_addr, 0x08);
 return ret;
}

int xr_usb_serial_ctrl_rts(struct usb_serial_port *port,int high)
{
 int ret = 0;
 struct xr_usb_serial_port_private *portdata = usb_get_serial_port_data(port);
 if(high)
    ret = xr_usb_serial_set_reg(port, portdata->reg_map.uart_gpio_set_addr, 0x20);
 else
 	 ret = xr_usb_serial_set_reg(port, portdata->reg_map.uart_gpio_clr_addr, 0x20);
 return ret;
}

#define URM_REG_BLOCK           4
#define URM_ENABLE_BASE        0x010
#define URM_ENABLE_0_TX        0x001
#define URM_ENABLE_0_RX        0x002

int xr_usb_serial_enable(struct usb_serial_port *port)
{
	int ret = 0;
	struct xr_usb_serial_port_private *portdata = usb_get_serial_port_data(port);
	int block = portdata->block;
	if((portdata->DeviceProduct == 0x1410)||
	   (portdata->DeviceProduct == 0x1412)||
	   (portdata->DeviceProduct == 0x1414))
	{
	  ret = xr_usb_serial_set_reg_ext(port,URM_REG_BLOCK,URM_ENABLE_BASE + block,URM_ENABLE_0_TX);
	  
	  ret = xr_usb_serial_set_reg(port,portdata->reg_map.uart_enable_addr,UART_ENABLE_TX | UART_ENABLE_RX);
	  
	  ret = xr_usb_serial_set_reg_ext(port,URM_REG_BLOCK,URM_ENABLE_BASE + block,URM_ENABLE_0_TX | URM_ENABLE_0_RX);
	}
	else 
	{
	  ret = xr_usb_serial_set_reg(port,portdata->reg_map.uart_enable_addr,UART_ENABLE_TX | UART_ENABLE_RX);
	}
	
	return ret;
}
int xr_usb_serial_disable(struct usb_serial_port *port)
{
	int ret = 0;
	struct xr_usb_serial_port_private *portdata = usb_get_serial_port_data(port);
	int block = portdata->block;
	ret = xr_usb_serial_set_reg(port,portdata->reg_map.uart_enable_addr,0);
	if((portdata->DeviceProduct == 0x1410)||
	   (portdata->DeviceProduct == 0x1412)||
	   (portdata->DeviceProduct == 0x1414))
	{
	  ret = xr_usb_serial_set_reg_ext(port,URM_REG_BLOCK,URM_ENABLE_BASE + block,URM_ENABLE_0_TX);
	}
		
	return ret;
}
int xr_usb_serial_set_loopback(struct usb_serial_port *port)
{
	int ret = 0;
	struct xr_usb_serial_port_private *portdata = usb_get_serial_port_data(port);
	xr_usb_serial_disable(port);
	ret = xr_usb_serial_set_reg(port,portdata->reg_map.uart_loopback_addr,0x07);
	xr_usb_serial_enable(port);
	return ret;
}

int xr_usb_serial_set_custom_mode(struct usb_serial_port *port,int val)
{
	int ret = 0;
	struct xr_usb_serial_port_private *portdata = usb_get_serial_port_data(port);
	xr_usb_serial_disable(port);
	ret = xr_usb_serial_set_reg(port,portdata->reg_map.uart_custom_driver,val);
	xr_usb_serial_enable(port);
	return ret;
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

int xr_usb_serial_pre_setup(struct usb_serial_port *port,unsigned short DeviceVendor,unsigned short ProductID)
{
	int ret = 0;
	struct xr_usb_serial_port_private *portdata = usb_get_serial_port_data(port);
    if(portdata == NULL ){
		 printk("xr_usb_serial_pre_setup portdata=%p\n",portdata);
		 return -1;
    	}
	portdata->DeviceVendor = DeviceVendor;
	portdata->DeviceProduct = ProductID;
	dev_dbg(&port->serial->dev->dev, "xr_usb_serial_pre_setup DeviceVendor=0x%x ProductID=0x%x\n",DeviceVendor,ProductID);
	init_xr21b140x_reg_map();
	init_xr21b1411_reg_map();
	init_xr21v141x_reg_map();
	init_xr21b142x_reg_map();
	
	if((ProductID&0xfff0) == 0x1400)
	{
	  memcpy(&(portdata->reg_map),&xr21b140x_reg_map,sizeof(struct reg_addr_map));
	}
	else if(portdata->DeviceProduct == 0x1411)
	{
	  memcpy(&(portdata->reg_map),&xr21b1411_reg_map,sizeof(struct reg_addr_map));
	}
	else if((portdata->DeviceProduct == 0x1410)||
		   (portdata->DeviceProduct == 0x1412)||
		   (portdata->DeviceProduct == 0x1414))
	{
	  memcpy(&(portdata->reg_map),&xr21v141x_reg_map,sizeof(struct reg_addr_map));
	}
	else if((portdata->DeviceProduct == 0x1420)||
		   (portdata->DeviceProduct == 0x1422)||
		   (portdata->DeviceProduct == 0x1424))
	{
	  memcpy(&(portdata->reg_map),&xr21b142x_reg_map,sizeof(struct reg_addr_map));
	 
	}
	else
	{
	  ret = -1;
	}
	
	if(portdata->reg_map.uart_custom_driver)
	  xr_usb_serial_set_reg(port, portdata->reg_map.uart_custom_driver, 1);

	xr_usb_serial_set_reg(port, portdata->reg_map.uart_gpio_mode_addr, 0); 
	xr_usb_serial_set_reg(port, portdata->reg_map.uart_gpio_dir_addr, 0x28);  
    xr_usb_serial_set_reg(port, portdata->reg_map.uart_gpio_set_addr, UART_GPIO_SET_DTR | UART_GPIO_SET_RTS); 
		
    return ret;
   
}

	

