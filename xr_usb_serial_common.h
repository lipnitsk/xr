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
 * CMSPAR, some architectures can't have space and mark parity.
 */

#ifndef CMSPAR
#define CMSPAR			0
#endif

/*
 * Major and minor numbers.
 */

#define XR_USB_SERIAL_TTY_MAJOR		    266
#define XR_USB_SERIAL_TTY_MINORS		32

/*
 * Requests.
 */

#define USB_RT_XR_USB_SERIAL		(USB_TYPE_CLASS | USB_RECIP_INTERFACE)

/*
 * Output control lines.
 */

#define XR_USB_SERIAL_CTRL_DTR		0x01
#define XR_USB_SERIAL_CTRL_RTS		0x02
/*
 * Input control lines and line errors.
 */

#define XR_USB_SERIAL_CTRL_DCD		0x01
#define XR_USB_SERIAL_CTRL_DSR		0x02
#define XR_USB_SERIAL_CTRL_BRK		0x04
#define XR_USB_SERIAL_CTRL_RI		0x08

#define XR_USB_SERIAL_CTRL_FRAMING	0x10
#define XR_USB_SERIAL_CTRL_PARITY		0x20
#define XR_USB_SERIAL_CTRL_OVERRUN	0x40

/*
 * Internal driver structures.
 */

/*
 * The only reason to have several buffers is to accommodate assumptions
 * in line disciplines. They ask for empty space amount, receive our URB size,
 * and proceed to issue several 1-character writes, assuming they will fit.
 * The very first write takes a complete URB. Fortunately, this only happens
 * when processing onlcr, so we only need 2 buffers. These values must be
 * powers of 2.
 */
#define XR_USB_SERIAL_NW  16
#define XR_USB_SERIAL_NR  16

struct reg_addr_map {
	unsigned int    uart_enable_addr;
	unsigned int    uart_format_addr;
	unsigned int    uart_flow_addr;
	unsigned int    uart_loopback_addr;
    unsigned int    uart_xon_char_addr;
	unsigned int    uart_xoff_char_addr;
	unsigned int    uart_gpio_mode_addr;
	unsigned int    uart_gpio_dir_addr;
	unsigned int    uart_gpio_set_addr;
	unsigned int    uart_gpio_clr_addr;
	unsigned int    uart_gpio_status_addr;
	unsigned int    tx_break_addr;
	unsigned int    uart_custom_driver;
	unsigned int    uart_low_latency;
};



#define CDC_DATA_INTERFACE_TYPE	0x0a


#define UART_FLOW                                          0x006
#define UART_FLOW_MODE_M                                   0x7
#define UART_FLOW_MODE_S                                   0
#define UART_FLOW_MODE                                     0x007

#define UART_FLOW_MODE_NONE                                (0x0 << UART_FLOW_MODE_S)
#define UART_FLOW_MODE_HW                                  (0x1 << UART_FLOW_MODE_S)
#define UART_FLOW_MODE_SW                                  (0x2 << UART_FLOW_MODE_S)
#define UART_FLOW_MODE_ADDR_MATCH                          (0x3 << UART_FLOW_MODE_S)
#define UART_FLOW_MODE_ADDR_MATCH_TX                       (0x4 << UART_FLOW_MODE_S)

#define UART_FLOW_HALF_DUPLEX_M                            0x1
#define UART_FLOW_HALF_DUPLEX_S                            3
#define UART_FLOW_HALF_DUPLEX                              0x008

#define UART_LOOPBACK_CTL                                  0x016
#define LOOPBACK_ENABLE_TX_RX                              1
#define LOOPBACK_ENABLE_RTS_CTS                            2
#define LOOPBACK_ENABLE_DTR_DSR                            4

#define RAMCTL_BUFFER_PARITY                               0x1
#define RAMCTL_BUFFER_BREAK                                0x2
#define RAMCTL_BUFFER_FRAME                                0x4
#define RAMCTL_BUFFER_OVERRUN                              0x8
#define CUSTOM_DRIVER_ACTIVE                               0x1


/* constants describing various quirks and errors */
#define NO_UNION_NORMAL			1
#define SINGLE_RX_URB			2
#define NO_CAP_LINE			4
#define NOT_A_MODEM			8
#define NO_DATA_INTERFACE		16
#define IGNORE_DEVICE			32


#define UART_ENABLE_TX                     1
#define UART_ENABLE_RX                     2

#define UART_GPIO_CLR_DTR                0x8
#define UART_GPIO_SET_DTR                0x8
#define UART_GPIO_CLR_RTS                0x20         
#define UART_GPIO_SET_RTS                0x20

#define LOOPBACK_ENABLE_TX_RX             1
#define LOOPBACK_ENABLE_RTS_CTS           2
#define LOOPBACK_ENABLE_DTR_DSR           4


#define UART_GPIO_MODE_SEL_GPIO          0x0
#define UART_GPIO_MODE_SEL_RTS_CTS       0x1

#define XR2280x_FUNC_MGR_OFFSET           0x40




