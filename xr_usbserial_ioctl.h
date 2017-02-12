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

#include <linux/ioctl.h>

#define XR_USB_SERIAL_IOC_MAGIC       	'v'

#define XRIOC_GET_REG           	_IOWR(XR_USB_SERIAL_IOC_MAGIC, 1, int)
#define XRIOC_SET_REG           	_IOWR(XR_USB_SERIAL_IOC_MAGIC, 2, int)
#define XRIOC_SET_ADDRESS_MATCH 	_IO(XR_USB_SERIAL_IOC_MAGIC, 3)
#define XRIOC_SET_PRECISE_FLAGS     	_IO(XR_USB_SERIAL_IOC_MAGIC, 4)
#define XRIOC_TEST_MODE         	_IO(XR_USB_SERIAL_IOC_MAGIC, 5)
#define XRIOC_LOOPBACK          	_IO(XR_USB_SERIAL_IOC_MAGIC, 6)
#define XRIOC_GET_STAT          	_IO(XR_USB_SERIAL_IOC_MAGIC, 7)
#define XRIOC_GET_VERSION          	_IO(XR_USB_SERIAL_IOC_MAGIC, 8)


#define XR_ADDRESS_UNICAST_S        	0
#define XR_ADDRESS_BROADCAST_S      	8
#define XR_ADDRESS_MATCH(U, B)          (0x8000000 | ((B) << XR_ADDRESS_BROADCAST_S) | ((U) << XR_ADDRESS_UNICAST_S))
#define XR_ADDRESS_MATCH_DISABLE    	0
