/*
	ccid_uefi.h: USB access routines using the UEFI USB protocol
    Copyright (C) 2014  Ludovic Rousseau

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with this library; if not, write to the Free Software Foundation,
	Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

/*
 * $Id: ccid_uefi.h 5473 2011-01-04 09:52:26Z rousseau $
 */

#ifndef __CCID_UEF_H__
#define __CCID_UEFI_H__
status_t OpenUEFI(unsigned int reader_index, DWORD channel);

status_t OpenUEFIByName(unsigned int reader_index, /*@null@*/ char *device);

status_t WriteUEFI(unsigned int reader_index, unsigned int length,
	unsigned char *Buffer);

status_t ReadUEFI(unsigned int reader_index, unsigned int *length,
	/*@out@*/ unsigned char *Buffer);

status_t CloseUEFI(unsigned int reader_index);

int ControlUSB(int reader_index, int requesttype, int request, int value,
	unsigned char *bytes, unsigned int size);

void duplicate_usb_device(unsigned int reader_index,
	unsigned int new_reader_index);

int InterruptRead(int reader_index, int timeout);

#endif
