/*
	ccid_uefi.c: USB access routines using the UEFI USB protocol
	Copyright (C) 2014	Ludovic Rousseau

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
 * $Id: ccid_usb.c 6890 2014-04-24 13:51:03Z rousseau $
 */

#include <Protocol/UsbIo.h>
#include <Library/UefiLib.h>
#include <CcidDriver.h>

#include <ifdhandler.h>

#include "config.h"
#include "misc.h"
#include "ccid.h"
#include "debug.h"
#include "defs.h"
#include "utils.h"
#include "ccid_ifdhandler.h"


/* write timeout
 * we don't have to wait a long time since the card was doing nothing */
#define USB_WRITE_TIMEOUT (5 * 1000)	/* 5 seconds timeout */

/*
 * Proprietary USB Class (0xFF) are (or are not) accepted
 * A proprietary class is used for devices released before the final CCID
 * specifications were ready.
 * We should not have problems with non CCID devices because the
 * Manufacturer and Product ID are also used to identify the device */
#define ALLOW_PROPRIETARY_CLASS

#define BUS_DEVICE_STRSIZE 32

#define CCID_INTERRUPT_SIZE 8

typedef struct
{
	EFI_USB_INTERFACE_DESCRIPTOR *dev_handle;
	EFI_USB_IO_PROTOCOL *UsbIo;
	uint8_t bus_number;
	uint8_t device_address;

	/*
	 * Endpoints
	 */
	EFI_USB_ENDPOINT_DESCRIPTOR bulk_in;
	EFI_USB_ENDPOINT_DESCRIPTOR bulk_out;
	EFI_USB_ENDPOINT_DESCRIPTOR interrupt;

	/* Number of slots using the same device */
	int real_nb_opened_slots;
	int *nb_opened_slots;

	/*
	 * CCID infos common to USB and serial
	 */
	_ccid_descriptor ccid;

} _usbDevice;

/* The _usbDevice structure must be defined before including ccid_usb.h */
#include "ccid_uefi.h"

/* ne need to initialize to 0 since it is static */
static _usbDevice usbDevice[CCID_DRIVER_MAX_READERS];

//
// Send general device request timeout.
//
// The USB Specification 2.0, section 11.24.1 recommends a value of
// 50 milliseconds.  We use a value of 500 milliseconds to work
// around slower hubs and devices.
//
#define USB_GENERAL_DEVICE_REQUEST_TIMEOUT 500


/*****************************************************************************
 *
 *					OpenUEFI
 *
 ****************************************************************************/
status_t OpenUEFI(unsigned int reader_index, DWORD Channel)
{
	int return_value = STATUS_UNSUCCESSFUL;
	USB_CCID_DEV *UsbCcidDevice = (USB_CCID_DEV *)Channel;
	int index;
	UINT8                       EndpointNumber;
	EFI_USB_ENDPOINT_DESCRIPTOR EndpointDescriptor;
	EFI_STATUS                  Status;
	EFI_USB_IO_PROTOCOL *UsbIo;
	unsigned char * device_descriptor, *ccid_descriptor;
	EFI_USB_DEVICE_REQUEST  DevReq;
	UINT32 Status_uint;

	DEBUG_COMM3("Reader index: %X, Channel: %p", reader_index, UsbCcidDevice);

	UsbIo = UsbCcidDevice->UsbIo;
	usbDevice[reader_index].UsbIo = UsbIo;

	//
	// Get the Device Path Protocol on Controller's handle
	//
	Status = gBS->OpenProtocol (
			UsbCcidDevice->ControllerHandle,
			&gEfiDevicePathProtocolGuid,
			(VOID **) &UsbCcidDevice->DevicePath,
			UsbCcidDevice->DriverBindingHandle,
			UsbCcidDevice->ControllerHandle,
			EFI_OPEN_PROTOCOL_GET_PROTOCOL
			);

	if (EFI_ERROR (Status))
	{
		Log2(PCSC_LOG_DEBUG, "OpenProtocol %d", Status);
		goto ErrorExit;
	}

	/* Get Device descriptor */
	Status = UsbIo->UsbGetDeviceDescriptor(UsbIo,
		&UsbCcidDevice->DeviceDescriptor);
	if (EFI_ERROR (Status))
	{
		Log2(PCSC_LOG_DEBUG, "UsbGetDeviceDescriptor %d", Status);
		goto ErrorExit;
	}

	/* Get Config descriptor */
	Status = UsbIo->UsbGetConfigDescriptor(UsbIo,
		&UsbCcidDevice->ConfigDescriptor);
	if (EFI_ERROR (Status))
	{
		Log2(PCSC_LOG_DEBUG, "UsbGetConfigDescriptor %d", Status);
		goto ErrorExit;
	}

	//
	// Get interface & endpoint descriptor
	//
	Status = UsbIo->UsbGetInterfaceDescriptor(UsbIo,
		&UsbCcidDevice->InterfaceDescriptor);
	if (EFI_ERROR (Status))
	{
		Log2(PCSC_LOG_DEBUG, "UsbGetInterfaceDescriptor %d", Status);
		goto ErrorExit;
	}

	EndpointNumber = UsbCcidDevice->InterfaceDescriptor.NumEndpoints;

	/* Get Endpoints values*/
	for (index = 0; index < EndpointNumber; index++)
	{
		UsbIo->UsbGetEndpointDescriptor (UsbIo, index, &EndpointDescriptor);

		if ((EndpointDescriptor.Attributes & (BIT0 | BIT1)) == USB_ENDPOINT_INTERRUPT)
			CopyMem(&usbDevice[reader_index].interrupt, &EndpointDescriptor, sizeof(EndpointDescriptor));

		if ((EndpointDescriptor.Attributes & (BIT0 | BIT1)) == USB_ENDPOINT_BULK)
		{
			if (EndpointDescriptor.EndpointAddress & USB_ENDPOINT_DIR_IN)
				CopyMem(&usbDevice[reader_index].bulk_in, &EndpointDescriptor, sizeof(EndpointDescriptor));
			else
				CopyMem(&usbDevice[reader_index].bulk_out, &EndpointDescriptor, sizeof(EndpointDescriptor));
		}
	}

	device_descriptor = AllocateZeroPool(UsbCcidDevice->ConfigDescriptor.TotalLength);
	ASSERT(device_descriptor != NULL);

	/* Get USB CCID Descriptor */
	DevReq.RequestType  = USB_DEV_GET_DESCRIPTOR_REQ_TYPE;
	DevReq.Request      = USB_REQ_GET_DESCRIPTOR;
	DevReq.Value        = (UINT16)((USB_DESC_TYPE_CONFIG << 8) | 0 /* Index */);
	DevReq.Index        = UsbCcidDevice->InterfaceDescriptor.InterfaceNumber;
	DevReq.Length       = UsbCcidDevice->ConfigDescriptor.TotalLength;

	Status = UsbIo->UsbControlTransfer(UsbIo,
		&DevReq,
		EfiUsbDataIn,
		USB_GENERAL_DEVICE_REQUEST_TIMEOUT,
		device_descriptor,
		UsbCcidDevice->ConfigDescriptor.TotalLength,
		&Status_uint
		);
	if (EFI_ERROR (Status)) {
		goto FreeExit;
	}

	/* find the CCID descriptor */
	index = 0;
	while (device_descriptor[index] != 0x36 && index < UsbCcidDevice->ConfigDescriptor.TotalLength)
	{
		/* bug in descriptor? */
		if (0 == device_descriptor[index])
			break;

		index += device_descriptor[index];
	}

	if (device_descriptor[index] != 0x36 && device_descriptor[index+1] != 0x21)
	{
		Print(L"CCID descriptor not found\n");
		goto FreeExit;
	}

	/* move to the first byte of the CCID descriptor */
	ccid_descriptor = device_descriptor + index;

	/* store device information */
	usbDevice[reader_index].real_nb_opened_slots = 1;
	usbDevice[reader_index].nb_opened_slots = &usbDevice[reader_index].real_nb_opened_slots;

	/* CCID common informations */
	usbDevice[reader_index].ccid.real_bSeq = 0;
	usbDevice[reader_index].ccid.pbSeq = &usbDevice[reader_index].ccid.real_bSeq;
	usbDevice[reader_index].ccid.readerID =
		(UsbCcidDevice->DeviceDescriptor.IdVendor << 16)
		+ UsbCcidDevice->DeviceDescriptor.IdProduct;
	usbDevice[reader_index].ccid.dwFeatures = dw2i(ccid_descriptor, 40);
	usbDevice[reader_index].ccid.wLcdLayout =
		(ccid_descriptor[51] << 8) + ccid_descriptor[50];
	usbDevice[reader_index].ccid.bPINSupport = ccid_descriptor[52];
	usbDevice[reader_index].ccid.dwMaxCCIDMessageLength = dw2i(ccid_descriptor, 44);
	usbDevice[reader_index].ccid.dwMaxIFSD = dw2i(ccid_descriptor, 28);
	usbDevice[reader_index].ccid.dwDefaultClock = dw2i(ccid_descriptor, 10);
	usbDevice[reader_index].ccid.dwMaxDataRate = dw2i(ccid_descriptor, 23);
	usbDevice[reader_index].ccid.bMaxSlotIndex = ccid_descriptor[4];
	usbDevice[reader_index].ccid.bCurrentSlotIndex = 0;
	usbDevice[reader_index].ccid.readTimeout = DEFAULT_COM_READ_TIMEOUT;
#if 0
	if (ccid_descriptor[27])
		usbDevice[reader_index].ccid.arrayOfSupportedDataRates = get_data_rates(reader_index, config_desc, num);
	else
	{
		usbDevice[reader_index].ccid.arrayOfSupportedDataRates = NULL;
		DEBUG_INFO1("bNumDataRatesSupported is 0");
	}
#endif
	usbDevice[reader_index].ccid.bInterfaceProtocol = UsbCcidDevice->InterfaceDescriptor.InterfaceProtocol;
	usbDevice[reader_index].ccid.bNumEndpoints = UsbCcidDevice->InterfaceDescriptor.NumEndpoints;
	usbDevice[reader_index].ccid.dwSlotStatus = IFD_ICC_PRESENT;
	usbDevice[reader_index].ccid.bVoltageSupport = ccid_descriptor[5];
	usbDevice[reader_index].ccid.sIFD_serial_number = NULL;
	usbDevice[reader_index].ccid.gemalto_firmware_features = NULL;
	usbDevice[reader_index].ccid.zlp = FALSE;
	usbDevice[reader_index].ccid.sIFD_iManufacturer = NULL;
	usbDevice[reader_index].ccid.IFD_bcdDevice = UsbCcidDevice->DeviceDescriptor.BcdDevice;

	/* no error */
	return_value = STATUS_SUCCESS;

FreeExit:
	FreePool(device_descriptor);

ErrorExit:
	return return_value;
} /* OpenUEFI */


/*****************************************************************************
 *
 *					OpenUEFIByName
 *
 ****************************************************************************/
status_t OpenUEFIByName(unsigned int reader_index, /*@null@*/ char *device)
{
	return STATUS_UNSUCCESSFUL;
} /* OpenUEFIByName */


/*****************************************************************************
 *
 *					WriteUEFI
 *
 ****************************************************************************/
status_t WriteUEFI(unsigned int reader_index, unsigned int length,
	unsigned char *buffer)
{
	UINT32 TransStatus;
	EFI_STATUS Status;
	EFI_USB_IO_PROTOCOL *UsbIo = usbDevice[reader_index].UsbIo;
	UINTN DataLength;

	DEBUG_XXD(">", buffer, length);

	DataLength = length;
	Status = UsbIo->UsbBulkTransfer(UsbIo,
		usbDevice[reader_index].bulk_out.EndpointAddress, buffer, &DataLength,
		USB_WRITE_TIMEOUT, &TransStatus);

	if (EFI_ERROR (Status))
	{
		Print(L"UsbBulkTransfer(write) failed\n");
		return STATUS_COMM_ERROR;
	}

	return STATUS_SUCCESS;
} /* WriteUEFI */


/*****************************************************************************
 *
 *					ReadUEFI
 *
 ****************************************************************************/
status_t ReadUEFI(unsigned int reader_index, unsigned int * length,
	unsigned char *buffer)
{
	_ccid_descriptor *ccid_descriptor = get_ccid_descriptor(reader_index);
	int duplicate_frame = 0;
	UINT32 TransStatus;
	EFI_STATUS Status;
	EFI_USB_IO_PROTOCOL *UsbIo = usbDevice[reader_index].UsbIo;
	UINTN DataLength;

read_again:
	DataLength = *length;
	Status = UsbIo->UsbBulkTransfer(UsbIo,
		usbDevice[reader_index].bulk_in.EndpointAddress, buffer, &DataLength,
		usbDevice[reader_index].ccid.readTimeout, &TransStatus);

	if (EFI_ERROR (Status))
	{
		*length = 0;
		Print(L"UsbBulkTransfer(read) failed\n");

		return STATUS_UNSUCCESSFUL;
	}

	*length = DataLength;

	DEBUG_XXD("<", buffer, *length);

#define BSEQ_OFFSET 6
	if ((*length >= BSEQ_OFFSET)
		&& (buffer[BSEQ_OFFSET] < *ccid_descriptor->pbSeq -1))
	{
		duplicate_frame++;
		if (duplicate_frame > 10)
		{
			DEBUG_CRITICAL("Too many duplicate frame detected");
			return STATUS_UNSUCCESSFUL;
		}
		DEBUG_INFO1("Duplicate frame detected");
		goto read_again;
	}

	return STATUS_SUCCESS;
} /* ReadUEFI */


/*****************************************************************************
 *
 *                  ControlUSB
 *
 ****************************************************************************/
int ControlUSB(int reader_index, int requesttype, int request, int value,
    unsigned char *bytes, unsigned int size)
{
	Print(L"ControlUEFI not implemented\n");
	return STATUS_COMM_ERROR;
} /* ControlUSB */


/*****************************************************************************
 *
 *					CloseUEFI
 *
 ****************************************************************************/
status_t CloseUEFI(unsigned int reader_index)
{
	/* device not opened */
	if (usbDevice[reader_index].dev_handle == NULL)
		return STATUS_UNSUCCESSFUL;

	Log0(PCSC_LOG_DEBUG);
	DEBUG_COMM3("Closing USB device: %d/%d",
		usbDevice[reader_index].bus_number,
		usbDevice[reader_index].device_address);

	/* one slot closed */
	(*usbDevice[reader_index].nb_opened_slots)--;

	/* release the allocated ressources for the last slot only */
	if (0 == *usbDevice[reader_index].nb_opened_slots)
	{
		DEBUG_COMM("Last slot closed. Release resources");

		if (usbDevice[reader_index].ccid.gemalto_firmware_features)
			free(usbDevice[reader_index].ccid.gemalto_firmware_features);

		if (usbDevice[reader_index].ccid.sIFD_serial_number)
			free(usbDevice[reader_index].ccid.sIFD_serial_number);

		if (usbDevice[reader_index].ccid.sIFD_iManufacturer)
			free(usbDevice[reader_index].ccid.sIFD_iManufacturer);

		if (usbDevice[reader_index].ccid.arrayOfSupportedDataRates)
			free(usbDevice[reader_index].ccid.arrayOfSupportedDataRates);
	}

	/* mark the resource unused */
	usbDevice[reader_index].dev_handle = NULL;

	return STATUS_SUCCESS;
} /* CloseUEFI */


/*****************************************************************************
 *
 *					get_ccid_descriptor
 *
 ****************************************************************************/
_ccid_descriptor *get_ccid_descriptor(unsigned int reader_index)
{
	return &usbDevice[reader_index].ccid;
} /* get_ccid_descriptor */


/*****************************************************************************
 *
 *					get_usb_device
 *
 ****************************************************************************/
void duplicate_usb_device(unsigned int reader_index,
	unsigned int new_reader_index)
{
	usbDevice[new_reader_index] = usbDevice[reader_index];
} /* get_usb_device */

/*****************************************************************************
 *
 *					InterruptRead
 *
 ****************************************************************************/
int InterruptRead(int reader_index, int timeout /* in ms */)
{
	int return_value = IFD_SUCCESS;
	return return_value;
} /* InterruptRead */


/*****************************************************************************
 *
 *					Stop the async loop
 *
 ****************************************************************************/
void InterruptStop(int reader_index)
{
} /* InterruptStop */


