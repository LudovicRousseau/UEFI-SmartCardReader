/*
Copyright (c) 2014, Gemalto. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in
  the documentation and/or other materials provided with the
  distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

/** @file
  Entry points for the UEFI CCID driver

  When a USB CCID device is inserted or removed then Start or Stop
  functions from this file are called.

**/

#include <Library/PrintLib.h>
#include <Protocol/SmartCardReader.h>

#include "CcidDriver.h"
#include "SmartCardReader_impl.h"

#include "ifdhandler.h"
#include "debuglog.h"
#include "defs.h"
#include "utils.h"
#include "ccid_ifdhandler.h"
#include "ccid.h"
#include "ccid_uefi.h"

static int Lun = 0;

///
/// Driver Binding Protocol instance
///
EFI_DRIVER_BINDING_PROTOCOL gSmartCardReaderDriverBinding = {
  SmartCardReaderDriverBindingSupported,
  SmartCardReaderDriverBindingStart,
  SmartCardReaderDriverBindingStop,
  CCID_DRIVER_VERSION,
  NULL,
  NULL
};


/**
  Unloads an image.

  @param  ImageHandle           Handle that identifies the image to be unloaded.

  @retval EFI_SUCCESS           The image has been unloaded.
  @retval EFI_INVALID_PARAMETER ImageHandle is not a valid image handle.

**/
EFI_STATUS
EFIAPI
SmartCardReaderUnload (
  IN EFI_HANDLE  ImageHandle
  )
{
  EFI_STATUS  Status;

  EFI_HANDLE  *HandleBuffer;
  UINTN       HandleCount;
  UINTN       Index;

  Log0(PCSC_LOG_DEBUG);

  Status = EFI_SUCCESS;

  //
  // Retrieve array of all handles in the handle database
  //
  Status = gBS->LocateHandleBuffer (
    AllHandles,
    NULL,
    NULL,
    &HandleCount,
    &HandleBuffer
    );
  if (EFI_ERROR (Status)) {
    Log2(PCSC_LOG_DEBUG, "LocateHandleBuffer %d", Status);
    return Status;
  }

  //
  // Disconnect the current driver from handles in the handle database
  //
  for (Index = 0; Index < HandleCount; Index++) {
    Status = gBS->DisconnectController (HandleBuffer[Index], gImageHandle, NULL);
  }

  //
  // Free the array of handles
  //
  FreePool (HandleBuffer);

  //
  // Uninstall protocols installed in the driver entry point
  //
  Status = gBS->UninstallMultipleProtocolInterfaces (
    ImageHandle,
    &gEfiDriverBindingProtocolGuid,  &gSmartCardReaderDriverBinding,
    &gEfiComponentNameProtocolGuid,  &gSmartCardReaderComponentName,
    &gEfiComponentName2ProtocolGuid, &gSmartCardReaderComponentName2,
    NULL
    );
  if (EFI_ERROR (Status)) {
    Log2(PCSC_LOG_DEBUG, "UninstallMultipleProtocolInterfaces %d", Status);
    return Status;
  }


  //
  // Do any additional cleanup that is required for this driver
  //

  return EFI_SUCCESS;
}

/**
  This is the declaration of an EFI image entry point. This entry point is
  the same for UEFI Applications, UEFI OS Loaders, and UEFI Drivers including
  both device drivers and bus drivers.

  @param  ImageHandle           The firmware allocated handle for the UEFI image.
  @param  SystemTable           A pointer to the EFI System Table.

  @retval EFI_SUCCESS           The operation completed successfully.
  @retval Others                An unexpected error occurred.
**/
EFI_STATUS
EFIAPI
SmartCardReaderDriverEntryPoint (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS  Status;

  Status = EFI_SUCCESS;

  // Install UEFI Driver Model protocol(s).
  //
  Status = EfiLibInstallAllDriverProtocols2 (
    ImageHandle,
    SystemTable,
    &gSmartCardReaderDriverBinding,
    ImageHandle,
    &gSmartCardReaderComponentName,
    &gSmartCardReaderComponentName2,
    NULL,
    NULL,
    NULL,
    NULL
    );
  ASSERT_EFI_ERROR (Status);

  return Status;
}


/**
  Tests to see if this driver supports a given controller. If a child
  device is provided, it further tests to see if this driver supports
  creating a handle for the specified child device.

  This function checks to see if the driver specified by This supports
  the device specified by ControllerHandle. Drivers will typically use
  the device path attached to ControllerHandle and/or the services from
  the bus I/O abstraction attached to ControllerHandle to determine if
  the driver supports ControllerHandle. This function may be called many
  times during platform initialization. In order to reduce boot times,
  the tests performed by this function must be very small, and take as
  little time as possible to execute. This function must not change the
  state of any hardware devices, and this function must be aware that
  the device specified by ControllerHandle may already be managed by the
  same driver or a different driver. This function must match its calls
  to AllocatePages() with FreePages(), AllocatePool() with FreePool(),
  and OpenProtocol() with CloseProtocol().  Because ControllerHandle may
  have been previously started by the same driver, if a protocol is
  already in the opened state, then it must not be closed with
  CloseProtocol(). This is required to guarantee the state of
  ControllerHandle is not modified by this function.

  @param[in]  This                 A pointer to the
                                   EFI_DRIVER_BINDING_PROTOCOL instance.
  @param[in]  ControllerHandle     The handle of the controller to test.
                                   This handle must support a protocol
                                   interface that supplies an I/O
                                   abstraction to the driver.
  @param[in]  RemainingDevicePath  A pointer to the remaining portion of
                                   a device path.  This parameter is
                                   ignored by device drivers, and is
                                   optional for bus drivers. For bus
                                   drivers, if this parameter is not
                                   NULL, then the bus driver must
                                   determine if the bus controller
                                   specified by ControllerHandle and the
                                   child controller specified by
                                   RemainingDevicePath are both
                                   supported by this bus driver.

  @retval EFI_SUCCESS              The device specified by
                                   ControllerHandle and
                                   RemainingDevicePath is supported by
                                   the driver specified by This.
  @retval EFI_ALREADY_STARTED      The device specified by
                                   ControllerHandle and
                                   RemainingDevicePath is already being
                                   managed by the driver specified by
                                   This.
  @retval EFI_ACCESS_DENIED        The device specified by
                                   ControllerHandle and
                                   RemainingDevicePath is already being
                                   managed by a different driver or an
                                   application that requires exclusive
                                   access.  Currently not implemented.
  @retval EFI_UNSUPPORTED          The device specified by
                                   ControllerHandle and
                                   RemainingDevicePath is not supported
                                   by the driver specified by This.
**/
EFI_STATUS
EFIAPI
SmartCardReaderDriverBindingSupported (
  IN EFI_DRIVER_BINDING_PROTOCOL  *This,
  IN EFI_HANDLE                   ControllerHandle,
  IN EFI_DEVICE_PATH_PROTOCOL     *RemainingDevicePath OPTIONAL
  )
{
  EFI_STATUS          Status;
  EFI_USB_IO_PROTOCOL *UsbIo;

  Status = gBS->OpenProtocol (
    ControllerHandle,
    &gEfiUsbIoProtocolGuid,
    (VOID **) &UsbIo,
    This->DriverBindingHandle,
    ControllerHandle,
    EFI_OPEN_PROTOCOL_BY_DRIVER
    );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  //
  // Use the USB I/O Protocol interface to check whether Controller is
  // a CCID device that can be managed by this driver.
  //
  Status = EFI_UNSUPPORTED;
  if (IsUsbCcid (UsbIo)) {
    Status = EFI_SUCCESS;
  }

  gBS->CloseProtocol (
    ControllerHandle,
    &gEfiUsbIoProtocolGuid,
    This->DriverBindingHandle,
    ControllerHandle
    );

  return Status;
}

/**
  Initialize the USB CCID device.

  @param  UsbCcidDev            Device instance to be initialized.

  @retval EFI_SUCCESS           USB CCID device successfully initialized..
  @retval Other                 USB CCID device was not initialized
                                successfully.

**/
EFI_STATUS
FinalizeUsbCcidDevice (
  IN OUT USB_CCID_DEV *UsbCcidDev
  )
{
  EFI_STATUS status = EFI_SUCCESS;
  Log0(PCSC_LOG_DEBUG);

  if (IFD_SUCCESS != IFDHCloseChannel(UsbCcidDev->Lun)) {
    status = EFI_UNSUPPORTED;
  }

  return status;
}

/**
  Initialize the USB CCID device.

  @param  UsbCcidDev            Device instance to be initialized.

  @retval EFI_SUCCESS           USB CCID device successfully initialized..
  @retval Other                 USB CCID device was not initialized
                                successfully.

**/
EFI_STATUS
InitializeUsbCcidDevice (
  IN OUT USB_CCID_DEV *UsbCcidDevice
  )
{
  EFI_USB_IO_PROTOCOL *UsbIo;
  EFI_STATUS Status;
  CHAR16 *ManufacturerString = NULL;
  CHAR16 *ProductString = NULL;
  UINT16 *LangIDTable;
  UINT16 TableSize;
  INTN Index;
  CHAR16 TxtLun[10];
  INTN reader_index;
  CcidDesc *ccid_slot;

  Log0(PCSC_LOG_DEBUG);

  if (IFD_SUCCESS != IFDHCreateChannel(Lun, (DWORD)UsbCcidDevice)) {
    return EFI_UNSUPPORTED;
  }

  UsbCcidDevice->Lun = Lun;
  UsbCcidDevice->State = SCARD_UNKNOWN;
  UsbCcidDevice->AtrLength = 0;
  UsbCcidDevice->InUse = 0;
  UsbCcidDevice->CardProtocol = -1;

  /* Lun are not recycled */
  Lun++;

  UsbIo = UsbCcidDevice->UsbIo;

  //
  // Get all supported languages.
  //
  TableSize = 0;
  LangIDTable = NULL;
  Status = UsbIo->UsbGetSupportedLanguages(UsbIo, &LangIDTable, &TableSize);
  if (EFI_ERROR(Status)) {
    Log1(PCSC_LOG_ERROR, "UsbGetSupportedLanguages");
    goto error;
  }

  /* Get Manufacturer string */
  for (Index = 0; Index < TableSize / sizeof (LangIDTable[0]); Index++) {
    ManufacturerString = NULL;
    Status = UsbIo->UsbGetStringDescriptor(UsbIo,
        LangIDTable[Index],
        UsbCcidDevice->DeviceDescriptor.StrManufacturer,
        &ManufacturerString);

    if (EFI_ERROR (Status) || (ManufacturerString == NULL)) {
      continue;
    }

    StrCpy(UsbCcidDevice->ReaderName, ManufacturerString);
    StrCat(UsbCcidDevice->ReaderName, L" ");

    FreePool(ManufacturerString);
    break;
  }

  /* Get Product string */
  for (Index = 0; Index < TableSize / sizeof (LangIDTable[0]); Index++) {
    ProductString = NULL;
    Status = UsbIo->UsbGetStringDescriptor(UsbIo,
        LangIDTable[Index],
        UsbCcidDevice->DeviceDescriptor.StrProduct,
        &ProductString);

    if (EFI_ERROR (Status) || (ProductString == NULL)) {
      continue;
    }

    StrCat(UsbCcidDevice->ReaderName, ProductString);

    FreePool(ProductString);
    break;
  }

error:
  /* Can't get the strings from the device */
  if ((NULL == ManufacturerString) || (NULL == ProductString)) {
    UnicodeSPrint(UsbCcidDevice->ReaderName,
        sizeof(UsbCcidDevice->ReaderName)/sizeof(UsbCcidDevice->ReaderName[0]),
        L"%04X:%04X",
        UsbCcidDevice->DeviceDescriptor.IdVendor,
        UsbCcidDevice->DeviceDescriptor.IdProduct);
  }

  /* Add the Lun */
  UnicodeSPrint(TxtLun, sizeof(TxtLun)/sizeof(TxtLun[0]), L" %d", UsbCcidDevice->Lun);
  StrCat(UsbCcidDevice->ReaderName, TxtLun);

  /* Length (in bytes) including Null terminator */
  UsbCcidDevice->ReaderNameLength = StrSize(UsbCcidDevice->ReaderName);

  /* set the reader name to the lower level */
  reader_index = LunToReaderIndex(UsbCcidDevice->Lun);
  ccid_slot = get_ccid_slot(reader_index);
  ccid_slot->readerName = UsbCcidDevice->ReaderName;

  return EFI_SUCCESS;
}

/**
  Starts a device controller or a bus controller.

  The Start() function is designed to be invoked from the EFI boot
  service ConnectController().

  As a result, much of the error checking on the parameters to Start()
  has been moved into this common boot service. It is legal to call
  Start() from other locations, but the following calling restrictions
  must be followed, or the system behavior will not be deterministic.

  1. ControllerHandle must be a valid EFI_HANDLE.
  2. If RemainingDevicePath is not NULL, then it must be a pointer to a
     naturally aligned EFI_DEVICE_PATH_PROTOCOL.
  3. Prior to calling Start(), the Supported() function for the driver
     specified by This must have been called with the same calling
     parameters, and Supported() must have returned EFI_SUCCESS.

  @param[in]  This                 A pointer to the
                                   EFI_DRIVER_BINDING_PROTOCOL instance.
  @param[in]  ControllerHandle     The handle of the controller to
                                   start. This handle must support a
                                   protocol interface that supplies an
                                   I/O abstraction to the driver.
  @param[in]  RemainingDevicePath  A pointer to the remaining portion of
                                   a device path.  This parameter is
                                   ignored by device drivers, and is
                                   optional for bus drivers. For a bus
                                   driver, if this parameter is NULL,
                                   then handles for all the children of
                                   Controller are created by this
                                   driver.  If this parameter is not
                                   NULL and the first Device Path Node
                                   is not the End of Device Path Node,
                                   then only the handle for the child
                                   device specified by the first Device
                                   Path Node of RemainingDevicePath is
                                   created by this driver.  If the first
                                   Device Path Node of
                                   RemainingDevicePath is the End of
                                   Device Path Node, no child handle is
                                   created by this driver.

  @retval EFI_SUCCESS              The device was started.
  @retval EFI_DEVICE_ERROR         The device could not be started due to
                                   a device error.Currently not implemented.
  @retval EFI_OUT_OF_RESOURCES     The request could not be completed due to
                                   a lack of resources.
  @retval Others                   The driver failded to start the device.

**/
EFI_STATUS
EFIAPI
SmartCardReaderDriverBindingStart (
  IN EFI_DRIVER_BINDING_PROTOCOL  *This,
  IN EFI_HANDLE                   ControllerHandle,
  IN EFI_DEVICE_PATH_PROTOCOL     *RemainingDevicePath OPTIONAL
  )
{
  EFI_STATUS           Status;
  EFI_USB_IO_PROTOCOL  *UsbIo;
  USB_CCID_DEV         *UsbCcidDevice;
  EFI_TPL              OldTpl;
  EFI_SMART_CARD_READER_PROTOCOL SmartCardReaderProtocol = {
    .SCardConnect = SCardConnect,
    .SCardDisconnect = SCardDisconnect,
    .SCardStatus = SCardStatus,
    .SCardTransmit =SCardTransmit,
    .SCardControl = SCardControl,
    .SCardGetAttrib = SCardGetAttrib
  };
  INTN slot, reader_index;
  CcidDesc *ccid_slot;
  _ccid_descriptor *ccid_descriptor;
  USB_CCID_DEV *previous_UsbCcidDevice;

  Log0(PCSC_LOG_DEBUG);

  OldTpl = gBS->RaiseTPL (TPL_CALLBACK);

  //
  // Open USB I/O Protocol
  //
  Status = gBS->OpenProtocol (
    ControllerHandle,
    &gEfiUsbIoProtocolGuid,
    (VOID **) &UsbIo,
    This->DriverBindingHandle,
    ControllerHandle,
    EFI_OPEN_PROTOCOL_BY_DRIVER
    );
  if (EFI_ERROR (Status)) {
    Log2(PCSC_LOG_DEBUG, "OpenProtocol %d", Status);
    goto ErrorExit1;
  }

  UsbCcidDevice = AllocateZeroPool (sizeof (USB_CCID_DEV));
  ASSERT (UsbCcidDevice != NULL);

  UsbCcidDevice->UsbIo     = UsbIo;
  UsbCcidDevice->Signature = USB_CCID_DEV_SIGNATURE;
  UsbCcidDevice->ControllerHandle = ControllerHandle;
  UsbCcidDevice->DriverBindingHandle = This->DriverBindingHandle;

  Status = InitializeUsbCcidDevice(UsbCcidDevice);
  if (EFI_ERROR (Status)) {
    goto ErrorExit;
  }

  /* set the protocol functions */
  UsbCcidDevice->SmartCardReaderProtocol = SmartCardReaderProtocol;

  Status = gBS->InstallProtocolInterface (
    &ControllerHandle,
    &gEfiSmartCardReaderProtocolGuid,
    EFI_NATIVE_INTERFACE,
    &UsbCcidDevice->SmartCardReaderProtocol
    );

  if (EFI_ERROR (Status)) {
    Log2(PCSC_LOG_DEBUG, "InstallProtocolInterface %d", Status);
    goto ErrorExit;
  }

  UsbCcidDevice->ControllerNameTable = NULL;
  AddUnicodeString2 (
    "eng",
    gSmartCardReaderComponentName.SupportedLanguages,
    &UsbCcidDevice->ControllerNameTable,
    L"Generic CCID Reader",
    TRUE
    );
  AddUnicodeString2 (
    "en",
    gSmartCardReaderComponentName2.SupportedLanguages,
    &UsbCcidDevice->ControllerNameTable,
    L"Generic CCID Reader",
    FALSE
    );

  /* multi slot readers */
  reader_index = LunToReaderIndex(UsbCcidDevice->Lun);
  ccid_slot = get_ccid_slot(reader_index);
  ccid_descriptor = get_ccid_descriptor(reader_index);
  previous_UsbCcidDevice = UsbCcidDevice;

  for (slot=1; slot <= ccid_descriptor->bMaxSlotIndex; slot++) {
    USB_CCID_DEV *new_UsbCcidDevice;
    EFI_HANDLE new_ControllerHandle = NULL;
    int new_reader_index;
    CHAR16 TxtSlot[10];

    Log2(PCSC_LOG_DEBUG, "slot: %d", slot);

    if (-1 == (new_reader_index = GetNewReaderIndex(Lun))) {
      return EFI_DEVICE_ERROR;
    }

    new_UsbCcidDevice = AllocateZeroPool (sizeof (USB_CCID_DEV));
    ASSERT (new_UsbCcidDevice != NULL);

    /* Copy the USB device */
    duplicate_usb_device(reader_index, new_reader_index);

    /* copy the UEFI device */
    *new_UsbCcidDevice = *UsbCcidDevice;
    new_UsbCcidDevice->Lun = Lun;
    new_UsbCcidDevice->SlotNumber = slot;

    /* Add the slot number */
    UnicodeSPrint(TxtSlot, sizeof(TxtSlot)/sizeof(TxtSlot[0]), L", %d", slot);
    StrCat(new_UsbCcidDevice->ReaderName, TxtSlot);

    /* Set the reader name to the lower level */
    new_reader_index = LunToReaderIndex(new_UsbCcidDevice->Lun);
    get_ccid_slot(new_reader_index)->readerName = new_UsbCcidDevice->ReaderName;

    /* Set the slot number ta the lower level */
    get_ccid_descriptor(new_reader_index)->bCurrentSlotIndex = slot;

    /* New Lun for a new slot */
    Lun++;

    /* Chaining */
    previous_UsbCcidDevice->NextSlot = new_UsbCcidDevice;
    previous_UsbCcidDevice = new_UsbCcidDevice;

    /* Create a new UEFI SmartCardReaderProtocol object */
    Status = gBS->InstallProtocolInterface (
      &new_ControllerHandle,
      &gEfiSmartCardReaderProtocolGuid,
      EFI_NATIVE_INTERFACE,
      &new_UsbCcidDevice->SmartCardReaderProtocol
      );

    if (EFI_ERROR (Status)) {
      Log2(PCSC_LOG_DEBUG, "InstallProtocolInterface %d", Status);
      goto ErrorExit;
    }

    new_UsbCcidDevice->ControllerHandle = new_ControllerHandle;
  }

  gBS->RestoreTPL (OldTpl);

  return EFI_SUCCESS;

  //
  // Error handler
  //
ErrorExit:
  if (EFI_ERROR (Status)) {
    gBS->CloseProtocol (
      ControllerHandle,
      &gEfiUsbIoProtocolGuid,
      This->DriverBindingHandle,
      ControllerHandle
      );

    FreePool (UsbCcidDevice);
    UsbCcidDevice = NULL;
  }

ErrorExit1:
  gBS->RestoreTPL (OldTpl);
  return Status;
}

/**
  Stops a device controller or a bus controller.

  The Stop() function is designed to be invoked from the EFI boot
  service DisconnectController().

  As a result, much of the error checking on the parameters to Stop()
  has been moved into this common boot service. It is legal to call
  Stop() from other locations, but the following calling restrictions
  must be followed, or the system behavior will not be deterministic.

  1. ControllerHandle must be a valid EFI_HANDLE that was used on a
     previous call to this same driver's Start() function.

  2. The first NumberOfChildren handles of ChildHandleBuffer must all be
     a valid EFI_HANDLE. In addition, all of these handles must have
     been created in this driver's Start() function, and the Start()
     function must have called OpenProtocol() on ControllerHandle with
     an Attribute of EFI_OPEN_PROTOCOL_BY_CHILD_CONTROLLER.

  @param[in]  This              A pointer to the
                                EFI_DRIVER_BINDING_PROTOCOL instance.
  @param[in]  ControllerHandle  A handle to the device being stopped.
                                The handle must support a bus specific
                                I/O protocol for the driver to use to
                                stop the device.
  @param[in]  NumberOfChildren  The number of child device handles in
                                ChildHandleBuffer.
  @param[in]  ChildHandleBuffer An array of child handles to be freed.
                                May be NULL if NumberOfChildren is 0.

  @retval EFI_SUCCESS           The device was stopped.
  @retval EFI_DEVICE_ERROR      The device could not be stopped due to a
                                device error.

**/
EFI_STATUS
EFIAPI
SmartCardReaderDriverBindingStop (
  IN EFI_DRIVER_BINDING_PROTOCOL  *This,
  IN EFI_HANDLE                   ControllerHandle,
  IN UINTN                        NumberOfChildren,
  IN EFI_HANDLE                   *ChildHandleBuffer OPTIONAL
  )
{
  EFI_STATUS                     Status;
  USB_CCID_DEV                   *UsbCcidDevice;
  EFI_SMART_CARD_READER_PROTOCOL *SmartCardReaderProtocol;

  Log0(PCSC_LOG_DEBUG);

  Status = gBS->OpenProtocol (
    ControllerHandle,
    &gEfiSmartCardReaderProtocolGuid,
    (VOID **) &SmartCardReaderProtocol,
    This->DriverBindingHandle,
    ControllerHandle,
    EFI_OPEN_PROTOCOL_GET_PROTOCOL
    );

  if (EFI_ERROR (Status)) {
    return EFI_UNSUPPORTED;
  }

  UsbCcidDevice = USB_CCID_DEV_FROM_SMART_CARD_READER_PROTOCOL (SmartCardReaderProtocol);

  gBS->CloseProtocol (
    ControllerHandle,
    &gEfiUsbIoProtocolGuid,
    This->DriverBindingHandle,
    ControllerHandle
    );

  //
  // Free all resources.
  //
  if (UsbCcidDevice->ControllerNameTable != NULL) {
    FreeUnicodeStringTable (UsbCcidDevice->ControllerNameTable);
  }

  /* For each slot */
  while (UsbCcidDevice) {
    USB_CCID_DEV *NextSlot;

    Log2(PCSC_LOG_DEBUG, "Closing slot: %d", UsbCcidDevice->SlotNumber);

    Status = FinalizeUsbCcidDevice(UsbCcidDevice);
    if (EFI_ERROR (Status)) {
      return Status;
    }

    /* Remove the protocol Interface */
    Status = gBS->UninstallProtocolInterface (
      UsbCcidDevice->ControllerHandle,
      &gEfiSmartCardReaderProtocolGuid,
      &UsbCcidDevice->SmartCardReaderProtocol
      );
    if (EFI_ERROR (Status)) {
      return Status;
    }

    NextSlot = UsbCcidDevice->NextSlot;

    /* Free allocated memory */
    FreePool (UsbCcidDevice);

    UsbCcidDevice = NextSlot;
  }

  return EFI_SUCCESS;
}

/**
  Uses USB I/O to check whether the device is a USB CCID device.

  @param  UsbIo    Pointer to a USB I/O protocol instance.

  @retval TRUE     Device is a USB CCID device.
  @retval FALSE    Device is a not USB CCID device.

 **/
BOOLEAN
IsUsbCcid (
  IN  EFI_USB_IO_PROTOCOL     *UsbIo
  )
{
  EFI_STATUS                    Status;
  EFI_USB_INTERFACE_DESCRIPTOR  InterfaceDescriptor;
  EFI_USB_DEVICE_DESCRIPTOR DeviceDescriptor;

  //
  // Get the default interface descriptor
  //
  Status = UsbIo->UsbGetInterfaceDescriptor (
    UsbIo,
    &InterfaceDescriptor
    );

  if (EFI_ERROR (Status)) {
    Log2(PCSC_LOG_DEBUG, "UsbGetInterfaceDescriptor %d", Status);
    return FALSE;
  }

  Status = UsbIo->UsbGetDeviceDescriptor (
    UsbIo,
    &DeviceDescriptor);

  if (EFI_ERROR (Status)) {
    Log2(PCSC_LOG_DEBUG, "UsbGetDeviceDescriptor %d", Status);
    return FALSE;
  }

  Log3(PCSC_LOG_DEBUG, "IdVendor: %04X, IdProduct: %04X",
    DeviceDescriptor.IdVendor, DeviceDescriptor.IdProduct);

  if (InterfaceDescriptor.InterfaceClass == CLASS_CCID) {
    Log1(PCSC_LOG_DEBUG, "Found a CCID device!");

    return TRUE;
  }

  /* Log1(PCSC_LOG_DEBUG, "Not a CCID device"); */
  return FALSE;
}

