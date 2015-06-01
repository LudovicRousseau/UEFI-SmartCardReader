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
  TODO: Brief Description of UEFI Driver SmartCardReader

  TODO: Detailed Description of UEFI Driver SmartCardReader

**/

#ifndef __EFI_CCID_DRIVER_H__
#define __EFI_CCID_DRIVER_H__

#include <Uefi.h>

//
// Libraries
//
#include <Library/UefiBootServicesTableLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/BaseLib.h>
#include <Library/UefiLib.h>
#include <Library/DevicePathLib.h>
#include <Library/DebugLib.h>
#include <Library/ReportStatusCodeLib.h>


//
// UEFI Driver Model Protocols
//
#include <Protocol/DriverBinding.h>
#include <Protocol/ComponentName2.h>
#include <Protocol/ComponentName.h>
#include <Protocol/SmartCardReader.h>

//
// Consumed Protocols
//
#include <Protocol/UsbIo.h>


//
// Produced Protocols
//


//
// Guids
//

//
// Driver Version
//
#define CCID_DRIVER_VERSION  0x00000001

#ifndef MAX_ATR_SIZE
#define MAX_ATR_SIZE 33 /* from pcsclite.h */
#endif

//
// Protocol instances
//
extern EFI_DRIVER_BINDING_PROTOCOL  gSmartCardReaderDriverBinding;
extern EFI_COMPONENT_NAME2_PROTOCOL  gSmartCardReaderComponentName2;
extern EFI_COMPONENT_NAME_PROTOCOL  gSmartCardReaderComponentName;


//
// Include files with function prototypes
//
#include "DriverBinding.h"
#include "ComponentName.h"

#define CLASS_CCID               11

#define USB_CCID_DEV_SIGNATURE SIGNATURE_32 ('c', 'c', 'i', 'd')


///
/// Device instance of USB CCID device.
///
typedef struct _USB_CCID_DEV {
  UINTN                         Signature;
  EFI_DEVICE_PATH_PROTOCOL      *DevicePath;
  EFI_EVENT                     DelayedRecoveryEvent;
  EFI_USB_IO_PROTOCOL           *UsbIo;
  EFI_HANDLE                    *ControllerHandle;
  EFI_HANDLE                    *DriverBindingHandle;
  EFI_SMART_CARD_READER_PROTOCOL SmartCardReaderProtocol;
  EFI_USB_DEVICE_DESCRIPTOR     DeviceDescriptor;
  EFI_USB_CONFIG_DESCRIPTOR     ConfigDescriptor;
  EFI_USB_INTERFACE_DESCRIPTOR  InterfaceDescriptor;
  EFI_UNICODE_STRING_TABLE      *ControllerNameTable;
  INTN                          SlotNumber;
  struct _USB_CCID_DEV          *NextSlot;
  UINTN                         Lun;
  CHAR16                        ReaderName[128];
  INTN                          ReaderNameLength;
  unsigned char                 Atr[MAX_ATR_SIZE];
  INTN                          AtrLength;
  INTN                          CardProtocol;
  INTN                          State;
  INTN                          InUse;
} USB_CCID_DEV;

#define USB_CCID_DEV_FROM_SMART_CARD_READER_PROTOCOL(a) \
    CR(a, USB_CCID_DEV, SmartCardReaderProtocol, USB_CCID_DEV_SIGNATURE)


//
// Internal worker functions
//

/**
  Uses USB I/O to check whether the device is a USB CCID device.

  @param  UsbIo    Pointer to a USB I/O protocol instance.

  @retval TRUE     Device is a USB CCID device.
  @retval FALSE    Device is a not USB CCID device.

**/
BOOLEAN
IsUsbCcid (
  IN  EFI_USB_IO_PROTOCOL     *UsbIo
  );

#endif
