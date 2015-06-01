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

#include "CcidDriver.h"
#include "SmartCardReader_impl.h"
#include <Protocol/SmartCardReader.h>

#include "debuglog.h"
#include "ifdhandler.h"
#include "utils.h"
#include "commands.h"
#include "ccid.h"
#include "atr.h"
#include "defs.h"

#define CheckThis if (NULL == This) return EFI_INVALID_PARAMETER

static
void
update_state(
  USB_CCID_DEV *UsbCcidDevice
  )
{
  INTN reader_index;
  RESPONSECODE response;
  unsigned char pcbuffer[SIZE_GET_SLOT_STATUS];

  UsbCcidDevice->State = SCARD_UNKNOWN;

  if (-1 == (reader_index = LunToReaderIndex(UsbCcidDevice->Lun))) {
    return;
  }

  response = CmdGetSlotStatus(reader_index, pcbuffer);

  if (response != IFD_SUCCESS) {
    return;
  }

  switch (pcbuffer[7] & CCID_ICC_STATUS_MASK) { /* bStatus */
    case CCID_ICC_PRESENT_ACTIVE:
      UsbCcidDevice->State = SCARD_ACTIVE;
      break;
    case CCID_ICC_PRESENT_INACTIVE:
      UsbCcidDevice->State = SCARD_INACTIVE;
      UsbCcidDevice->AtrLength = 0;
      break;
    case CCID_ICC_ABSENT:
      UsbCcidDevice->State = SCARD_ABSENT;
      UsbCcidDevice->AtrLength = 0;
      break;
  }
}

EFI_STATUS
EFIAPI
SCardConnect (
  IN     EFI_SMART_CARD_READER_PROTOCOL *This,
  IN     UINT32                         AccessMode,
  IN     UINT32                         CardAction,
  IN     UINT32                         PreferredProtocols,
     OUT UINT32                         *ActiveProtocol
  )
{
  EFI_STATUS Status = EFI_SUCCESS;
  RESPONSECODE response;
  USB_CCID_DEV *UsbCcidDevice;
  int protocol, availableProtocols;
  ATR_t atr;

  Log0(PCSC_LOG_DEBUG);

  /* Check parameters */
  CheckThis;

  /* Invalid card actions */
  if ((SCARD_CA_UNPOWER == CardAction) || (SCARD_CA_EJECT == CardAction)) {
    return EFI_INVALID_PARAMETER;
  }

  UsbCcidDevice = USB_CCID_DEV_FROM_SMART_CARD_READER_PROTOCOL (This);

  /* Already used by another SCardConnect */
  if (UsbCcidDevice->InUse) {
    return EFI_ACCESS_DENIED;
  }

  UsbCcidDevice->InUse = 1;

  /* Access the reader: nothing to do */
  if (SCARD_AM_READER == AccessMode) {
    return EFI_SUCCESS;
  }

  if (AccessMode != SCARD_AM_CARD) {
    return EFI_INVALID_PARAMETER;
  }

  /* Perform the CardAction action */
  Status = SCardDisconnect(This, CardAction);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  /* No ATR so no protocol to negociate */
  *ActiveProtocol = SCARD_PROTOCOL_UNDEFINED;
  if (0 == UsbCcidDevice->AtrLength) {
    return EFI_SUCCESS;
  }

  if (ATR_OK != ATR_InitFromArray(&atr, UsbCcidDevice->Atr, UsbCcidDevice->AtrLength)) {
    return EFI_DEVICE_ERROR;
  }

  if (ATR_OK != ATR_GetDefaultProtocol(&atr, &protocol, &availableProtocols)) {
    return EFI_DEVICE_ERROR;
  }

  if ((PreferredProtocols & SCARD_PROTOCOL_T1)
      && (availableProtocols & 1 << 1)) {
    *ActiveProtocol = SCARD_PROTOCOL_T1;
    protocol = SCARD_PROTOCOL_T1;
  }
  else
    if ((PreferredProtocols & SCARD_PROTOCOL_T0)
        && (availableProtocols & 1 << 0)) {
      *ActiveProtocol = SCARD_PROTOCOL_T0;
      protocol = SCARD_PROTOCOL_T0;
    }
    else {
      return EFI_UNSUPPORTED;
    }

  switch (protocol) {
    case SCARD_PROTOCOL_T0:
      UsbCcidDevice->CardProtocol = T_0;
      break;
    case SCARD_PROTOCOL_T1:
      UsbCcidDevice->CardProtocol = T_1;
      break;
  }
  response = IFDHSetProtocolParameters(UsbCcidDevice->Lun, protocol, 0, 0, 0, 0);
  if (response != IFD_SUCCESS) {
    Status = EFI_UNSUPPORTED;
  }

  return Status;
}

EFI_STATUS
EFIAPI
SCardDisconnect (
  IN EFI_SMART_CARD_READER_PROTOCOL *This,
  IN UINT32                         CardAction
  )
{
  EFI_STATUS Status = EFI_SUCCESS;
  USB_CCID_DEV *UsbCcidDevice;
  RESPONSECODE response;
  DWORD length;

  Log0(PCSC_LOG_DEBUG);

  /* Check parameters */
  CheckThis;

  UsbCcidDevice = USB_CCID_DEV_FROM_SMART_CARD_READER_PROTOCOL (This);

  UsbCcidDevice->InUse = 0;

  switch (CardAction) {
    case SCARD_CA_EJECT:
      Status = EFI_UNSUPPORTED;
      UsbCcidDevice->InUse = 1; /* do not release the reader. See spec */
      break;

    case SCARD_CA_NORESET:
      break;

    case SCARD_CA_COLDRESET:
      /* Power Off */
      response = IFDHPowerICC(UsbCcidDevice->Lun, IFD_POWER_DOWN,
          UsbCcidDevice->Atr, &length);
      if (response != IFD_SUCCESS) {
        Status = EFI_DEVICE_ERROR;
        break;
      }

      /* Power On */
      response = IFDHPowerICC(UsbCcidDevice->Lun, IFD_POWER_UP,
          UsbCcidDevice->Atr, &length);
      if (response != IFD_SUCCESS) {
        Status = EFI_NOT_READY;
      }

      UsbCcidDevice->AtrLength = length;
      break;

    case SCARD_CA_WARMRESET:
      /* Reset */
      response = IFDHPowerICC(UsbCcidDevice->Lun, IFD_RESET,
          UsbCcidDevice->Atr, &length);
      if (response != IFD_SUCCESS) {
        Status = EFI_DEVICE_ERROR;
      }

      UsbCcidDevice->AtrLength = length;
      break;

    case SCARD_CA_UNPOWER:
      /* Power Off */
      response = IFDHPowerICC(UsbCcidDevice->Lun, IFD_POWER_DOWN,
          UsbCcidDevice->Atr, &length);
      if (response != IFD_SUCCESS) {
        Status = EFI_DEVICE_ERROR;
      }

      UsbCcidDevice->AtrLength = 0;
      break;

    default:
      Status = EFI_INVALID_PARAMETER;
  }

  return Status;
}

EFI_STATUS
EFIAPI
SCardStatus (
  IN     EFI_SMART_CARD_READER_PROTOCOL *This,
     OUT CHAR16                         *ReaderName,
  IN OUT UINTN                          *ReaderNameLength,
     OUT UINT32                         *State,
     OUT UINT32                         *CardProtocol,
     OUT UINT8                          *Atr,
  IN OUT UINTN                          *AtrLength
  )
{
  USB_CCID_DEV *UsbCcidDevice;
  UINTN old_AtrLength = 0, old_ReaderNameLength = 0;

  Log0(PCSC_LOG_DEBUG);

  /* Check parameters */
  CheckThis;
  if ((NULL == ReaderNameLength) && (NULL != ReaderName)) {
    return EFI_INVALID_PARAMETER;
  }
  if ((NULL == AtrLength) && (NULL != Atr)) {
    return EFI_INVALID_PARAMETER;
  }

  UsbCcidDevice = USB_CCID_DEV_FROM_SMART_CARD_READER_PROTOCOL (This);

  update_state(UsbCcidDevice);

  if (State) {
    *State = UsbCcidDevice->State;
  }
  if (CardProtocol) {
    *CardProtocol = UsbCcidDevice->CardProtocol;
  }

  if (AtrLength) {
    old_AtrLength = *AtrLength;
    *AtrLength = UsbCcidDevice->AtrLength;
  }
  if (old_AtrLength < UsbCcidDevice->AtrLength) {
    return EFI_BUFFER_TOO_SMALL;
  }

  if (Atr) {
    CopyMem(Atr, UsbCcidDevice->Atr, UsbCcidDevice->AtrLength);
  }

  if (ReaderNameLength) {
    old_ReaderNameLength = *ReaderNameLength;
    *ReaderNameLength = UsbCcidDevice->ReaderNameLength;
  }
  if (old_ReaderNameLength < UsbCcidDevice->ReaderNameLength) {
    return EFI_BUFFER_TOO_SMALL;
  }

  if (ReaderName) {
    CopyMem(ReaderName, UsbCcidDevice->ReaderName, UsbCcidDevice->ReaderNameLength);
  }

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
SCardTransmit (
  IN     EFI_SMART_CARD_READER_PROTOCOL *This,
  IN     UINT8                          *CAPDU,
  IN     UINTN                          CAPDULength,
     OUT UINT8                          *RAPDU,
  IN OUT UINTN                          *RAPDULength
  )
{
  USB_CCID_DEV *UsbCcidDevice;
  RESPONSECODE response;
  SCARD_IO_HEADER SendPci;
  DWORD RxLength;
  EFI_STATUS Status = EFI_SUCCESS;
  unsigned char RxBuffer[MAX_BUFFER_SIZE_EXTENDED];

  Log0(PCSC_LOG_DEBUG);

  /* Check parameters */
  CheckThis;
  if ((NULL == CAPDU) || (0 == CAPDULength)
      || (NULL == RAPDU) || (NULL == RAPDULength)) {
    return EFI_INVALID_PARAMETER;
  }

  UsbCcidDevice = USB_CCID_DEV_FROM_SMART_CARD_READER_PROTOCOL (This);

  /* Check the card is present and powered */
  update_state(UsbCcidDevice);
  if (UsbCcidDevice->State != SCARD_ACTIVE) {
    return EFI_NOT_READY;
  }

  RxLength = sizeof RxBuffer;
  SendPci.Protocol = UsbCcidDevice->CardProtocol;
  response = IFDHTransmitToICC(UsbCcidDevice->Lun, SendPci, CAPDU, CAPDULength, RxBuffer, &RxLength, NULL);
  if (*RAPDULength < RxLength) {
    Status = EFI_BUFFER_TOO_SMALL;
  }
  *RAPDULength = RxLength;

  if (IFD_SUCCESS != response) {
    Status = EFI_DEVICE_ERROR;
  }
  else {
    CopyMem(RAPDU, RxBuffer, RxLength);
  }

  return Status;
}

EFI_STATUS
EFIAPI
SCardControl (
  IN     EFI_SMART_CARD_READER_PROTOCOL *This,
  IN     UINT32                         ControlCode,
  IN     UINT8                          *InBuffer,
  IN     UINTN                          InBufferLength,
     OUT UINT8                          *OutBuffer,
  IN OUT UINTN                          *OutBufferLength
  )
{
  USB_CCID_DEV *UsbCcidDevice;
  RESPONSECODE response;
  DWORD dwBytesReturned;
  UINTN dummy = 0;

  Log0(PCSC_LOG_DEBUG);

  /* Check parameters */
  CheckThis;
  if ((NULL == OutBuffer) && (NULL != OutBufferLength)) {
    return EFI_INVALID_PARAMETER;
  }

  UsbCcidDevice = USB_CCID_DEV_FROM_SMART_CARD_READER_PROTOCOL (This);

  /* OutBufferLength may be NULL */
  if (NULL == OutBufferLength) {
    OutBufferLength = &dummy;
  }

  dwBytesReturned = *OutBufferLength;
  response = IFDHControl(UsbCcidDevice->Lun, ControlCode, InBuffer, InBufferLength, OutBuffer, *OutBufferLength, &dwBytesReturned);
  *OutBufferLength = dwBytesReturned;

  switch (response) {
    case IFD_SUCCESS:
      return EFI_SUCCESS;
    case IFD_ERROR_NOT_SUPPORTED:
      return EFI_UNSUPPORTED;
    case IFD_ERROR_INSUFFICIENT_BUFFER:
      return EFI_BUFFER_TOO_SMALL;
    default:
      return EFI_DEVICE_ERROR;
  }
}

EFI_STATUS
EFIAPI
SCardGetAttrib (
  IN     EFI_SMART_CARD_READER_PROTOCOL *This,
  IN     UINT32                         Attrib,
     OUT UINT8                          *OutBuffer,
  IN OUT UINTN                          *OutBufferLength
  )
{
  USB_CCID_DEV *UsbCcidDevice;
  RESPONSECODE response;
  DWORD dwBytesReturned;

  Log0(PCSC_LOG_DEBUG);

  /* Check parameters */
  CheckThis;
  if ((NULL == OutBuffer) || (NULL == OutBufferLength) || (0 == *OutBufferLength)) {
    return EFI_INVALID_PARAMETER;
  }

  UsbCcidDevice = USB_CCID_DEV_FROM_SMART_CARD_READER_PROTOCOL (This);
  dwBytesReturned = *OutBufferLength;
  response = IFDHGetCapabilities(UsbCcidDevice->Lun, Attrib, &dwBytesReturned, OutBuffer);
  *OutBufferLength = dwBytesReturned;

  switch (response) {
    case IFD_SUCCESS:
      return EFI_SUCCESS;
    case IFD_ERROR_TAG:
      return EFI_UNSUPPORTED;
    case IFD_ERROR_INSUFFICIENT_BUFFER:
      return EFI_BUFFER_TOO_SMALL;
    default:
      return EFI_DEVICE_ERROR;
  }
}

