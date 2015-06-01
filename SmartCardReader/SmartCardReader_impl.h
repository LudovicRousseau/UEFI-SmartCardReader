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

#ifndef _SMART_CARD_READER_IMPL_H_
#define _SMART_CARD_READER_IMPL_H_

#include <Protocol/SmartCardReader.h>

EFI_STATUS
EFIAPI
SCardConnect (
  IN     EFI_SMART_CARD_READER_PROTOCOL *This,
  IN     UINT32                         AccessMode,
  IN     UINT32                         CardAction,
  IN     UINT32                         PreferredProtocols,
     OUT UINT32                         *ActiveProtocol
);

EFI_STATUS
EFIAPI
SCardDisconnect (
  IN EFI_SMART_CARD_READER_PROTOCOL *This,
  IN UINT32                         CardAction
);

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
);

EFI_STATUS
EFIAPI
SCardTransmit (
  IN     EFI_SMART_CARD_READER_PROTOCOL *This,
  IN     UINT8                          *CAPDU,
  IN     UINTN                          CAPDULength,
     OUT UINT8                          *RAPDU,
  IN OUT UINTN                          *RAPDULength
);

EFI_STATUS
EFIAPI
SCardControl (
  IN     EFI_SMART_CARD_READER_PROTOCOL *This,
  IN     UINT32                         ControlCode,
  IN     UINT8                          *InBuffer,
  IN     UINTN                          InBufferLength,
     OUT UINT8                          *OutBuffer,
  IN OUT UINTN                          *OutBufferLength
);

EFI_STATUS
EFIAPI
SCardGetAttrib (
  IN EFI_SMART_CARD_READER_PROTOCOL *This,
  IN UINT32 Attrib,
  OUT UINT8 *OutBuffer,
  IN OUT UINTN *OutBufferLength
);

#endif

