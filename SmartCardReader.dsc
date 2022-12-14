## @file
#   Intel(r) UEFI Application Development Kit for EDK II.
#   This package contains applications which depend upon Standard
#   Libraries from the StdLib package.
#
#   See the comments in the [LibraryClasses.IA32] and [BuildOptions]
#   sections for important information about configuring this package
#   for your environment.
#
#   Copyright (c) 2010 - 2012, Intel Corporation. All rights
#   reserved.<BR> This program and the accompanying materials are
#   licensed and made available under the terms and conditions of the
#   BSD License which accompanies this distribution. The full text of
#   the license may be found at
#   http://opensource.org/licenses/bsd-license.
#
#   THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS"
#   BASIS, WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER
#   EXPRESS OR IMPLIED.
##

[Defines]
  PLATFORM_NAME                  = AppPkg
  PLATFORM_GUID                  = 0458dade-8b6e-4e45-b773-1b27cbda3e06
  PLATFORM_VERSION               = 0.01
  DSC_SPECIFICATION              = 0x00010006
  OUTPUT_DIRECTORY               = Build/AppPkg
  SUPPORTED_ARCHITECTURES        = IA32|IPF|X64
  BUILD_TARGETS                  = DEBUG|RELEASE
  SKUID_IDENTIFIER               = DEFAULT

#
#  Debug output control
#
  DEFINE DEBUG_ENABLE_OUTPUT      = TRUE       # Set to TRUE to enable debug output
  DEFINE DEBUG_PRINT_ERROR_LEVEL  = 0x80ffffff  # Flags to control amount of debug output
  DEFINE DEBUG_PROPERTY_MASK      = 0xFF

[PcdsFeatureFlag]

[PcdsFixedAtBuild]
  gEfiMdePkgTokenSpaceGuid.PcdDebugPropertyMask|$(DEBUG_PROPERTY_MASK)
  gEfiMdePkgTokenSpaceGuid.PcdDebugPrintErrorLevel|$(DEBUG_PRINT_ERROR_LEVEL)

[LibraryClasses]
  #
  # Entry Point Libraries
  #
  UefiApplicationEntryPoint|MdePkg/Library/UefiApplicationEntryPoint/UefiApplicationEntryPoint.inf
  ShellCEntryLib|ShellPkg/Library/UefiShellCEntryLib/UefiShellCEntryLib.inf
  UefiDriverEntryPoint|MdePkg/Library/UefiDriverEntryPoint/UefiDriverEntryPoint.inf
  #
  # Common Libraries
  #
  BaseLib|MdePkg/Library/BaseLib/BaseLib.inf
  BaseMemoryLib|MdePkg/Library/BaseMemoryLib/BaseMemoryLib.inf
  UefiLib|MdePkg/Library/UefiLib/UefiLib.inf
  PrintLib|MdePkg/Library/BasePrintLib/BasePrintLib.inf
  PcdLib|MdePkg/Library/BasePcdLibNull/BasePcdLibNull.inf
  MemoryAllocationLib|MdePkg/Library/UefiMemoryAllocationLib/UefiMemoryAllocationLib.inf
  UefiBootServicesTableLib|MdePkg/Library/UefiBootServicesTableLib/UefiBootServicesTableLib.inf
  UefiRuntimeServicesTableLib|MdePkg/Library/UefiRuntimeServicesTableLib/UefiRuntimeServicesTableLib.inf
  !if $(DEBUG_ENABLE_OUTPUT)
    DebugLib|MdePkg/Library/UefiDebugLibConOut/UefiDebugLibConOut.inf
    DebugPrintErrorLevelLib|MdePkg/Library/BaseDebugPrintErrorLevelLib/BaseDebugPrintErrorLevelLib.inf
  !else   ## DEBUG_ENABLE_OUTPUT
    DebugLib|MdePkg/Library/BaseDebugLibNull/BaseDebugLibNull.inf
  !endif  ## DEBUG_ENABLE_OUTPUT

  DevicePathLib|MdePkg/Library/UefiDevicePathLib/UefiDevicePathLib.inf
  RegisterFilterLib|MdePkg/Library/RegisterFilterLibNull/RegisterFilterLibNull.inf


###############################################################################
#
# Components Section - list of the modules and components that will be
#   processed by compilation tools and the EDK II tools to generate
#   PE32/PE32+/Coff image files.
#
# Note: The EDK II DSC file is not used to specify how compiled binary
#   images get placed into firmware volume images. This section is just
#   a list of modules to compile from source into UEFI-compliant
#   binaries.  It is the FDF file that contains information on combining
#   binary files into firmware volume images, whose concept is beyond
#   UEFI and is described in PI specification.  Binary modules do not
#   need to be listed in this section, as they should be specified in
#   the FDF file.  For example: Shell binary (Shell_Full.efi), FAT
#   binary (Fat.efi), Logo (Logo.bmp), and etc.  There may also be
#   modules listed in this section that are not required in the FDF
#   file, When a module listed here is excluded from FDF file, then
#   UEFI-compliant binary will be generated for it, but the binary will
#   not be put into any firmware volume.
#
###############################################################################

[Components]
#### SmartCardReader driver
  ../UEFI-SmartCardReader/SmartCardReader/SmartCardReader.inf
