#!/bin/bash

set -e

# environment variables
EDK_PATH=$(pwd)/../edk2

# define build command
cd $EDK_PATH
source edksetup.sh
cd -

set -x

# Cleaning build dir
rm -rf $EDK_PATH/Build/AppPkg/DEBUG_GCC5/X64/UEFI-SmartCardReader

# build
build \
	--arch=X64 \
	--tagname=GCC5 \
	--platform=../UEFI-SmartCardReader/SmartCardReader.dsc \
	--module=../UEFI-SmartCardReader/SmartCardReader/SmartCardReader.inf

