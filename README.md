# UEFI-SmartCardReader

## Setup

Clone the [edk2](https://github.com/tianocore/edk2) and U[EFI-SmartCardReader](https://github.com/LudovicRousseau/UEFI-SmartCardReader) repositories in the same directory.

Checkout the (stable) version of edk2 you want to use. For example:

```
cd edk2
git checkout edk2-stable202208
```

## Build

To build the UEFI driver run the `./build.sh` script.

```
cd UEFI-SmartCardReader
./build.sh
```

## Configuration

You can edit the file `SmartCardReader/config.h` to *enable* the logs by commenting the line:

```
/* no log or debug messages */
#define NO_LOG
```
