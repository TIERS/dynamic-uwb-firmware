# Firmware for UWB localization with TOF and TDoA using Decawave's DWM1001-DEV board
Firmware for Decawave DWM1001-Dev devices: ToF and TDoA localization

## Installation

**NOTE:** We recomment using Keil µVision IDE if SES version is not available any more.

  1. Download Keil µVision 5 IDE (Note the version to ensure compatibility)
  2. Download Segger J-Flash Lite and GNU ARM Embedded Toolchain 5.4 2016q3
  3. Clone Decawave's code example from https://github.com/Decawave/dwm1001-examples.git

## How to program the devices

Follow these steps to load the initiator and responder firmware on the DWM1001-DEV boards.

### Active Initiator and Responder TOF
1. Open the project inside `dwm1001-examples/examples/ss_twr_init` but replace the files `ss_init_main.c` and `main.c` with the ones of the same name provided in the folder TOF (inside of the Dynamic folder).

2. Open the project in Keil µVision, set the `NODE_ID` to the desired value and set up the desired `sequence`, connect the target device via USB and click Build and Run.

3. For `NODE_ID=1` send an `S` via serial port to perfome one complete sequence. (A python script to automatize the process can be found in the folder Control).

The predefined sequence in the firmware is an all to all for 6 nodes. All nodes need to be active to complete the sequence correctly.


### Passive Listener TDOA
1. Copy the folder `dwm1001-examples/examples/ss_twr_init` and change the name to `ss_twr_list` to identify it.

2. Replace the files `ss_init_main.c` and `main.c` with  `ss_list_main.c` and `main.c` provided in the folder TDOA (inside of the Dynamic folder).

3. Open the project in Keil µVision, connect the target device via USB and click Build and Run.

The device will print via serial terminal all the distances between each pair of nodes and the difference of distance to pair of nodes after one sequence is completed. (Sequence set in `ss_init_main.c`, fomr the TOF folder)

## KEIL µVision IDE

Each example contains a µVision5 project file for Keil µVision IDE. The examples compile and load cleanly to the DWM1001.
The project was created with the KEIL uVision version V5.24.2.0. 

Keil µVision has a free license for project up to 32KB. For more information regarding Keil µVision, please visit http://www2.keil.com/mdk5/uvision/

### µVision Error: Flash Download failed - "Cortex-M4"

This error can be observed if there is a memory conflict between the binary to load and the current firmware on the target hardware. This issue can be easily fixed by fully erasing the target device 's flash memory. Keil µVision cannot perform a full erase and the following free tool can be used :

* J-flash lite 
* nrfjprog command line script

For more information about the issue, please see :

https://devzone.nordicsemi.com/f/nordic-q-a/18278/error-flash-download-failed---cortex---m4-while-flashing-softdevice-from-keil-uvision-5


## Restore the factory firmware

In order to store the factory firmware, first download the `.hex` file from

```
https://www.decawave.com/wp-content/uploads/2019/03/DWM1001_DWM1001-DEV_MDEK1001_Sources_and_Docs_v9.zip
```

Extract the ZIP. You will find the `.hex` factory firmware file in `DWM1001_DWM1001-DEV_MDEK1001_Sources_and_Docs_v9/DWM1001/Factory_Firmware_Image/DWM1001_PANS_R2.0.hex`.

Open SEGGER J-Flash Lite (tested with V6.62a, the executable file is `JFlashLiteExe`) and choose the following fields:

1. Device: `NRF52832_XXAA`
2. Interface: `SWD`
3. Speed: `4000 kHz`
4. Data File: `DWM1001_PANS_R2.0.hex` (downloaded following steps above)

Connect the DWM1001-DEV board to a USB port (we found some issues if you connect it after you start JFlashLite, so we recommend that you connect the dev board **BEFORE** starting JFlashLite). Optionally, click first on `Erase Chip`.

Click on `Program Device`. If you also erased the previous board, you should see the following output:

```
Connecting to J-Link...Connecting to target...Erasing...Done
Conecting to J-Link...Connecting to target...Downloading...Done
```

**NOTE:** The LEDs on the DEV board will be OFF while the firmware is being updated, **do not disconnect the board** until you see the `Done` message and the LEDs are flashing again. After erasing the device, you should see a single non-blinking red LED on the DEV board.
