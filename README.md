# Custom_Bootloader

A custom, bare-metal bootloader written in C that allows for seamless firmware updates and application management on a microcontroller. It communicates with a host PC over a Serial/UART connection, allowing you to update the user application without needing a dedicated hardware programmer every time. 

To ensure the integrity and safety of the firmware during transmission, CRC (Cyclic Redundancy Check) verification is implemented for all host-to-target communications.

## 📁 Project Structure

* **/bootloader**: The core C code for the bootloader. This is the first program that runs when the microcontroller powers up.
* **/user_application**: The main application firmware that the bootloader jumps to after successful verification or update.
* **host_commands.py**: A Python script run on the host PC to send commands and firmware binaries to the microcontroller.

## 🛠️ Supported Host Commands 

The Python host script interacts with the bootloader using the following command set:

* **BL_GET_VER**: Fetches the current version of the bootloader.
* **BL_GET_HELP**: Retrieves a list of all supported commands from the bootloader.
* **BL_GET_CID**: Reads the Chip Identification number (CID) of the microcontroller.
* **BL_GET_RDP_STATUS**: Checks the current Read Protection (RDP) level of the flash memory.
* **BL_GO_TO_ADDR**: Instructs the bootloader to jump to a specific memory address and execute the code there.
* **BL_FLASH_ERASE**: Erases specific sectors or performs a mass erase of the flash memory.
* **BL_MEM_WRITE**: Writes new data (like a user application payload) into the flash memory.
* **BL_MEM_READ**: Reads data from a specified memory address.
* **BL_EN_RW_PROTECT**: Enables Read/Write protection for specific flash sectors.
* **BL_DIS_RW_PROTECT**: Disables Read/Write protection for the flash sectors.
* **BL_READ_SECTOR_P_STATUS**: Reads the current protection status of all flash sectors.
* **BL_OTP_READ**: Reads contents from the MCU's One-Time Programmable (OTP) memory.

## 🚀 How to setup

1. **Initial Setup (Hardware Flash):** Compile the code in the `/bootloader` directory and flash it to the base address of your microcontroller's flash memory using a standard hardware programmer (e.g., ST-Link, J-Link).
2. **Build Your Application:** Compile the code in the `/user_application` directory to generate your application binary (`.bin`). Ensure your application's linker script is configured to start at the correct memory offset, leaving room for the bootloader.
3. **Connect to the Host:** Connect your microcontroller to your PC via a USB-to-Serial (UART) adapter.
4. **Run the Host Script:** Open your terminal, navigate to the project directory, and run the Python script:
   ```bash
   python host_commands.py
## Author
[@bhoomikahardwani09](https://github.com/bhoomikahardwani09)
