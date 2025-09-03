# BGT60UTR11AIP with STM32H523CEU6
>This project uses infineons bgt60utr11aip chip to detect the distance of a stationary object. Interfacing this sensor with the stm32h523ceu6 involved implementing the platform functions specific to the stm32 chip in xensiv_bgt60trxx_platform.c.

### Communication protocols
- SPI: spi is used to communicate with the bgt60utr11aip chip.
- UART: uart is used to communicate with an esp32 dev board.

### libraries
- ARM CMSIS DSP
- ARM CMSIS Free RTOS v2
- Platform functions from [infineon git repo](https://github.com/Infineon/sensor-xensiv-bgt60trxx)

### Program Structure

##### 1. Initialization
- **System Clock Setup**  
  - Configures PLL, HSE/LSE, and system frequencies.
- **Peripheral Initialization**  
  - SPI (for radar FIFO reads)  
  - GPIO (power enable, oscillator enable, translator OE, LEDs, IRQ lines)  
  - DMA (for high-speed SPI transfers, optional)  
  - FreeRTOS (task creation, scheduler start)  

##### 2. FreeRTOS Tasks
##### Default Task (`StartDefaultTask`)
- Configures radar power sequence:
  - Enable LDO (`en_ldo_radar`)  
  - Enable oscillator (`osc_en`)  
  - Enable translator (`Translator_OE`)  
  - Initialize LEDs / status GPIOs
- Handles radar chip setup:
  - Reset radar (HW/SW reset)  
  - Configure registers (`register_list[]`)  
  - Setup virtual frame (chirps, samples, repetitions, cutoffs, gain, etc.)

##### Data Acquisition Task
- Waits for **IRQ pin** to signal FIFO ready.
- Calls `xensiv_bgt60trxx_get_fifo_data()` to fetch samples (1024 samples per chirp).
- Uses **SPI burst reads** to unpack 24-bit words into 12-bit ADC samples.
- Optional: DMA mode for continuous FIFO reads.

##### Signal Processing Task
- Applies **window function** (e.g., Hann).
- Runs **FFT (CMSIS-DSP RFFT)**:
  - Input: 1024 ADC samples  
  - Output: 512 magnitude bins
- Post-processing:
  - Remove DC bias  
  - Drop low bins (1–20) to suppress noise  
  - Detect peaks, compute **range bins** (map FFT index → distance)

##### Application Task
- Aggregates results across multiple frames.  
- Computes distance estimates.  
- Can recognize static vs moving objects.  
- Sends results to:
  - UART (debug output)  
  - External database or network (future expansion)  

##### 3. Memory Management
- Uses **FreeRTOS heap_4.c** (best-fit allocator with coalescing).  
- Large buffers allocated statically when possible:
  - `uint16_t data[1024]`  
  - `float32_t fft_output[1024]`  
  - `float32_t mag[512]`  

##### 4. Program Flow
1. **Boot → System Init**  
2. **Start FreeRTOS Scheduler**  
3. **Default Task powers radar**  
4. **Data Acquisition Task reads samples**  
5. **Signal Processing Task computes FFT**  
6. **Application Task interprets distances**  
7. **Loop indefinitely**

## Setting the Dev Environment

### Install Git
### Installing OpenOCD for STM32 (h5ceu6) (stlinkv3mini)
>Dependencies: make, libtool, pkg-config, autoconf, automake, texinfo. Libusb-1.0
- `git clone [openocd for stm32 repo url]`
- `./bootstrap`
- `./configure --help`    .
- `/configure [options]`
- `(./configure --enable-stlink)`
- `make`
- `sudo make install`
### Install cmake
### Install ninja-build
### Install gdb-multiarch
### Install vscode – get extensions (cortex debug, c++ extension pack, graphical debugger), configure paths

### If  Error: libusb_open() failed with LIBUSB_ERROR_ACCESS Error: open failed
Add rule to /etc/udev/rules.d/(rulename.rules)
- use lsusb to find IDs

>`SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="374e", MODE="0666"`
add your user to the plugdev group
`sudo usermod -a -G plugdev $USER`

### Initializing bare gdb server using open ocd
- `gdb`
- `openocd -f interface/stlink-dap.cfg -f target/stm32h5x.cfg`
- `target extended-remote :3333`

### WSL USB passthrough:
- wsl shell session open in background
- Install USBIPD (if not already installed): 
- In powershell:
    - `winget install –interactive –exact dorsal.usbipd-win`
- share the device:
    - `usbipd list`
    - `usbipd bind –busid <busid>`

- attach device:
    - `usbipd attach –wsl –busid <busid>`

 
### Initializing a git repo:
1.     git init
2.     git add .
3.     git remote add origin <github repo url>
4.     git commit -m “Initial commit”
5.     git push origin main
6.     Submodules: adds subdirectory that git sees as submodule, doesn’t track changes when not in directory.
    1.     git submodule add <github repo url>
    2.     Cloning project with submodules:
        1.     git clone <github repo url>
        2.     git submodule init
        3.     git submodule update
    3.     Updating submodules in main project:
        1.     git submodule update --remote
7.     moving folders: git mv <location> <destination>

