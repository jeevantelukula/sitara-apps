./CMSIS_5 contains the TI CMSIS_5 patch to the ARM CMSIS_5 library, v5.6.0
./lib_prebuild contains the Pre-built CMSIS DSP library in binary for R5 core using TI Code Gen Tools

Here are the steps to rebuild the CMSIS DSP library for R5 core using TI Code Gen Tools.

Required Software:
    1) ARM CMSIS_5 library, v5.6.0
        URL: https://github.com/ARM-software/CMSIS_5
        Git Repository: https://github.com/ARM-software/CMSIS_5.git
        Git Tag / Commit ID: 5.6.0 / b5f0603
        
        Clone Git repo to ~/ti/CMSIS_5

    2) TI AM65xx Processor-SDK-RTOS, v06.03.00.106
        URL: http://software-dl.ti.com/processor-sdk-rtos/esd/AM65X/latest/index_FDS.html
        
        Install to ~/ti_am65xx_06_03_00_106
        
    3) TI CMSIS_5 patch
        Copy ./CMSIS_5 to <CMSIS install directory>, i.e. ~/ti/CMSIS_5 (overwrite files)
    
CMSIS_5 Library Build:
    1) Change $(DEPOT) to parent folder of CMSIS_5 (absolute path name is needed)

    2) Change $(TOOLS) to the absolute path name for ti_am65xx_06_03_00_106

    3) Open shell
       > cd ~/ti/CMSIS_5/CMSIS/DSP
       > make clean
       > make all
    
    4) Resulting archive: ~/ti/CMSIS_5/CMSIS/DSP/bin/release/ti_math_Cortex_R5_lspf.lib
        
