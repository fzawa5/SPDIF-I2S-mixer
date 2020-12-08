# SPDIF-I2S-mixer

This software is a digital audio mixer running on an STM32 microcontroller, mixing S/PDIF and I2S and outputting it as S/PDIF.

Target MCU is STM32F446RE (NUCLEO-F446RE).
SPDIFRX module receives S/PDIF 24bit 48kHz PCM stream from IN0 and SAI1B module receives I2S 32bit 48kHz PCM stream.
CPU adds both streams (I2S stream is compressed to 24bit), and then SAI1A module transmits them by S/SPDIF protocol.

This software is based on STM32CubeIDE 1.4.0 (for macOS) auto generated code.
I/O and mixing function are implemented at [main.c](Core/Src/main.c).
