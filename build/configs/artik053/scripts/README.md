# Memory Map (ARTIK05x)
 
### Physical Address Space (4GB)
 
The following is the memory map of ARTIK05x.
 
```
0xFFFFFFFF --------------------------
            High Vector (4KB)
0xFFFF0000 --------------------------
0x88000000 --------------------------
            SFR
0x80000000 --------------------------
0x60800000 --------------------------
            SPI FLASH (8MB) mirrored
0x60000000 --------------------------
0x04800000 --------------------------
            SPI FLASH (8MB)
0x04000000 --------------------------
0x02320000 --------------------------
            SRAM for WiFi (128KB)
0x02300000 --------------------------
0x02160000 --------------------------
            SRAM (1280KB)
0x02020000 --------------------------
            IROM
0x09000000 --------------------------
```



### Flash Partitions (8MB)

8MB is allocated to the SPI Flash area. After building TizenRT, refer to the following areas when downloading to the board.

```
0x04800000 --------------------------
            SSS R/W Key
0x04780000 --------------------------
            WiFi NVRAM
0x0477E000 --------------------------
            User R/W
0x04620000 --------------------------
            OTA download
0x044A0000 --------------------------
            Factory Reset
0x04320000 --------------------------
            OS (TizenRT)
0x040C8000 --------------------------
            WiFi F/W
0x04048000 --------------------------
            SSS F/W
0x04040000 --------------------------
            BL2
0x04010000 --------------------------
            SSS R/O Key
0x04004000 --------------------------
            BL1
0x04000000 --------------------------
```



### SRAM usage (1280KB)

Actually, BL1, OS and WiFi firmware are operated in SRAM.

```
0x02160000 --------------------------
            WiFi (320KB)
0x02110000 --------------------------
            User Data (OS) (946KB)
0x02023800 --------------------------
            BL1
0x02021800 --------------------------
            Reserved
0x02021000 --------------------------
            Vector Table
0x02020000 --------------------------
```
