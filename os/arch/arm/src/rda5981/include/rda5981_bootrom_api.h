#ifndef RDA5991H_BOOTROM_API_H_
#define RDA5991H_BOOTROM_API_H_

//#define RDA5991H_U04
#define RDA5991H_U02

#define RDA5991H_DEBUG

//#if defined RDA5991H_U04
#if defined (UNO_81A_U04) || defined (UNO_81AM_U04)|| defined (UNO_81C_U04)
#define FLASH_ERASE_FUN_ADDR 0x2221//smartlink_erase_for_mbed
#define FLASH_WRITE_FUN_ADDR 0x2271//smartlink_write_for_mbed
#define FLASH_READ_FUN_ADDR 0x2273//smartlink_read_for_mbed
#define SPI_FLASH_READ_DATA_FOR_MBED_ADDR 0x2037//spi_flash_read_data_for_mbed
#define UART_SEND_BYTE_ADDR 0x1be5//uart_send_byte
#define UART_RECV_BYTE_ADDR 0x1ca7//uart_recv_byte
#define BOOT_ADDR 0xabf5//void boot(u32 addr)
#define CRC32_ADDR 0x8e33//u32 crc32(const u8 *p, size_t len)
#define spi_flash_erase_4KB_sector_addr 0x23d3
#define spi_wip_reset_addr 0x1dbb
#define spi_write_reset_addr 0x1dcf
#define wait_busy_down_addr 0x1db1
//#elif defined RDA5991H_U02
#elif defined (UNO_81A_U02) || defined (UNO_81AM_U02)|| defined (UNO_81C_U02)
#define FLASH_ERASE_FUN_ADDR 0x21f1//smartlink_erase_for_mbed
#define FLASH_WRITE_FUN_ADDR 0x2241//smartlink_write_for_mbed
#define FLASH_READ_FUN_ADDR 0x2243//smartlink_read_for_mbed
#define SPI_FLASH_READ_DATA_FOR_MBED_ADDR 0x2007//spi_flash_read_data_for_mbed
#define UART_SEND_BYTE_ADDR 0x1bb5//uart_send_byte
#define UART_RECV_BYTE_ADDR 0x1c77//uart_recv_byte
#define BOOT_ADDR 0xab93//void boot(u32 addr)
#define CRC32_ADDR 0x8dff//u32 crc32(const u8 *p, size_t len)
#define spi_flash_erase_4KB_sector_addr 0x23a3
#define spi_wip_reset_addr 0x1d8b
#define spi_write_reset_addr 0x1d9f
#define wait_busy_down_addr 0x1d81
#else
#define FLASH_ERASE_FUN_ADDR 0x21f3//smartlink_erase_for_mbed
#define FLASH_WRITE_FUN_ADDR 0x225d//smartlink_write_for_mbed
#define FLASH_READ_FUN_ADDR 0x225f//smartlink_read_for_mbed
#define SPI_FLASH_READ_DATA_FOR_MBED_ADDR 0x1fd9//spi_flash_read_data_for_mbed
#define UART_SEND_BYTE_ADDR 0x1bab//uart_send_byte
#define UART_RECV_BYTE_ADDR 0x1c6d//uart_recv_byte
#define BOOT_ADDR 0xaafb//void boot(u32 addr)
#define CRC32_ADDR 0x8d67//u32 crc32(const u8 *p, size_t len)
#endif

#define rda5991h_erase_flash(addr, len) \
    (((void(*)(void *, unsigned int))FLASH_ERASE_FUN_ADDR)(addr, len))
#define rda5991h_write_flash(addr, data, len) \
    (((void(*)(void *, const void *, unsigned int))FLASH_WRITE_FUN_ADDR)(addr, data, len))
#define rda5991h_read_flash(addr, buf, len) \
    (((void(*)(void *, void *, unsigned int))FLASH_READ_FUN_ADDR)(addr, buf, len))
#define spi_flash_read_data_for_mbed(addr, buf, len) \
        (((void(*)(void *, void *, unsigned int))SPI_FLASH_READ_DATA_FOR_MBED_ADDR)(buf, addr, len))

#define wait_busy_down() \
        (((void(*)(void))wait_busy_down_addr)())
#define spi_write_reset() \
        (((void(*)(void))spi_write_reset_addr)())
#define spi_wip_reset() \
        (((void(*)(void))spi_wip_reset_addr)())
#define rda5981_spi_flash_erase_4KB_sector(addr) \
        (((void(*)(unsigned int))spi_flash_erase_4KB_sector_addr)(addr))

#ifdef RDA5991H_DEBUG
#define uart_send_byte(c) \
	(((void(*)(const unsigned char))UART_SEND_BYTE_ADDR)(c))
#define uart_recv_byte() \
	(((unsigned char(*)(void))UART_RECV_BYTE_ADDR)())
#else /*RDA5991H_DEBUG*/
#define uart_send_byte(c) \
	do{}while(0)
#define uart_recv_byte() -1
#endif /*RDA5991H_DEBUG*/

#define rda_boot(addr) \
    (((void(*)(unsigned int))BOOT_ADDR)(addr))
#define crc32(p, len) \
    (((unsigned int(*)(unsigned char *, unsigned int))CRC32_ADDR)(p, len))
#endif
