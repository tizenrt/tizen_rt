#ifndef _RDA5981_FLASH_H_
#define _RDA5981_FLASH_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * function: erase flash
 * @addr: mast be 4k alignment
 * @len:  must be 4k alignment. (package 64KB erase and 4KB erase for different condition automatically)
 * return: 0:success, else:fail
 */
int rda5981_erase_flash(unsigned int addr, unsigned int len);

/*
 * function: write flash
 * @addr: mast be 256 alignment
 * @buf: data to be written, best be 4 alignment
 * @len: buffer len, mast be 4 alignment
 * return: 0:success, else:fail
 */
int rda5981_write_flash(unsigned int addr, char *buf, unsigned int len);

/*
 * function: read flash to @buf
 * @addr: best be 4 alignment
 * @buf: best be 4 alignment
 * @len: buffer len
 * return: 0:success, else:fail
 */
int rda5981_read_flash(unsigned int addr, char *buf, unsigned int len);

#ifdef __cplusplus
}
#endif

#endif

