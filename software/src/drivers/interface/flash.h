#ifndef __FLASH_H__
#define __FLASH_H__


#include "stm32f4xx.h"

//typedef enum {false , true} bool;
#define false     0
#define true      1
#define bool _Bool

/**********************************************************************************************************
*单片机Flash存储区域分配
**********************************************************************************************************/
//Flash扇区的基地址
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

//    项目            开始地址	          结束地址	      大小
//	bootloader	 0x8000000 sector0	 0x800BFFF sector2	  48KB
//	boot_para  	 0x800C000 sector3	 0x800FFFF sector3	  16KB
//	application	 0x8010000 sector4	 0x807FFFF sector7	  448KB
//	bin file	 0x8080000 sector8	 0x80DFFFF sector10	  384KB
//	app_para	 0x80E0000 sector11	 0x80FFFFF sector11	  128KB
#define FLASH_BASE_START_ADDR           0x08000000 	    //Flash的起始地址
#define FLASH_BASE_END_ADDR             0x080FFFFF 	    //Flash的结束地址

#define FLASH_BOOT_START_ADDR		    ADDR_FLASH_SECTOR_0  	//Bootloader存储区
#define FLASH_BOOT_PARA_START_ADDR      ADDR_FLASH_SECTOR_3  	//Bootloader参数存储区
#define FLASH_USER_START_ADDR		    ADDR_FLASH_SECTOR_4  	//用户程序存储区
#define FLASH_BIN_START_ADDR            ADDR_FLASH_SECTOR_8  	//固件升级bin文件存储区
#define FLASH_USER_PARA_START_ADDR      ADDR_FLASH_SECTOR_11  	//用户参数存储区

#define FLASH_VECTOR_TAB_OFFSET         0x10000

uint8_t Flash_ReadByte(uint32_t start_addr, uint16_t cnt);
bool Flash_WriteByte(uint32_t dest,uint8_t *src,uint32_t length);

#endif
