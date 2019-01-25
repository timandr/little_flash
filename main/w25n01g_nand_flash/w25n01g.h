/*
 * w25n01g.h
 *
 *  Created on: Jan 17, 2019
 *      Author: Andrew Timoshenko
 */

#ifndef W25N01G_NAND_FLASH_W25N01G_H_
#define W25N01G_NAND_FLASH_W25N01G_H_

//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include <stdbool.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
extern "C"{
#include "driver/spi_master.h"
#include "driver/gpio.h"
}
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Sector Size = Block Size = 128KB, Page Size = 2048 bytes
#define EXT_FLASH_PAGE_SIZE				2048
#define EXT_FLASH_P_SECTOR_SIZE			512
#define EXT_FLASH_PAGE_BLOCK_COUNT  	64
#define EXT_FLASH_BLOCK_SIZE			(EXT_FLASH_PAGE_SIZE * EXT_FLASH_PAGE_BLOCK_COUNT)
#define EXT_FLASH_BLOCK_COUNT			1024
#define EXT_FLASH_SECTOR_SIZE			EXT_FLASH_BLOCK_SIZE
#define EXT_FLASH_CAPACITY				(EXT_FLASH_BLOCK_SIZE * EXT_FLASH_BLOCK_COUNT)

#define EXT_FLASH_JEDECID				0xEFAA21
#define DUMMY_BYTE						0x00

/* Commands */
#define EXT_FLASH_CMD_WRITEENABLE		0x06
#define EXT_FLASH_CMD_WRITEDISABLE		0x04
#define EXT_FLASH_CMD_STATUSREAD		0x05
#define EXT_FLASH_CMD_STATUSWRITE		0x01
#define EXT_FLASH_CMD_PAGEREAD			0x13
#define EXT_FLASH_CMD_READDATA			0x03
#define EXT_FLASH_CMD_IDREAD			0x9F
#define EXT_FLASH_CMD_BLOCKERASE		0xD8
#define EXT_FLASH_CMD_BYTEPAGEPROGRAM	0x02
#define EXT_FLASH_CMD_EXECUTE			0x10
#define EXT_FLASH_RESET           		0xFF
#define EXT_FLASH_STAT_PROT_REG			0xAF
#define EXT_FLASH_STAT_CFG_REG			0xBF
#define EXT_FLASH_STAT_REG				0xCF

/* Return statuses */
#define NAND_SPI_WRITE_ERR           1
#define NAND_SPI_READ_ERR            2
#define NAND_SPI_SECTOR_ERASE_ERR    3
#define NAND_SPI_SPI_NOT_ACK         4 // There isn't response from flash
#define NAND_FLASH_READ_ERROR        5 // There isn't response from flash
#define NAND_FLASH_PROG_ERROR        6 // There isn't response from flash

typedef struct
{
    bool vspi;                  // true=VSPI, false=HSPI
    gpio_num_t cs_io_num;
    int8_t sck_io_num;          // any GPIO or VSPICLK = 18, HSPICLK = 14
    int8_t miso_io_num;         // any GPIO or VSPIQ   = 19, HSPIQ   = 12
    int8_t mosi_io_num;         // any GPIO or VSPID   = 23, HSPID   = 13
    int8_t ss_io_num;           // any GPIO or VSPICS0 = 5,  HSPICS0 = 15
    int8_t hd_io_num;           // any GPIO or VSPIHD  = 21, HSPIHD  = 4
    int8_t wp_io_num;           // any GPIO or VSPIWP  = 22, HSPIWP  = 2
    int8_t speed_mhz;           // ex. 10, 20, 40, 80
    int8_t dma_channel;         // must be 1 or 2
    int8_t queue_size;          // size of transaction queue, 1 - n
    int    max_dma_size;        // larger = faster, smaller = less memory, 0 = default
    size_t sector_size;         // sector size or 0 for detection
    size_t capacity;            // number of bytes on flash or 0 for detection
} ext_flash_config_t;

class ext_flash {
public:
	ext_flash();
    virtual ~ext_flash();

    esp_err_t init(const ext_flash_config_t *config);
    void term();

    virtual size_t sector_size();
    virtual size_t chip_size();
    virtual esp_err_t erase_sector(size_t sector);
    virtual esp_err_t erase_range(size_t addr, size_t size);
    virtual esp_err_t erase_chip();
    virtual esp_err_t write(size_t addr, const void *src, size_t size);
    virtual esp_err_t read(size_t addr, void *dest, size_t size);

protected:
    virtual void write_enable();
    virtual void write_disable();
    virtual void wait_for_device_idle();
    virtual void reset();

protected:
    spi_device_handle_t spi;
    size_t sector_sz;
    size_t capacity;

private:

    SemaphoreHandle_t nand_mutex = NULL;

    ext_flash_config_t cfg;
    spi_host_device_t bus;

    void chip_select();
    void chip_deselect();

    uint8_t spi_read(uint8_t *buffer, uint16_t length);
    uint8_t spi_write(uint8_t *buffer, uint16_t length);

    uint8_t spi_read_byte();
    uint8_t spi_write_byte(uint8_t byte);

    uint8_t read_stat_reg(uint8_t address);
    void write_stat_reg(uint8_t address, uint8_t data);
    uint32_t read_device_id(void);
};

#endif /* W25N01G_NAND_FLASH_W25N01G_H_ */
