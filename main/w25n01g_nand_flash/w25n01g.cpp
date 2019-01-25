#include "w25n01g.h"

static const char *TAG = "nand_flash";


ext_flash::ext_flash()
{
    spi = NULL;

    capacity = EXT_FLASH_CAPACITY;
    sector_sz = EXT_FLASH_SECTOR_SIZE;
}

ext_flash::~ext_flash()
{
    if (spi)
    {
        spi_bus_remove_device(spi);
        spi_bus_free(bus);
    }
}


esp_err_t ext_flash::init(const ext_flash_config_t *config)
{
    ESP_LOGD(TAG, "%s", __func__);

    esp_err_t err;

    cfg = *config;

    if ((cfg.sector_size != 0) + (cfg.capacity != 0) == 1)
    {
        ESP_LOGE(TAG, "sector_size and capacticy config values must both be set (or neither)");
        return ESP_ERR_INVALID_ARG;
    }

    if (cfg.queue_size < 1)
    {
        ESP_LOGE(TAG, "queue_size config value must be greather than 0");
        return ESP_ERR_INVALID_ARG;
    }

    if (cfg.speed_mhz <= 0)
    {
        ESP_LOGE(TAG, "speed_mhz config value must be greater than 0");
        return ESP_ERR_INVALID_ARG;
    }

    if (cfg.dma_channel != 1 && cfg.dma_channel != 2)
    {
        ESP_LOGE(TAG, "dma_channel config value must either 1 or 2");
        return ESP_ERR_INVALID_ARG;
    }

    if ((cfg.hd_io_num != -1) + (cfg.wp_io_num != -1) == 1)
    {
        ESP_LOGE(TAG, "hd_io_num and wp_io_num config values must both be set");
        return ESP_ERR_INVALID_ARG;
    }

    spi_bus_config_t buscfg =
    {
        .mosi_io_num = cfg.mosi_io_num,
        .miso_io_num = cfg.miso_io_num,
        .sclk_io_num = cfg.sck_io_num,
        .quadwp_io_num = cfg.wp_io_num,
        .quadhd_io_num = cfg.hd_io_num,
        .max_transfer_sz = cfg.max_dma_size ? cfg.max_dma_size : SPI_MAX_DMA_LEN,
        .flags = SPICOMMON_BUSFLAG_MASTER
    };

    spi_device_interface_config_t devcfg =
    {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = cfg.speed_mhz * 1000 * 1000,
		.input_delay_ns = 0,
        .spics_io_num = cfg.ss_io_num,
        .flags = SPI_DEVICE_NO_DUMMY,
        .queue_size = cfg.queue_size,
        .pre_cb = NULL,
        .post_cb = NULL
    };

    bus = cfg.vspi ? VSPI_HOST : HSPI_HOST;

    err = spi_bus_initialize(bus, &buscfg, cfg.dma_channel);
    if (err != ESP_OK)
    {
        return err;
    }

    err = spi_bus_add_device(bus, &devcfg, &spi);
    if (err != ESP_OK)
    {
        spi_bus_free(bus);
        return err;
    }

    /* Chip select pin initialization. We're using GPIO 4 */
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = (gpio_int_type_t)GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
//    io_conf.pin_bit_mask = (1UL << (cfg.cs_io_num));
    io_conf.pin_bit_mask = GPIO_SEL_4;
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(cfg.cs_io_num, 1);

    reset();

    uint8_t stat_cfg_reg = 0;
    stat_cfg_reg = ((read_stat_reg(EXT_FLASH_STAT_CFG_REG) & 0x1F) | 0x18);

    err = ESP_FAIL;

	if (read_device_id() == EXT_FLASH_JEDECID)
	{
//		printf("NAND ID: 0x%x\n\r", read_device_id());
		write_stat_reg(EXT_FLASH_STAT_CFG_REG, stat_cfg_reg);
		write_stat_reg(EXT_FLASH_STAT_PROT_REG, 0x02);
		err = ESP_OK;
	}

    if (!nand_mutex) {
    	nand_mutex = xSemaphoreCreateMutex();
    }

    return err;
}

void ext_flash::term()
{
    ESP_LOGD(TAG, "%s", __func__);

    if (spi)
    {
        reset();

        spi_bus_remove_device(spi);
        spi = NULL;

        spi_bus_free(bus);
    }
}


void ext_flash::wait_for_device_idle()
{
    int i = 0;

    while (read_stat_reg(EXT_FLASH_STAT_REG) & 0x01)
    {
        vTaskDelay(1);
        if (++i > 10000) break;
    }
}
void ext_flash::reset()
{
    ESP_LOGD(TAG, "%s", __func__);

	wait_for_device_idle();
	chip_select();
	spi_write_byte(EXT_FLASH_RESET);
	chip_deselect();
}

void ext_flash::write_enable()
{
	chip_select();
	spi_write_byte(EXT_FLASH_CMD_WRITEENABLE);
	chip_deselect();
}

void ext_flash::write_disable()
{
	chip_select();
	spi_write_byte(EXT_FLASH_CMD_WRITEDISABLE);
	chip_deselect();
}

size_t ext_flash::sector_size()
{
    ESP_LOGD(TAG, "%s - %d = %d KB", __func__, sector_sz, sector_sz/1024);

    return sector_sz;
}

size_t ext_flash::chip_size()
{
    ESP_LOGD(TAG, "%s - %d = %d MB", __func__, capacity, capacity/(1024*1024));

    return capacity;
}

esp_err_t ext_flash::erase_sector(size_t sector)
{
	ESP_LOGD(TAG, "%s - 0x%x: sector=0x%08x = %d, page=%d", __func__, sector*sector_sz, sector, sector, sector * EXT_FLASH_PAGE_BLOCK_COUNT);

	esp_err_t ret = 0;
	uint8_t buf[4];
	volatile uint8_t status;

    xSemaphoreTake(nand_mutex, portMAX_DELAY);

	wait_for_device_idle();
	write_enable();
//	uint16_t page = addr / EXT_FLASH_PAGE_SIZE; // Counting page number
	uint16_t page = sector * EXT_FLASH_PAGE_BLOCK_COUNT; // Counting page number /*+++ Simply choosing the block to erase here (rather than dividing by EXT_FLASH_PAGE_SIZE) */
	buf[0] = EXT_FLASH_CMD_BLOCKERASE;
	buf[1] = DUMMY_BYTE;
	buf[2] = page >> 8;
	buf[3] = page;

	chip_select();
	ret = spi_write(buf, 4);
	if(ret != ESP_OK){
		return NAND_SPI_WRITE_ERR;
	}

	chip_deselect();

	status = read_stat_reg(EXT_FLASH_STAT_REG);
	if (status & 0x4)
	{
		//Erase Failure
		status = 0;
		ESP_LOGE(TAG, "Sector %d erase error.\n\r", sector);
		return NAND_SPI_SECTOR_ERASE_ERR;
	}

	wait_for_device_idle();	// Should be waited until the flash is done erase operation!

	xSemaphoreGive(nand_mutex);

    return ESP_OK;
}

esp_err_t ext_flash::erase_range(size_t addr, size_t size)
{
	esp_err_t ret = 0;

    ESP_LOGD(TAG, "%s - add=0x%08x (%d) size=%d \n\r", __func__, addr, addr/sector_sz, size);

    while (size > 0)
    {
        write_enable();
        ret = erase_sector(addr/sector_sz);
        wait_for_device_idle();

        addr += sector_sz;
        size -= sector_sz;
    }

    return ret;
}

esp_err_t ext_flash::erase_chip()
{
    ESP_LOGD(TAG, "%s", __func__);

    write_enable();

	for (uint16_t i = 0; i < EXT_FLASH_BLOCK_COUNT; ++i)
	{
		erase_sector(i);
	}

    wait_for_device_idle();

    ESP_LOGD(TAG, "%s - chip erased!\n\r", __func__);

    return ESP_OK;
}

esp_err_t ext_flash::write(size_t addr, const void *src, size_t size)
{
    ESP_LOGD(TAG, "%s - 0x%08x  block: %d  page: %d   %d bytes", __func__, addr, addr/sector_sz, addr/2048, size);

    uint8_t *buf = (uint8_t *)src;
	uint32_t offset = 0;
	uint8_t wr_buf[4];
	uint32_t count;
	uint32_t address;
//	uint32_t count_sectors = 0;
	volatile uint8_t status;

    xSemaphoreTake(nand_mutex, portMAX_DELAY);

//	//Counting blocks for erasing
//	count_sectors = (addr + size)/EXT_FLASH_SECTOR_SIZE - addr/EXT_FLASH_SECTOR_SIZE;
//
//	// If needed erase blocks.
//	// if number of first block not equal number of last block OR number of blocks is whole
//	if (((addr/EXT_FLASH_SECTOR_SIZE) != ((addr+size)/EXT_FLASH_SECTOR_SIZE)) || (addr%EXT_FLASH_SECTOR_SIZE == 0))
//	{
//		// End address of writing on the beginning of the new block
//		if((addr + size) % EXT_FLASH_SECTOR_SIZE == 0)
//		{
//			count_sectors -= 1;
//		}
//		//
//		if(addr % EXT_FLASH_SECTOR_SIZE != 0)
//		{
//			//Begin erasing from the next block
//			for (count = 0; count < count_sectors; ++count)
//			{
//				erase_sector(((addr/EXT_FLASH_SECTOR_SIZE)+count+1)% EXT_FLASH_BLOCK_COUNT);/*+++ We are sending the block that we needs to be erased here */
//				wait_for_device_idle();
//				printf("*** block %d erased!\n\r", ((addr/EXT_FLASH_SECTOR_SIZE)+count+1)% EXT_FLASH_BLOCK_COUNT);
//			}
//		}
//		else
//		{
//			//Begin erasing from the current block
//			for (count = 0; count <= count_sectors; ++count)
//			{
//				erase_sector(((addr/EXT_FLASH_SECTOR_SIZE)+count)% EXT_FLASH_BLOCK_COUNT);/*+++ and here */
//				wait_for_device_idle();
//				printf("*** block %d erased!\n\r", ((addr/EXT_FLASH_SECTOR_SIZE)+count)% EXT_FLASH_BLOCK_COUNT);
//			}
//		}
//	}

	//g_Spi1.LockMutex();

	while (size > 0)
	{
		count = size;
		if ((addr/EXT_FLASH_PAGE_SIZE) != ((addr+size)/EXT_FLASH_PAGE_SIZE))
		{
			count = EXT_FLASH_PAGE_SIZE - (addr % EXT_FLASH_PAGE_SIZE);
		}

		write_enable();

		//Set column address
		address = (addr % EXT_FLASH_CAPACITY) % EXT_FLASH_PAGE_SIZE;

		wr_buf[0] = (uint8_t)EXT_FLASH_CMD_BYTEPAGEPROGRAM;
		wr_buf[1] = (uint8_t)(address >> 8);
		wr_buf[2] = (uint8_t)(address);
		chip_select();
		spi_write(wr_buf, 3);
		spi_write(&buf[offset], count);
		chip_deselect();

		wait_for_device_idle();

		//Set page address
		address = (addr % EXT_FLASH_CAPACITY) / EXT_FLASH_PAGE_SIZE;
		wr_buf[0] = (uint8_t)EXT_FLASH_CMD_EXECUTE;
		wr_buf[1] = (uint8_t)DUMMY_BYTE;
		wr_buf[2] = (uint8_t)(address >> 8);
		wr_buf[3] = (uint8_t)(address);
		chip_select();
		spi_write(wr_buf, 4);
		chip_deselect();

		status = read_stat_reg(EXT_FLASH_STAT_REG);
		if (status & 0x8)
		{
			//Program Failure
			status = 0;
			ESP_LOGE(TAG, "%s NAND program ERROR. addr: 0x%x", __func__, addr);
			return NAND_FLASH_READ_ERROR;
		}
		wait_for_device_idle();
		size -= count;
		addr += count;
		offset += count;
	}

	xSemaphoreGive(nand_mutex);

    return ESP_OK;
}

esp_err_t ext_flash::read(size_t addr, void *dest, size_t size)
{
	ESP_LOGD(TAG, "%s - 0x%08x  block: %d  page: %d   %d bytes", __func__, addr, addr/sector_sz, addr/2048, size);

	uint8_t *buf = (uint8_t *)dest;
	uint8_t wr_buf[4] = { 0, 0, 0, 0 };
	uint32_t address;
	volatile uint8_t status;

    xSemaphoreTake(nand_mutex, portMAX_DELAY);

    uint32_t offset = 0;
	uint32_t count;

	while (size > 0)
	{
		count = size;
		if ((addr/EXT_FLASH_PAGE_SIZE) != ((addr+size)/EXT_FLASH_PAGE_SIZE))
		{
			count = EXT_FLASH_PAGE_SIZE - (addr % EXT_FLASH_PAGE_SIZE);
		}

		wait_for_device_idle();

		//Set page address
		address = (addr % EXT_FLASH_CAPACITY) / EXT_FLASH_PAGE_SIZE;
		wr_buf[0] = (uint8_t)EXT_FLASH_CMD_PAGEREAD;
		wr_buf[1] = (uint8_t)DUMMY_BYTE;
		wr_buf[2] = (uint8_t)(address >> 8);
		wr_buf[3] = (uint8_t)(address);
		chip_select();
		spi_write(wr_buf, 4);
		chip_deselect();

		wait_for_device_idle();

		//Set column address
		address = (addr % EXT_FLASH_CAPACITY) % EXT_FLASH_PAGE_SIZE;
		wr_buf[0] = (uint8_t)EXT_FLASH_CMD_READDATA;
		wr_buf[1] = (uint8_t)(address >> 8);
		wr_buf[2] = (uint8_t)(address);
		wr_buf[3] = (uint8_t)DUMMY_BYTE;
		chip_select();
		spi_write(wr_buf, 4);
		spi_read(&buf[offset], count);
		chip_deselect();

		status = read_stat_reg(EXT_FLASH_STAT_REG);
		if (status & 0x30)
		{
			//ERROR ECC
			status = 0;
			ESP_LOGE(TAG, "%s NAND read ERROR ECC. addr: 0x%x", __func__, addr);
			return NAND_FLASH_READ_ERROR;
		}

		size -= count;
		addr += count;
		offset += count;
	}

	xSemaphoreGive(nand_mutex);

    return ESP_OK;
}

void ext_flash::chip_select(){
	gpio_set_level(cfg.cs_io_num, 0);
}

void ext_flash::chip_deselect(){
	gpio_set_level(cfg.cs_io_num, 1);
}

/*
 * Read some data to the buffer
 */
uint8_t ext_flash::spi_read(uint8_t *buffer, uint16_t length){
	esp_err_t ret = 0;
	spi_transaction_t transaction;

	memset(&transaction, 0, sizeof(transaction));   // Zero out the transaction

	transaction.length = length * 8;                // Transactions length in bits
	transaction.tx_buffer = NULL;
	transaction.rx_buffer = buffer;                // TODO: check SPI_USE_TXDATA in spi_master.h - there is an issue with DMA
	transaction.user = (void*)1;                    // D/C needs to be set to 1

    // Transmit and receive some data
    ret = spi_device_transmit(spi, &transaction);
    assert(ret == ESP_OK);

	return ret;
}

/*
 * Write some data from buffer
 */
uint8_t ext_flash::spi_write(uint8_t *buffer, uint16_t length){

	esp_err_t ret = 0;
	spi_transaction_t transaction;

	memset(&transaction, 0, sizeof(transaction));   // Zero out the transaction

	transaction.length = length*8;             // Transactions length in bits
	transaction.tx_buffer = buffer;
	transaction.rx_buffer = NULL;       // TODO: check SPI_USE_TXDATA in spi_master.h - there is an issue with DMA
	transaction.user = (void*)1;        // D/C needs to be set to 1

    // Transmit and receive some data
    ret = spi_device_transmit(spi, &transaction);
    if(ret!=0) {
//    	printf("length = %d\n\r buffer = %p\n\r\n\r", length, buffer);
    }
    assert(ret == ESP_OK);

	return ret;
}

uint8_t ext_flash::spi_read_byte(){

	uint8_t byte;
	spi_read(&byte, 1);

	return byte;
}

uint8_t ext_flash::spi_write_byte(uint8_t byte){

	uint8_t ret = 0;
	ret = spi_write(&byte, 1);

	return ret;
}

uint8_t ext_flash::read_stat_reg(uint8_t address){

	uint8_t status = 0 ;
	uint8_t buf[2] = {0} ;

	chip_select();
	buf[0] = EXT_FLASH_CMD_STATUSREAD;
	buf[1] = address;
	spi_write(buf, 2);
	status = spi_read_byte();
	chip_deselect();

	return status;
}

uint32_t ext_flash::read_device_id(void){
	uint32_t jedecid = 0;
	uint8_t buf[2] = {0};
	uint8_t rcv_buf[3] = {0};
	buf[0] = EXT_FLASH_CMD_IDREAD;
	chip_select();
	spi_write(buf, 2);
	spi_read(rcv_buf, 3);
	jedecid = rcv_buf[0] << 16;
	jedecid |= rcv_buf[1] << 8;
	jedecid |= rcv_buf[2];
	chip_deselect();

 	return jedecid;
}

void ext_flash::write_stat_reg(uint8_t address, uint8_t data){
	uint8_t buf[3] = {0} ;
	chip_select();
	buf[0] = EXT_FLASH_CMD_STATUSWRITE;
	buf[1] = address;
	buf[2] = data;
	spi_write(buf, 3);
	chip_deselect();
}

