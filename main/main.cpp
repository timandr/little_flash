#include <esp_log.h>
#include <string>
#include <unistd.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "w25n01g_nand_flash/w25n01g.h"
#include "littleflash/include/littleflash.h"

#include "esp_heap_caps.h"

extern "C"
{
#include "unity.h"
#include "test_lfs_common.h"
}

#define UNIT_TESTING true

/* SPI flash's pins definitions */
#define PIN_NUM_MISO 39 // SENSOR_VN
#define PIN_NUM_MOSI 22 //
#define PIN_NUM_CLK  19
#define PIN_NUM_CS   4

#define MOUNT_POINT "/littleflash"
#define OPENFILES 4

static const char *TAG = "main";

extern "C" {
	void app_main(void);
}

// Set to the partition name if using internal flash,
// else comment to use externa flash
//#define CONFIG_LITTLEFS_PARTITION_LABEL  "littlefs"

static ext_flash nand_flash;
static LittleFlash littleflash;

void block_test();
void page_test();
void print_one_page(uint32_t addr);

void print_free_memory(){

    uint32_t psram_kb = heap_caps_get_free_size(MALLOC_CAP_SPIRAM)/1;
    uint32_t iram_kb = heap_caps_get_free_size(MALLOC_CAP_INTERNAL)/1;
    uint32_t rtc_kb = heap_caps_get_free_size(MALLOC_CAP_EXEC)/1;

    printf("\n\r");
    ESP_LOGI(TAG, "\n\rAvailable free memory:\n\r"
    		      "PSRAM: %d = %d KB\n\r"
    		      "IRAM:  %d = %d KB\n\r"
    		      "RTC:   %d = %d KB\n\r"
    		      "\n\r", psram_kb, psram_kb/1024, iram_kb, iram_kb/1024, rtc_kb, rtc_kb/1024);
}

#if UNIT_TESTING == true
void app_main(void)
{
	esp_err_t ret;

	int cap = nand_flash.chip_size();
	int sector_sz = nand_flash.sector_size();
	int step = cap / 1024;

//	uint8_t *rbuf = (uint8_t *) heap_caps_malloc(sector_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
//	if(rbuf != NULL){
//		ESP_LOGI(TAG, "Memory %d KB for rbuf allocated at %p!", sector_sz/1024, (uint32_t*)rbuf);
//	} else ESP_LOGE(TAG, "Memory for rbuf didn't allocated!");
//
//	uint8_t *wbuf = (uint8_t *) heap_caps_malloc(sector_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
//	if(wbuf != NULL){
//		ESP_LOGI(TAG, "Memory %d KB for wbuf allocated at %p!", sector_sz/1024, (uint32_t*)wbuf);
//	} else ESP_LOGE(TAG, "Memory for wbuf didn't allocated!");

	print_free_memory();

    ext_flash_config_t ext_cfg =
    {
        .vspi = false, // We are using HSPI host
		.cs_io_num = GPIO_NUM_4,
        .sck_io_num = PIN_NUM_CLK,
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .ss_io_num = -1,
        .hd_io_num = -1,
        .wp_io_num = -1,
        .speed_mhz = 20,
        .dma_channel = 1,
        .queue_size = 7,
        .max_dma_size = 8192,
        .sector_size = 0,
        .capacity = 0,
    };

    ext_flash_config_t *config = &ext_cfg;
//
    if(nand_flash.init(config) != 0){
    	ESP_LOGE(TAG, "NAND hasn't been initialized!");
    } else{
    	ESP_LOGI(TAG, "NAND has been successfully initialized!");
    }
    print_free_memory();
//    print_one_page(0);
//    print_one_page(2048*64);
//    print_one_page(4096);

//    page_test();
//    block_test();

//    nand_flash.erase_chip(); printf("The chip was erased\n\r");

    const little_flash_config_t little_cfg =
    {
#if !defined(CONFIG_LITTLEFS_PARTITION_LABEL)
        .flash = &nand_flash,
        .part_label = NULL,
#else
        .flash = NULL,
        .part_label = CONFIG_LITTLEFS_PARTITION_LABEL,
#endif
        .base_path = MOUNT_POINT,
        .open_files = OPENFILES,
        .auto_format = true,
        .lookahead = 32
    };


    ret = littleflash.init(&little_cfg);
    if(ret != 0){
    	ESP_LOGE(TAG, "littleflash hasn't been initialized! %d", ret);
    } else{
    	ESP_LOGI(TAG, "littleflash has been successfully initialized!");
    }
    print_free_memory();



    while(1){
    	vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void block_test()
{
//    printf("Read-Write block flash test started\n\r");
//
//	int cap = nand_flash.chip_size();
//	int sector_sz = nand_flash.sector_size();
//	int step = cap / 1024;
//
//	uint8_t *rbuf = (uint8_t *) heap_caps_malloc(sector_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
//	if(rbuf != NULL){
//		ESP_LOGI(TAG, "Memory %d KB for rbuf allocated at %p!", sector_sz/1024, (uint32_t*)rbuf);
//	} else ESP_LOGE(TAG, "Memory for rbuf didn't allocated!");
//
//	uint8_t *wbuf = (uint8_t *) heap_caps_malloc(sector_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
//	if(wbuf != NULL){
//		ESP_LOGI(TAG, "Memory %d KB for wbuf allocated at %p!", sector_sz/1024, (uint32_t*)wbuf);
//	} else ESP_LOGE(TAG, "Memory for wbuf didn't allocated!");
//
//	uint8_t var = 0;
//	for (int i = 0; i < sector_sz; i++)
//	{
//		var++;
//		if(i % 255 == 0) {
//			var = 0;
//		}
//		wbuf[i] = var & 0xff;
//		rbuf[i] = 0;
//	}
//
//	int sectors = 0;
//	int addr;
//
//    struct timeval start;
//    gettimeofday(&start, NULL);
//
//	for (addr = 0; addr < cap; addr += step)
//	{
//		printf("\n\r");
//		sectors++;
//		nand_flash.erase_sector(addr / sector_sz);
//		nand_flash.write(addr, wbuf, sector_sz);
//		memset(rbuf, 0, sector_sz);
//		nand_flash.read(addr, rbuf, sector_sz);
//		bool good = true;
//		for (int b = 0; b < sector_sz; b++)
//		{
//			if (wbuf[b] != rbuf[b])
//			{
//				printf("erase/write/verify failed at block %d offset %d\n", addr / sector_sz, b);
//				for (int i = 0; i < sector_sz; i += 16)
//				{
//					printf("wbuf %08x: "
//						   "%02x%02x%02x%02x "
//						   "%02x%02x%02x%02x "
//						   "%02x%02x%02x%02x "
//						   "%02x%02x%02x%02x ",
//						   addr + i,
//						   wbuf[i + 0], wbuf[i + 1], wbuf[i + 2], wbuf[i + 3],
//						   wbuf[i + 4], wbuf[i + 5], wbuf[i + 6], wbuf[i + 7],
//						   wbuf[i + 8], wbuf[i + 9], wbuf[i + 10], wbuf[i + 11],
//						   wbuf[i + 12], wbuf[i + 13], wbuf[i + 14], wbuf[i + 15]);
//					for (int j = i; j < i + 16; j++)
//					{
//						printf("%c", isprint(wbuf[j]) ? wbuf[j] : '.');
//					}
//					printf("\n");
//				}
//
//				for (int i = 0; i < sector_sz; i += 16)
//				{
//					printf("rbuf %08x: "
//						   "%02x%02x%02x%02x "
//						   "%02x%02x%02x%02x "
//						   "%02x%02x%02x%02x "
//						   "%02x%02x%02x%02x ",
//						   addr + i,
//						   rbuf[i + 0], rbuf[i + 1], rbuf[i + 2], rbuf[i + 3],
//						   rbuf[i + 4], rbuf[i + 5], rbuf[i + 6], rbuf[i + 7],
//						   rbuf[i + 8], rbuf[i + 9], rbuf[i + 10], rbuf[i + 11],
//						   rbuf[i + 12], rbuf[i + 13], rbuf[i + 14], rbuf[i + 15]);
//					for (int j = i; j < i + 16; j++)
//					{
//						printf("%c", isprint(rbuf[j]) ? rbuf[j] : '.');
//					}
//					printf("\n");
//				}
//				good = false;
//				break;
//			}
//		}
//
//		if (!good)
//		{
//			break;
//		}
//	}
//
//    struct timeval end;
//    gettimeofday(&end, NULL);
//
//    struct timeval elapsed;
//    timersub(&end, &start, &elapsed);
//
//    float ms = elapsed.tv_sec * 1000.0 + elapsed.tv_usec;
//
//	if (addr >= cap)
//	{
//		printf("erase/write/verify successfully tested %d sectors\n", sectors);
//	}
//
//	// Test unaligned write
//	memset(wbuf, 2, sector_sz);
//	wbuf[0] = 1;
//	wbuf[sector_sz - 1] = 3;
//
//	nand_flash.erase_sector(0);
//	nand_flash.erase_sector(1);
//	nand_flash.write(1, wbuf, sector_sz);
//	memset(wbuf, 0, sector_sz);
//	nand_flash.read(0, wbuf, sector_sz);
//	vTaskDelay(1000 / portTICK_PERIOD_MS);
//	nand_flash.read(sector_sz, rbuf, sector_sz);
//	printf("address: 0x%x \n\r", sector_sz);
//	if (wbuf[0] != 0xff ||
//		wbuf[1] != 1 ||
//		wbuf[2] != 2 ||
//		wbuf[sector_sz - 1] != 2 ||
//		rbuf[0] != 3 ||
//		rbuf[1] != 0xff)
//	{
//		printf("               unaliged write test failed!\n");
//	}
//	else
//	{
//		printf("               unaliged write test successful!\n");
//	}
//
//	heap_caps_free(wbuf);
//	heap_caps_free(rbuf);
//
////	nand_flash.term();
}

void page_test(){

    printf("Read-Write page flash test started\n\r");

	int cap = nand_flash.chip_size();
	int page_sz = 2048;
	int step = 2048;
	int block = 0;
	int addr;

//	uint8_t *rbuf = (uint8_t *) malloc(page_sz);
//	if(rbuf != NULL){
//		ESP_LOGI(TAG, "Memory %d KB for rbuf allocated at %p!", page_sz/1024, (uint32_t*)rbuf);
//	} else ESP_LOGE(TAG, "Memory for rbuf didn't allocated!");
//
//	uint8_t *wbuf = (uint8_t *) malloc(page_sz);
//	if(wbuf != NULL){
//		ESP_LOGI(TAG, "Memory %d KB for wbuf allocated at %p!", page_sz/1024, (uint32_t*)wbuf);
//	} else ESP_LOGE(TAG, "Memory for wbuf didn't allocated!");

	static uint8_t rbuf[2048];
	static uint8_t wbuf[2048];

	uint8_t var = 0;
	for (int i = 0; i < page_sz; i++)
	{
		var++;
		if(i % 255 == 0) {
			var = 0;
		}
		wbuf[i] = var & 0xff;
		rbuf[i] = 0;
	}

	struct timeval start_wr;
	struct timeval end_wr;
	struct timeval elapsed_wr;

	struct timeval start_rd;
	struct timeval end_rd;
	struct timeval elapsed_rd;

	for (addr = 0; addr < cap; addr += step)
	{

	    gettimeofday(&start_wr, NULL);

		if(addr % EXT_FLASH_BLOCK_SIZE == 0){
			if(addr != 0) block++;
			printf("\n\r");
			printf("0x%x: block: %d page: %d\n\r", addr, addr/EXT_FLASH_BLOCK_SIZE, addr/EXT_FLASH_PAGE_SIZE);
			nand_flash.erase_sector(addr / EXT_FLASH_BLOCK_SIZE);
//
		}

		memset(rbuf, 0, page_sz);

		nand_flash.write(addr, wbuf, page_sz);

		gettimeofday(&end_wr, NULL);
	    timersub(&end_wr, &start_wr, &elapsed_wr);

	    gettimeofday(&start_rd, NULL);
		nand_flash.read(addr, rbuf, page_sz);
		gettimeofday(&end_rd, NULL);
	    timersub(&end_rd, &start_rd, &elapsed_rd);

	    float ms_wr = elapsed_wr.tv_sec * 1000000.0 + elapsed_wr.tv_usec;
	    float mbs_wr = (((page_sz * 1) / ms_wr) * 1000000.0) / 1048576.0;

	    float ms_rd = elapsed_rd.tv_sec * 1000000.0 + elapsed_rd.tv_usec;
	    float mbs_rd = (((page_sz * 1) / ms_rd) * 1000000.0) / 1048576.0;

	    printf("%f MB/s %f MB/s \n\r",mbs_wr ,mbs_rd);


		bool good = true;
		for (int b = 0; b < page_sz; b++)
		{
			if (wbuf[b] != rbuf[b])
			{
				printf("erase/write/verify failed at block %d offset %d\n", addr / page_sz, b);
				for (int i = 0; i < page_sz; i += 16)
				{
					printf("wbuf %08x: "
						   "%02x%02x%02x%02x "
						   "%02x%02x%02x%02x "
						   "%02x%02x%02x%02x "
						   "%02x%02x%02x%02x ",
						   addr + i,
						   wbuf[i + 0], wbuf[i + 1], wbuf[i + 2], wbuf[i + 3],
						   wbuf[i + 4], wbuf[i + 5], wbuf[i + 6], wbuf[i + 7],
						   wbuf[i + 8], wbuf[i + 9], wbuf[i + 10], wbuf[i + 11],
						   wbuf[i + 12], wbuf[i + 13], wbuf[i + 14], wbuf[i + 15]);
					for (int j = i; j < i + 16; j++)
					{
						printf("%c", isprint(wbuf[j]) ? wbuf[j] : '.');
					}
					printf("\n");
				}

				printf("\n\r");

				for (int i = 0; i < page_sz; i += 16)
				{
					printf("rbuf %08x: "
						   "%02x%02x%02x%02x "
						   "%02x%02x%02x%02x "
						   "%02x%02x%02x%02x "
						   "%02x%02x%02x%02x ",
						   addr + i,
						   rbuf[i + 0], rbuf[i + 1], rbuf[i + 2], rbuf[i + 3],
						   rbuf[i + 4], rbuf[i + 5], rbuf[i + 6], rbuf[i + 7],
						   rbuf[i + 8], rbuf[i + 9], rbuf[i + 10], rbuf[i + 11],
						   rbuf[i + 12], rbuf[i + 13], rbuf[i + 14], rbuf[i + 15]);
					for (int j = i; j < i + 16; j++)
					{
						printf("%c", isprint(rbuf[j]) ? rbuf[j] : '.');
					}
					printf("\n");
				}
				good = false;
				break;
			}
		}

		if (!good)
		{
			break;
		}
	}



	if (addr >= cap)
	{
		printf("erase/write/verify successfully tested %d sectors\n", block);
	}

//	free(wbuf);
//	free(rbuf);
}

void print_one_page(uint32_t addr){
//	uint8_t *rbuf = (uint8_t *) heap_caps_malloc(2048, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
//	if(rbuf != NULL){
//		ESP_LOGI(TAG, "Memory %d KB for rbuf allocated at %p!", 2048/1024, (uint32_t*)rbuf);
//	} else ESP_LOGE(TAG, "Memory for rbuf didn't allocated!");
//	memset(rbuf, 0, 2048);
//	nand_flash.read(addr, rbuf, 2048);
//
//	printf("block: %d, page: %d \n\r\n\r", addr/nand_flash.sector_size(), (addr/nand_flash.sector_size())/64);
//
//    for (int i = 0; i < 2048; i += 16)
//    {
//
//        printf("rbuf %08x: "
//               "%02x%02x%02x%02x "
//               "%02x%02x%02x%02x "
//               "%02x%02x%02x%02x "
//               "%02x%02x%02x%02x ",
//               addr + i,
//			   rbuf[i + 0], rbuf[i + 1], rbuf[i + 2], rbuf[i + 3],
//			   rbuf[i + 4], rbuf[i + 5], rbuf[i + 6], rbuf[i + 7],
//			   rbuf[i + 8], rbuf[i + 9], rbuf[i + 10], rbuf[i + 11],
//			   rbuf[i + 12], rbuf[i + 13], rbuf[i + 14], rbuf[i + 15]);
//        for (int j = i; j < i + 16; j++)
//        {
//            printf("%c", isprint(rbuf[j]) ? rbuf[j] : '.');
//        }
//        printf("\n");
//    }
//    heap_caps_free(rbuf);
}

#else
static void test_extflash_setup()
{
#if !defined(CONFIG_LITTLEFS_PARTITION_LABEL)
    ext_flash_config_t ext_cfg =
    {
        .vspi = false, // We are using HSPI host
		.cs_io_num = GPIO_NUM_4,
        .sck_io_num = PIN_NUM_CLK,
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .ss_io_num = -1,
        .hd_io_num = -1,
        .wp_io_num = -1,
        .speed_mhz = 20,
        .dma_channel = 1,
        .queue_size = 7,
        .max_dma_size = 8192,
        .sector_size = 0,
        .capacity = 0,
    };
    ext_flash_config_t *config = &ext_cfg;
    TST(nand_flash.init(config) == ESP_OK, "ExtFlash initialization failed");
#endif
}

static void test_extflash_teardown()
{
#if !defined(CONFIG_LITTLEFS_PARTITION_LABEL)
	nand_flash.term();
#endif
}

static void test_littleflash_setup(int openfiles)
{
    const little_flash_config_t little_cfg =
    {
#if !defined(CONFIG_LITTLEFS_PARTITION_LABEL)
        .flash = &nand_flash,
        .part_label = NULL,
#else
        .flash = NULL,
        .part_label = CONFIG_LITTLEFS_PARTITION_LABEL,
#endif
        .base_path = MOUNT_POINT,
        .open_files = OPENFILES,
        .auto_format = true,
        .lookahead = 32
    };

    littleflash.init(&little_cfg);


    TST(littleflash.init(&little_cfg) == ESP_OK, "LittleFlash initialization failed");
}

static void test_littleflash_teardown()
{
    littleflash.term();
}

static void test_format()
{
#if !defined(CONFIG_LITTLEFS_PARTITION_LABEL)
    test_extflash_setup();
    nand_flash.erase_sector(0);
    test_extflash_teardown();
#else
    const esp_partition_t *part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                                           ESP_PARTITION_SUBTYPE_ANY,
                                                           CONFIG_LITTLEFS_PARTITION_LABEL);
    TEST_ASSERT_NOT_NULL(part);
    TEST_ASSERT_EQUAL(esp_partition_erase_range(part, 0, SPI_FLASH_SEC_SIZE), ESP_OK);
#endif
}

static void test_setup(int openfiles)
{
    test_extflash_setup();
    test_littleflash_setup(openfiles);
}

static void test_teardown()
{
    test_littleflash_teardown();
    test_extflash_teardown();
}

//
// Tests shamelessly copied from "esp-idf/components/fatfs/test" and modified
//
TEST_CASE(can_format, "can format chip", "[fatfs][wear_levelling]")
{
    test_format();
    test_setup(OPENFILES);
    test_teardown();
}

TEST_CASE(can_create_write, "can create and write file", "[fatfs][wear_levelling]")
{
    test_setup(OPENFILES);
    test_lfs_create_file_with_text(MOUNT_POINT "/hello.txt", lfs_test_hello_str);
    test_teardown();
}

TEST_CASE(can_read, "can read file", "[fatfs][wear_levelling]")
{
    test_setup(OPENFILES);
    test_lfs_create_file_with_text(MOUNT_POINT "/hello.txt", lfs_test_hello_str);
    test_lfs_read_file(MOUNT_POINT "/hello.txt");
    test_teardown();
}

TEST_CASE(can_open_max, "can open maximum number of files", "[fatfs][wear_levelling]")
{
    int max_files = FOPEN_MAX - 3; /* account for stdin, stdout, stderr */
    max_files = OPENFILES;
    test_setup(max_files);
    test_lfs_open_max_files(MOUNT_POINT "/f", max_files);
    test_teardown();
}

TEST_CASE(can_overwrite_append, "overwrite and append file", "[fatfs][wear_levelling]")
{
    test_setup(OPENFILES);
    test_lfs_overwrite_append(MOUNT_POINT "/hello.txt");
    test_teardown();
}

TEST_CASE(can_lseek, "can lseek", "[fatfs][wear_levelling]")
{
    test_setup(OPENFILES);
    test_lfs_lseek(MOUNT_POINT "/seek.txt");
    test_teardown();
}

TEST_CASE(can_stat, "stat returns correct values", "[fatfs][wear_levelling]")
{
    test_setup(OPENFILES);
    test_lfs_stat(MOUNT_POINT "/stat.txt", MOUNT_POINT "");
    test_teardown();
}

TEST_CASE(can_unlink, "unlink removes a file", "[fatfs][wear_levelling]")
{
    test_setup(OPENFILES);
    test_lfs_unlink(MOUNT_POINT "/unlink.txt");
    test_teardown();
}

TEST_CASE(can_rename, "rename moves a file", "[fatfs][wear_levelling]")
{
    test_setup(OPENFILES);
    test_lfs_rename(MOUNT_POINT "/link");
    test_teardown();
}

TEST_CASE(can_create_remove, "can create and remove directories", "[fatfs][wear_levelling]")
{
    test_setup(OPENFILES);
    test_lfs_mkdir_rmdir(MOUNT_POINT "/dir");
    test_teardown();
}

TEST_CASE(can_open_root, "can opendir root directory of FS", "[fatfs][wear_levelling]")
{
    test_setup(OPENFILES);
    test_lfs_can_opendir(MOUNT_POINT "");
    test_teardown();
}

TEST_CASE(can_dir, "opendir, readdir, rewinddir, seekdir work as expected", "[fatfs][wear_levelling]")
{
    test_setup(OPENFILES);
    test_lfs_opendir_readdir_rewinddir(MOUNT_POINT "/dir");
    test_teardown();
}

TEST_CASE(can_task, "multiple tasks can use same volume", "[fatfs][wear_levelling]")
{
    test_setup(OPENFILES);
    test_lfs_concurrent(MOUNT_POINT "/f");
    test_teardown();
}

TEST_CASE(can_read_write, "write/read speed test", "[fatfs][wear_levelling]")
{
    /* Erase partition before running the test to get consistent results */
    test_format();

    test_setup(OPENFILES);

    const size_t buf_size = 16 * 1024;
    uint32_t* buf = (uint32_t*) calloc(1, buf_size);
    for (size_t i = 0; i < buf_size / 4; ++i) {
        buf[i] = esp_random();
    }
    const size_t file_size = 256 * 1024;
    const char* file = MOUNT_POINT "/256k.bin";

    test_lfs_rw_speed(file, buf, 4 * 1024, file_size, true);
    test_lfs_rw_speed(file, buf, 8 * 1024, file_size, true);
    test_lfs_rw_speed(file, buf, 16 * 1024, file_size, true);

    test_lfs_rw_speed(file, buf, 4 * 1024, file_size, false);
    test_lfs_rw_speed(file, buf, 8 * 1024, file_size, false);
    test_lfs_rw_speed(file, buf, 16 * 1024, file_size, false);

    unlink(file);

    free(buf);
    test_teardown();
}

void app_main(void)
{
//	esp_log_level_set(TAG, ESP_LOG_INFO);
	print_free_memory();

    can_format();
    can_create_write();
    can_read();
    can_open_max();
    can_overwrite_append();
    can_lseek();
    can_stat();
    can_unlink();
    can_rename();
    can_create_remove();
    can_open_root();
    can_dir();
    can_task();
    can_read_write();

    printf("All tests done...\n");

    print_free_memory();

    vTaskDelay(portMAX_DELAY);
}
#endif // UNIT_TESTING





