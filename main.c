/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GCTask */
osThreadId_t GCTaskHandle;
const osThreadAttr_t GCTask_attributes = {
  .name = "GCTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
/*
#define PAGES_PER_BLOCK 4
#define TOTAL_BLOCKS 16
#define PAGE_SIZE 512

typedef struct {
    bool page_status[PAGES_PER_BLOCK];
    int valid_page_count;
    bool is_erased;
    int lba_mapping[PAGES_PER_BLOCK];
    uint8_t data_content[PAGES_PER_BLOCK][PAGE_SIZE];
} BlockStatus;

BlockStatus flash[TOTAL_BLOCKS];
int L2P_Table[64];

int free_block_list[TOTAL_BLOCKS];
int fb_head = 0, fb_tail = 0, fb_count = 0;

int curr_block = -1;
int curr_page = 0;
*/
#define NUM_CHANNELS    2
#define BLOCKS_PER_CH   5
#define PAGES_PER_BLOCK 4
#define PAGE_SIZE       512
#define TOTAL_LBA       20
#define TOTAL_BLOCKS    10

typedef struct {
    bool page_status[PAGES_PER_BLOCK];
    int valid_page_count;
    bool is_erased;
    int lba_mapping[PAGES_PER_BLOCK];
    uint8_t data_content[PAGES_PER_BLOCK][PAGE_SIZE];
} BlockStatus;

typedef struct {
    BlockStatus flash[BLOCKS_PER_CH];
    int free_block_list[BLOCKS_PER_CH];
    int fb_head, fb_tail, fb_count;
    int curr_block;
    int curr_page;
} Channel;

Channel channels[NUM_CHANNELS];

int L2P_Table[TOTAL_LBA]; // physical address = (Channel << 16) | (Block << 8) | Page

extern DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
volatile bool dma_transfer_complete = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartGCTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

UART_HandleTypeDef huart2;

int __io_putchar(int ch) {
    ITM_SendChar(ch);
    return ch;
}

void HAL_DMA_XferCpltCallback(DMA_HandleTypeDef *_hdma) {
    dma_transfer_complete = true;
}

void move_page_with_dma(uint32_t src_addr, uint32_t dest_addr, uint32_t size) {
	/*
    dma_transfer_complete = false;

    HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream0, src_addr, dest_addr, size/4);

    while(!dma_transfer_complete);
    */

	/*
	memcpy((void*)dest_addr, (void*)src_addr, size);
	dma_transfer_complete = true;
	*/

	dma_transfer_complete = false;

	if (HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream0, src_addr, dest_addr, size/4) != HAL_OK) {
	    printf("  [DMA Error] Start failed!\n");
	    return;
	}

	uint32_t timeout = 100000;
	while(!dma_transfer_complete && timeout--);

	if (timeout == 0) {
	    printf("  [DMA Timeout] Interrupt not received!\n");
	}
}

int get_channel_index(int lba) {
    return lba % NUM_CHANNELS; // LBA 0->Ch0, LBA 1->Ch1, LBA 4->Ch0
}

int encode_addr(int ch, int blk, int pg) {
    return (ch << 16) | (blk << 8) | pg;
}

void decode_addr(int addr, int *ch, int *blk, int *pg) {
    if (addr == -1) {
        *ch = -1; *blk = -1; *pg = -1;
        return;
    }
    *ch  = (addr >> 16) & 0xFF;
    *blk = (addr >> 8)  & 0xFF;
    *pg  = addr & 0xFF;
}

/*
void push_free_block(int b_idx) {
    free_block_list[fb_tail] = b_idx;
    fb_tail = (fb_tail + 1) % TOTAL_BLOCKS;
    fb_count++;
}

int pop_free_block() {
    if (fb_count == 0) return -1;
    int b_idx = free_block_list[fb_head];
    fb_head = (fb_head + 1) % TOTAL_BLOCKS;
    fb_count--;
    return b_idx;
}
*/

void push_free_block(int ch_idx, int b_idx) {
	Channel *ch = &channels[ch_idx];

	ch->free_block_list[ch->fb_tail] = b_idx;
	ch->fb_tail = (ch->fb_tail + 1) % BLOCKS_PER_CH;
	ch->fb_count++;
}

int pop_free_block(int ch_idx) {
    Channel *ch = &channels[ch_idx];
    if (ch->fb_count <= 0) return -1;

    int b_idx = ch->free_block_list[ch->fb_head];
    ch->fb_head = (ch->fb_head + 1) % BLOCKS_PER_CH;
    ch->fb_count--;
    return b_idx;
}

/*
void init_ssd() {
	printf("FTL Start!\r\n");
    for (int i = 0; i < 64; i++) L2P_Table[i] = -1;
    for (int i = 0; i < TOTAL_BLOCKS; i++) {
        flash[i].is_erased = true;
        flash[i].valid_page_count = 0;

        for (int p = 0; p < PAGES_PER_BLOCK; p++) {
            flash[i].page_status[p] = false;
            flash[i].lba_mapping[p] = -1;

            memset(flash[i].data_content[p], 0xFF, PAGE_SIZE);
        }

        push_free_block(i);
    }
}
*/

void init_ssd() {
	for (int i = 0; i < TOTAL_LBA; i++) L2P_Table[i] = -1;
    for (int i = 0; i < NUM_CHANNELS; i++) {
        channels[i].fb_head = 0;
        channels[i].fb_tail = 0;
        channels[i].fb_count = 0;
        channels[i].curr_block = -1;
        channels[i].curr_page = 0;

        // Block 0~4 -> Ch0, 5~9 -> Ch1...
        for (int j = 0; j < BLOCKS_PER_CH; j++) {
            push_free_block(i, j);
            channels[i].flash[j].is_erased = true;
            channels[i].flash[j].valid_page_count = 0;
        }
    }
}

/*
int get_next_write_addr(int *curr_b, int *curr_p) {
    if (*curr_b == -1 || *curr_p >= PAGES_PER_BLOCK) {
        *curr_b = pop_free_block();
        *curr_p = 0;
        if (*curr_b == -1) return -1;
    }
    return (*curr_b) * PAGES_PER_BLOCK + (*curr_p)++;
}
*/

int get_next_write_addr(int ch_idx) {
    Channel *ch = &channels[ch_idx];

    if (ch->curr_block == -1 || ch->curr_page >= PAGES_PER_BLOCK) {

        int new_blk = pop_free_block(ch_idx);

        if (new_blk == -1) {
            return -1;
        }

        ch->curr_block = new_blk;
        ch->curr_page = 0;

        ch->flash[new_blk].is_erased = false;
        ch->flash[new_blk].valid_page_count = 0;
        for(int i = 0; i < PAGES_PER_BLOCK; i++) {
            ch->flash[new_blk].page_status[i] = false;
        }
    }

    int addr = encode_addr(ch_idx, ch->curr_block, ch->curr_page);

    ch->curr_page++;

    return addr;
}

/*
void invalidate_old_data(int old_physical_addr) {
    if (old_physical_addr == -1) return;

    int b_idx = old_physical_addr / PAGES_PER_BLOCK;
    int p_idx = old_physical_addr % PAGES_PER_BLOCK;

    if (flash[b_idx].page_status[p_idx]) {
        flash[b_idx].page_status[p_idx] = false;
        flash[b_idx].valid_page_count--;
        printf("  [L2P] Invalidate Old Addr: %d. Block %d Valid Pages left: %d\n",
                old_physical_addr, b_idx, flash[b_idx].valid_page_count);
    }
}
*/

void invalidate_old_data(int old_physical_addr) {
    if (old_physical_addr == -1) return;

    int ch_idx, b_idx, p_idx;

    decode_addr(old_physical_addr, &ch_idx, &b_idx, &p_idx);

    if (ch_idx < 0 || ch_idx >= NUM_CHANNELS || b_idx >= BLOCKS_PER_CH) return; // check

    Channel *ch = &channels[ch_idx];
    BlockStatus *blk = &ch->flash[b_idx];

    if (blk->page_status[p_idx]) {
        blk->page_status[p_idx] = false;
        blk->valid_page_count--;

        printf("  [L2P] Invalidate Old Addr: %d (Ch %d, Blk %d, Pg %d). Valid Pages left: %d\n",
                old_physical_addr, ch_idx, b_idx, p_idx, blk->valid_page_count);
    }
}

/*
void run_gc(int *curr_b, int *curr_p) {
    int victim = -1;
    int min_valid = PAGES_PER_BLOCK;

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);


    // for (int i = 0; i < TOTAL_BLOCKS; i++) {
        // if (!flash[i].is_erased && i != *curr_b && flash[i].valid_page_count < min_valid) {
            // min_valid = flash[i].valid_page_count;
            // victim = i;
        // }
    // }


    for (int i = 0; i < TOTAL_BLOCKS; i++) {
        if (!flash[i].is_erased && i != *curr_b && flash[i].valid_page_count > 0) {
            if (flash[i].valid_page_count < min_valid) {
                min_valid = flash[i].valid_page_count;
                victim = i;
            }
        }
    }

    if (min_valid > 2) {
        printf("  [GC Skip] No block with fewer than 3 valid pages. Skip GC.\n");
        victim = -1;
    }

    if (victim != -1) {
    	printf("\n[GC] Triggered! Victim: Block %d, Migrating %d valid pages...\n", victim, min_valid);

        for (int p = 0; p < PAGES_PER_BLOCK; p++) {
            if (flash[victim].page_status[p]) {
                int lba = flash[victim].lba_mapping[p];

                int new_addr = get_next_write_addr(curr_b, curr_p);
                int new_b = new_addr / PAGES_PER_BLOCK;
                int new_p = new_addr % PAGES_PER_BLOCK;

                move_page_with_dma((uint32_t)&flash[victim].data_content[p][0],
                                (uint32_t)&flash[new_b].data_content[new_p][0],
                                PAGE_SIZE);

                L2P_Table[lba] = new_addr;

                printf("  [MOVE] LBA %d: Addr %d -> %d\n", lba, (victim * 4 + p), new_addr);
            }
        }


        flash[victim].valid_page_count = 0;
        flash[victim].is_erased = true;
        for (int p = 0; p < PAGES_PER_BLOCK; p++) {
            flash[victim].page_status[p] = false;
            memset(flash[victim].data_content[p], 0xFF, PAGE_SIZE);
        }

        push_free_block(victim);
        printf("[GC] Block %d Erased & Recycled to Free List.\n\n", victim);
        HAL_Delay(500);
    }
    else {
        printf("  [GC Skip] No suitable victim found.\n");
    }

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
}
*/

void run_gc_specific_channel(int ch_idx) {
    int victim = -1;
    int min_valid = PAGES_PER_BLOCK;
    Channel *ch = &channels[ch_idx];

    uint16_t gc_led = (ch_idx == 0) ? GPIO_PIN_14 : GPIO_PIN_15; // Ch0 right, Ch1 bottom
        HAL_GPIO_WritePin(GPIOD, gc_led, GPIO_PIN_SET);

    for (int i = 0; i < BLOCKS_PER_CH; i++) {
        if (!ch->flash[i].is_erased && i != ch->curr_block && ch->flash[i].valid_page_count > 0) {
            if (ch->flash[i].valid_page_count < min_valid) {
                min_valid = ch->flash[i].valid_page_count;
                victim = i;
            }
        }
    }

    // Greedy Policy
    /*
    if (victim != -1 && min_valid > 2) {
        printf("  [Ch %d GC Skip] No block with fewer than 3 valid pages.\n", ch_idx);
        victim = -1;
    }
    */

    if (victim != -1) {
        printf("\n[Ch %d GC] Triggered! Victim: Block %d, Migrating %d valid pages...\n",
                ch_idx, victim, min_valid);

        for (int p = 0; p < PAGES_PER_BLOCK; p++) {
            if (ch->flash[victim].page_status[p]) {
                int lba = ch->flash[victim].lba_mapping[p];

                int new_addr = get_next_write_addr(ch_idx);

                if (new_addr != -1) {
                    int n_ch, n_blk, n_pg;
                    decode_addr(new_addr, &n_ch, &n_blk, &n_pg);

                    move_page_with_dma((uint32_t)&ch->flash[victim].data_content[p][0],
                                    (uint32_t)&channels[n_ch].flash[n_blk].data_content[n_pg][0],
                                    PAGE_SIZE);

                    channels[n_ch].flash[n_blk].page_status[n_pg] = true;
                    channels[n_ch].flash[n_blk].lba_mapping[n_pg] = lba;
                    channels[n_ch].flash[n_blk].valid_page_count++;
                    L2P_Table[lba] = new_addr;

                    printf("  [MOVE] LBA %d: Ch%d Blk%d Pg%d -> Ch%d Blk%d Pg%d\n",
                            lba, ch_idx, victim, p, n_ch, n_blk, n_pg);
                }
            }
        }

        ch->flash[victim].valid_page_count = 0;
        ch->flash[victim].is_erased = true;
        for (int p = 0; p < PAGES_PER_BLOCK; p++) {
            ch->flash[victim].page_status[p] = false;
            memset(ch->flash[victim].data_content[p], 0xFF, PAGE_SIZE);
        }

        push_free_block(ch_idx, victim);
        printf("[Ch %d GC] Block %d Erased & Recycled.\n\n", ch_idx, victim);
        HAL_Delay(500);
    } else {
        printf("  [Ch %d GC Skip] No suitable victim found.\n", ch_idx);
    }

    osDelay(100);
    HAL_GPIO_WritePin(GPIOD, gc_led, GPIO_PIN_RESET);
}

/*
void host_write(int lba, uint8_t *input_data, int *curr_b, int *curr_p) {
	printf("Host Write Request: LBA %d...\n", lba);

    invalidate_old_data(L2P_Table[lba]);

    if (fb_count < 2) {
    	printf("\n[WARNING] Free blocks below threshold! Pre-launching GC...\n");
        run_gc(curr_b, curr_p);
    }

    int new_addr = get_next_write_addr(curr_b, curr_p);

    L2P_Table[lba] = new_addr;

    int b_idx = new_addr / PAGES_PER_BLOCK;
    int p_idx = new_addr % PAGES_PER_BLOCK;
    flash[b_idx].page_status[p_idx] = true;
    flash[b_idx].lba_mapping[p_idx] = lba;
    flash[b_idx].valid_page_count++;
    flash[b_idx].is_erased = false;

    memcpy(flash[b_idx].data_content[p_idx], input_data, PAGE_SIZE);

    printf("  [L2P] LBA %d -> PhysAddr %d (Block %d, Page %d)\n", lba, new_addr, b_idx, p_idx);
}
*/

void host_write(int lba, uint8_t *input_data) {
    int ch_idx = lba % NUM_CHANNELS;
    Channel *ch = &channels[ch_idx];

    if (ch_idx == 0) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);   // left
    } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);   // top
    }

    printf("Host Write Request: LBA %d (Routing to Ch %d)...\n", lba, ch_idx);

    invalidate_old_data(L2P_Table[lba]);

    int new_addr = get_next_write_addr(ch_idx);

    if (new_addr != -1) {
        int c_idx, b_idx, p_idx;
        decode_addr(new_addr, &c_idx, &b_idx, &p_idx);

        ch->flash[b_idx].page_status[p_idx] = true;
        ch->flash[b_idx].lba_mapping[p_idx] = lba;
        ch->flash[b_idx].valid_page_count++;
        ch->flash[b_idx].is_erased = false;

        // change move_page_with_dma
        memcpy(ch->flash[b_idx].data_content[p_idx], input_data, PAGE_SIZE);

        L2P_Table[lba] = new_addr;

        printf("  [L2P] LBA %d -> EncAddr %d (Ch %d, Blk %d, Page %d)\n",
                lba, new_addr, c_idx, b_idx, p_idx);
    } else {
        printf("  [ERROR] Channel %d is completely full! Write failed.\n", ch_idx);
    }

    osDelay(10);
    if (ch_idx == 0) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  init_ssd();
  printf("--- SSD Simulation System Ready ---\r\n");
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of GCTask */
  GCTaskHandle = osThreadNew(StartGCTask, NULL, &GCTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /*
  uint8_t my_test_data[PAGE_SIZE];
  static int lba_counter = 0;
  */

  while (1)
  {
	  /*
	  memset(my_test_data, 0xAA, PAGE_SIZE);
	  my_test_data[0] = (uint8_t)(lba_counter % 64);

	      host_write(lba_counter % 64, my_test_data);

	      lba_counter++;

	      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

	      HAL_Delay(1000);
	  */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream0
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
  hdma_memtomem_dma2_stream0.Instance = DMA2_Stream0;
  hdma_memtomem_dma2_stream0.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream0.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_memtomem_dma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_memtomem_dma2_stream0.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream0.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_memtomem_dma2_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream0) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  static uint8_t my_test_data[PAGE_SIZE];
	  static int lba_counter = 0;

	  memset(my_test_data, 0xAA, PAGE_SIZE);
	  my_test_data[0] = (uint8_t)(lba_counter % 20);

	  host_write(lba_counter % 20, my_test_data);

	  lba_counter++;
	  osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartGCTask */
/**
* @brief Function implementing the GCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGCTask */
void StartGCTask(void *argument)
{
  /* USER CODE BEGIN StartGCTask */
	printf("RTOS: Background GC Engine Standby.\n");
  /* Infinite loop */
  for(;;)
  {
	  for(int i = 0; i < NUM_CHANNELS; i++) {
	      if (channels[i].fb_count < 2) {
	          printf("\n[RTOS GC] Background Monitoring: Ch %d is low on space!\n", i);
	          run_gc_specific_channel(i);
	      }
	  }

	  osDelay(100);
  }
  /* USER CODE END StartGCTask */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
