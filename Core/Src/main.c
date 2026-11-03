/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_frame.h"
#include "can_ring_buffer.h"
#include "isotp.h"
#include "uds_types.h"
#include "uds_server.h"
#include "uds_session.h"
#include "dtc_table_85.h"
#include "dtc_store.h"
#include "dtc_nvm_adapter.h"
#include "uds_wear_leveling.h"
#include "uds_platform_time.h"



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

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

can_ring_buffer_t queue;

static uint8_t rx_buf[ISOTP_MAX_LEN];
static isotp_link_t diag;
static const uds_app_config_t s_app_cfg = {NULL};

/* ── function definition ────────────────── */

static int can_tx(void *user, uint32_t id, const uint8_t *data, uint8_t len)
{
    return CAN_TX(id, data, len) == HAL_OK ? 0 : -1; /* -1 = mailbox full, retried later */
}


void diag_init(void)
{
    isotp_config_t cfg = {
        .tx_id = ISOTP_TX_ID, .rx_id = ISOTP_RX_ID,
        .send = can_tx,
        .rx_buf = rx_buf, .rx_cap = sizeof(rx_buf),
        .block_size = 8, .st_min = 0x00, .padding = 0xCC,
    };
    isotp_init(&diag, &cfg);
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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  CAN_Handler_Init();

  can_ring_buffer_init(&queue);
  diag_init();
  uds_server_init(&s_app_cfg);
  dtc_table_85_init();
  dtc_nvm_init();

  printf("stm32 power on reset!\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/* main loop: drain, then poll */
	can_frame_t f;
	while (can_ring_buffer_pop(&queue, &f))
	{
		if (f.id == ISOTP_RX_ID || f.id == ISOTP_FUNC_RX_ID)
		{
			isotp_on_frame(&diag, f.data, f.dlc, uds_platform_time_us());
		}
	}
	isotp_poll(&diag, uds_platform_time_us());
    /* Read the RX status before isotp_rx_take(), which clears it. */
    const isotp_status_t rs = isotp_rx_status(&diag);
    if (rs >= ISOTP_ERR_TIMEOUT_BS)
        printf("rx error: %s", isotp_status_str(rs));

	const int n = isotp_rx_take(&diag);
	if (n > 0)
	{
		static uds_response_t resp;
		bool send = uds_server_process(rx_buf, (uint16_t)n, false, &resp);

		if (send) 
		{
			isotp_status_t st = isotp_send(&diag, resp.data, resp.len, uds_platform_time_us());
			if (st != ISOTP_BUSY && st != ISOTP_DONE)
			{
				printf("isotp_send: %s\r\n", isotp_status_str(st));
			}
		}
	}
	uds_server_poll();

	
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* Receive FIFO 0 message pending interrupt management */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	can_frame_t f = {0};
    CAN_RxHeaderTypeDef hdr;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, f.data) == HAL_OK)
    {
    	f.id    = hdr.StdId;
		f.dlc   = hdr.DLC;
		f.is_ext = false;
		f.is_rtr = false;
		f.is_fd  = false;
		(void)can_ring_buffer_push(&queue, &f);
    }

}

/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
