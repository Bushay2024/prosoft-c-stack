/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    char command[8];    // Буфер для хранения команды ("on"/"off")
    uint8_t led_num;    // Номер светодиода (1-3)
} uart_command_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_NUMBER 3
#define LED1_PIN GPIO_PIN_0
#define LED2_PIN GPIO_PIN_7
#define LED3_PIN GPIO_PIN_14
#define LED_PORT GPIOB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
osMessageQueueId_t uart_queue;
osThreadId_t uart_rx_task_id;
osThreadId_t cmd_process_task_id;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void UART_RX_Task(void *argument);
void CMD_Process_Task(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

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
  /* Create queue */
  // Создание очереди сообщений (16 элементов типа uart_command_t)
  uart_queue = osMessageQueueNew(16, sizeof(uart_command_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  /* USER CODE BEGIN RTOS_THREADS */
  // Задача приема UART (стек 1024 байта)
  const osThreadAttr_t uart_rx_attr = {
      .name = "UART_RX_Task",
      .stack_size = 1024
  };
  uart_rx_task_id = osThreadNew(UART_RX_Task, NULL, &uart_rx_attr);

  // Задача обработки команд (стек 1024 байта)
  const osThreadAttr_t cmd_process_attr = {
      .name = "CMD_Process_Task",
      .stack_size = 1024
  };
  cmd_process_task_id = osThreadNew(CMD_Process_Task, NULL, &cmd_process_attr);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_PIN|LED2_PIN|LED3_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pins */
  GPIO_InitStruct.Pin = LED1_PIN|LED2_PIN|LED3_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
//Задача приема UART
void UART_RX_Task(void *argument) {
    uint8_t rx_buffer[32]; // Буфер для приема команды
    uint8_t rx_char;
    uint8_t idx = 0;
    uint8_t cmd_received = 0;

    while (1) {
        if (HAL_UART_Receive(&huart3, &rx_char, 1, osWaitForever) == HAL_OK) {
        	// Конец команды - символ \r
            if (rx_char == '\r') {
                rx_buffer[idx] = '\0';
                cmd_received = 1;
            // Иначе добавляем символ в буфер
            } else if (idx < sizeof(rx_buffer) - 1) {
                rx_buffer[idx++] = rx_char;
            }
            // Если команда полностью принята
            if (cmd_received) {
            	// Разбиваем строку на токены (команда и номер светодиода)
                char *token = strtok((char *)rx_buffer, " ");
                uart_command_t cmd = {0};

                if (token) {
                    strncpy(cmd.command, token, sizeof(cmd.command) - 1);
                    token = strtok(NULL, " ");

                    if (token) {
                        cmd.led_num = atoi(token);

                        // Проверка на дополнительные аргументы
                        if (strtok(NULL, " ") != NULL) {
                            cmd.led_num = 0; // Помечаем как ошибку
                        }
                    }

                    // Отправка команды на обработку
                    osMessageQueuePut(uart_queue, &cmd, 0, osWaitForever);
                }

                // Сброс состояния для следующей команды
                idx = 0;
                cmd_received = 0;
            }
        }
    }
}

//Задача обработки команд
void CMD_Process_Task(void *argument) {
    uart_command_t cmd;
    uint8_t response[32];

    while (1) {
    	// Ожидание команды из очереди (блокирующий вызов)
        if (osMessageQueueGet(uart_queue, &cmd, NULL, osWaitForever) == osOK) {
            uint8_t valid = 1;

            // Проверка команды
            if (strcmp(cmd.command, "on") != 0 && strcmp(cmd.command, "off") != 0) {
                valid = 0;
            }

            // Проверка номера светодиода
            if (cmd.led_num < 1 || cmd.led_num > LED_NUMBER) {
                valid = 0;
            }

            if (valid) {
                // Выполнение команды
                GPIO_PinState state = (strcmp(cmd.command, "on") == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;

                switch (cmd.led_num) {
                    case 1: HAL_GPIO_WritePin(LED_PORT, LED1_PIN, state); break;
                    case 2: HAL_GPIO_WritePin(LED_PORT, LED2_PIN, state); break;
                    case 3: HAL_GPIO_WritePin(LED_PORT, LED3_PIN, state); break;
                }
                // Формируем ответ
                strcpy((char *)response, "ok\r\n");
            } else {
                strcpy((char *)response, "error invalid arguments\r\n");
            }
            // Отправляем ответ через UART
            HAL_UART_Transmit(&huart3, response, strlen((char *)response), HAL_MAX_DELAY);
        }
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
