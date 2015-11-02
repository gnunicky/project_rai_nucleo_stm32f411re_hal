/**
 * MCMRWM is a program written in C for STMicroelectronics NUCLEO F411RE 
 * microcontroller.
 * The software was produced as a part of an examination proof of Industrial
 * Automation Networks course held by the University of Catania, Italy,
 * academic year 2014-15.
 * The code, flashed on an STMIcroelectronics microcontroller, implements
 * basic, read/write routines of a magnetic core memory array and is a porting
 * of an existing project for Arduino I developed by Ben North and Oliver Nash.
 * 
 * Copyright (C) 2015 onwards Nicola Didomenico (nicola.didomenico@gmail.com)
 * Copyright (C) 2015 onwards Salvatore Del Popolo (popolo@tin.it)
 *
 * This program is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software 
 * Foundation, either version 3 of the License, or (at your option) any later 
 * version.
 * This program is distributed in the hope that it will be useful, but WITHOUT 
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS 
 * FOR A PARTICULAR PURPOSE.See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public Licens along with 
 * this program. If not, see <http://www.gnu.org/licenses/>.
**/

#include "main.h"
#include <string.h>

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#define		ADDRSIZE	5
#define		WORDSIZE	(1 << ADDRSIZE)

static GPIO_InitTypeDef  GPIO_InitStruct_A;
static GPIO_InitTypeDef  GPIO_InitStruct_B;
static GPIO_InitTypeDef  GPIO_InitStruct_C;
UART_HandleTypeDef UartHandle;
HAL_StatusTypeDef uartstatus;

static void SystemClock_Config(void);
static void Error_Handler(void);
void USART2_Configuration(void);
void GPIO_Configuration(void);

static void w_test(void);
static void r_test(void);
static void W_test(void);
static void R_test(void);
static void t_test(void);

int read_bit(const int n);
void write_bit(int n, const int v);
unsigned long read_word(void);
void write_word(unsigned long v);

void long_to_binary(unsigned long decimalNumber);
void address_to_binary(unsigned long decimalNumber);

void timing_enable(void);
void timing_delay_us(unsigned int tick);
volatile unsigned int *DWT_CYCCNT   = (volatile unsigned int *)0xE0001004; //address of the register
volatile unsigned int *DWT_CONTROL  = (volatile unsigned int *)0xE0001000; //address of the register
volatile unsigned int *SCB_DEMCR    = (volatile unsigned int *)0xE000EDFC; //address of the register

int option;
/* Variable used to identify the command inserted in the interactive menu. */

void address_to_binary(unsigned long decimalNumber)
{
/* The function prints the address of the selected read/written core into
   binary from. */

    unsigned long quotient;
    int binaryNumber[5],i=0,j;
    quotient = decimalNumber;
    while(quotient!=0){
         binaryNumber[i++]= quotient % 2;
         quotient = quotient / 2;
    }
    printf(" B"); 
    if (decimalNumber == 0 ) printf("00000");
    else
    { 
    for(j=i-1; j>=0; j--)
         printf("%d",binaryNumber[j]);
    }    
}

void long_to_binary(unsigned long decimalNumber)
{
/* The function prints the value contained or to be written in the Magnetic
   Core Memory into binary form. All 32 bits are displayed. */

    unsigned long quotient;
    int binaryNumber[WORDSIZE],i,j;
    for(i=0; i<WORDSIZE;i++)
         binaryNumber[i]=0;
    quotient = decimalNumber;
    i=0;
    while(quotient!=0){
         binaryNumber[i++]= quotient % 2;
         quotient = quotient / 2;
    }
    printf(" B"); 
    for(j = WORDSIZE-1; j>=0; j--)
         printf("%d",binaryNumber[j]);
    printf("\n\r");
}

void write_bit(int n, const int v)
{
/* This function writes 0 or 1 in the selected core of the Memory Array. It is
   a basic routine called from write_word(). */

 int q, r;

  if (v == 0)
  {
       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
  }
  else
  {
       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
  }
/* At this stage it is clear whether 0 or 1 is to be written in the selected
   core. */

  q = n / 2;
  r = n % 2;
  if (r == 1)
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
  else
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
/* This if-then structure and the following 4 are used to set the appropriate
   pin to address the selected core to be written. */ 

  n = q;
  q = n / 2;
  r = n % 2;
  if (r == 1)
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  else
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  
  n = q;
  q = n / 2;
  r = n % 2;
  if (r == 1)
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  else
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
  
  n = q;
  q = n / 2;
  r = n % 2;
  if (r == 1)
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
  else
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
  
  n = q;
  q = n / 2;
  r = n % 2;
  if (r == 1)
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  else
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); 
/* Now we are ready to write 0 or 1 in the selected core. */

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);  
  timing_delay_us(565);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
  timing_delay_us(565);
}

int read_bit(const int n)
{
/* This function reads the selected core of the Memory Array. It is a basic
   routine called from read_word(). */ 

  write_bit(n, 0);
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)==1) 
  {
    write_bit(n, 1);
    return 1;
   }
   else
   {
    return 0;
   }
}		

unsigned long read_word(void)
{
/* This piece of code reads the content of the entire Memory Arrary, all 32
   cores. */

 unsigned long v = 0;
 int n;
	for (n = WORDSIZE-1; n >= 0; n--)
	{
		v <<= 1;
		v |= read_bit(n);
	}
	return v;
}		

static void R_test()
{
/* This routine, calling read_word(), displays the content of the entire
   Memory Array. */

 unsigned long d = read_word();
 printf("\n\r Core data read: "); 
 printf(" 0x%lx",d); 
 printf(" =>"); 
 long_to_binary(d);  
}

void write_word(unsigned long v)
{
/* This routine, calling write_bit(), writes a value, max 32 bits, in the
   Memory Array. */

  int n;
  for(n = 0; n < WORDSIZE; n++)
  {
    write_bit(n, v & 1);
    v >>= 1;
  }
}

static void W_test()
{
/* The function, calling write_word(), Writes the desired value, max 32 bits,
   on the Memory Array. */

  char ch[9];
  int i=0;
  unsigned long d = 0;
  printf("\n\r Please enter 8 hexadecimal digits: \n\r"); 
  ch[8]='\0'; 
  uartstatus=HAL_UART_Receive(&UartHandle, (uint8_t *)&ch, 8, 0xFFFF);
  if (uartstatus != HAL_TIMEOUT){
  for(i = 0; i <= 7; i++)
  {
    d <<= 4;
    if('0' <= ch[i] && ch[i] <= '9')
    {
      d += ch[i] - '0'; 
    }
    else if('A' <= ch[i] && ch[i] <= 'F')
    {
      d += ch[i] - 'A' + 10;
    }
    else if('a' <= ch[i] && ch[i] <= 'f')
    {
      d += ch[i] - 'a' + 10;
    }
    else
    {
      printf("\n\r Assuming 0 for non-hexadecimal digit: %c \r\n",ch[i]);    
    }
    }
    write_word(d);
    printf("\n\r Core data write: "); 
    printf(" 0x%lx",d); 
    printf(" =>"); 
    long_to_binary(d);
    } 
    else printf("\n\r TIMED OUT!!\n\r");
}

static void w_test()
{
/* This routine, calling write_bit(), writes 0 or 1 on the desired core, from
   0 to 31. */

  char ch[6];
  int i, a = 0; 
  printf("\n\r Please enter address of bit to write: \n\r");
  uartstatus=HAL_UART_Receive(&UartHandle, (uint8_t *)&ch, 5, 0xFFFF);
  ch[5]='\0'; 
  if (uartstatus != HAL_TIMEOUT){
   for(i = 0; i < ADDRSIZE; i++)
   {
    a <<= 1;
    if(ch[i] != '0') // Assert ch == '1'
    {
      a += 1;
    }
   } 
  } else { 
    printf("\n\r TIMED OUT!!\n\r");
    return;  
  }
  printf("\n\r Please enter bit to write: \n\r");   
  uartstatus=HAL_UART_Receive(&UartHandle, (uint8_t *)&ch,1, 0xFFFF);
  ch[1]='\0';
  if (uartstatus != HAL_TIMEOUT){
  printf("\n\r Core data write to address: "); 
  address_to_binary (a);
  printf(" value ");
  if(ch[0] == '0')
  {
    write_bit(a, 0);
    printf("0 \n\r");
  }
  else // Assert ch == '1'
  {
    write_bit(a, 1);
    printf(" 1 \n\r");
  }
 } else printf("\n\r TIMED OUT!!\n\r");
}

static void r_test()
{
/* The function, calling read_bit(), reads the desired core, from 0 to 31. */

  char ch[6];
  int i, a = 0; 
  printf("\n\r Please enter address of bit to read: \n\r");
  uartstatus=HAL_UART_Receive(&UartHandle, (uint8_t *)&ch, 5, 0xFFFF);
  ch[5]='\0';   
  if (uartstatus != HAL_TIMEOUT){  
  for(i = 0; i < ADDRSIZE; i++)
  {
    a <<= 1;
    if(ch[i] != '0') // Assert ch == '1'
    {
      a += 1;
    }
  }  
  printf("\n\r Core data read from address: ");
  address_to_binary(a);
  printf(" found: ");
  printf(" %d\n\r",read_bit(a));
 }  
 else
   printf("\n\r TIMED OUT!!\n\r");
}

static void t_test()
{
/* This is an utility function that verifies whether all 32 cores are
   correctly functioning. Cores are scanned from 0 to 31 and read and write
   operations are performed displaying a results' table. */

  int i;
  unsigned long d;
  for(i = 0; i < WORDSIZE; i++)
  {
    write_bit(i, 0);
    d = read_bit(i);
    printf("\n\r Address (");
    printf("%d",i>>3);
    printf("%d",i&7);
    printf(") ");
    address_to_binary(i);
    printf(": wrote 0 read "); 
    printf("%ld",d);
    write_bit(i, 1);
    d = read_bit(i);
    printf(" wrote 1 read "); 
    printf("%ld\n\r",d);
  }    
}

void timing_enable(void)
{
/* This piece of code set microcontroller's registers required from the
   timing_delay_us() function. */

    static int enabled = 0;
    if (!enabled)
    {
        *SCB_DEMCR = *SCB_DEMCR | 0xF42400;
        *DWT_CYCCNT = 0; // reset the counter
        *DWT_CONTROL = *DWT_CONTROL | 1 ; // enable the counter
        enabled = 1;
    }
}

void timing_delay_us(unsigned int tick)
{
/* This piece of code, together with timing_enable(), implements microseconds
   delays. We use a 2 microseconds delay. */

    unsigned int start, current;
    start = *DWT_CYCCNT;
    do
    {
        current = *DWT_CYCCNT;
    } while((current - start) < tick);
}

void USART2_Configuration(void)
{
/* In this function, USART2 is configured and initialized. */

  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None parity // ODD parity
      - BaudRate = 115200 baud // 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance          = USARTx;
  UartHandle.Init.BaudRate     = 115200; 
  UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     = UART_STOPBITS_1;
  UartHandle.Init.Parity       = UART_PARITY_NONE; 
  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = UART_MODE_TX_RX;
  //UartHandle.Init.OverSampling = UART_OVERSAMPLING_16; 
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
}

void GPIO_Configuration(void)
{
/* In this function GPIOs are configured and initialized. */

  // Enable GPIOA Clock (to be able to program the configuration registers) 
   __HAL_RCC_GPIOA_CLK_ENABLE();
  // Configure PA05 IO in output external LED2 - Green.
  // Configure PA10 IO in output D2 PIN ARDUINO
  // Configure PA08 IO in output D7 PIN ARDUINO  
  // Configure PA09 IO in output D8 PIN ARDUINO   
  GPIO_InitStruct_A.Pin = GPIO_PIN_10 | GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct_A.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct_A.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct_A.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_A); 
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
  
  // Enable GPIOB Clock (to be able to program the configuration registers) 
   __HAL_RCC_GPIOB_CLK_ENABLE();
  // Configure PB03 IO in output D3 PIN ARDUINO  
  // Configure PB05 IO in output D4 PIN ARDUINO  
  // Configure PB04 IO in output D5 PIN ARDUINO   
  // Configure PB10 IO in output D6 PIN ARDUINO 
  GPIO_InitStruct_B.Pin = GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_10;
  GPIO_InitStruct_B.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct_B.Pull = GPIO_PULLDOWN; 
  GPIO_InitStruct_B.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_B);
 
  // Configure PC07 IO in output D9 PIN ARDUINO
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitStruct_C.Pin = GPIO_PIN_7;
  GPIO_InitStruct_C.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct_C.Pull = GPIO_PULLDOWN; 
  GPIO_InitStruct_C.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct_C);
}

int main(void)
{
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
  */
  HAL_Init();
  
  /* Configure the system clock to 100 MHz */
  SystemClock_Config();
   
  /* Configure the UART peripheral */
  USART2_Configuration(); 

  /* Configure the GPIOs */
  GPIO_Configuration();

  timing_enable();

  printf("\n\rWelcome! If you're new, try using the commands 'r', 'w', 't', 'R', 'W' to get started.\n\r"); 
  
  /* Infinite loop */ 
  while (1)
  {
   uartstatus = HAL_UART_Receive(&UartHandle, (uint8_t *)&option, 1, 0xFFFF);
   if (uartstatus != HAL_TIMEOUT)
   {
/* We test whether timeout has expired. */
    switch (option) 
    {
/* We test which command is to be executed. */
      case 'w':
        w_test();
        break;
      case 'r':  
        r_test();
        break;
      case 'W':
        W_test();
        break;
      case 'R':  
        R_test();
        break;
      case 't':  
        t_test();
        break; 
      default:
        printf("\n\r Ignoring unknown command: %c \n\r", option);
    }
   } 
  }
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF); 

  return ch;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 100000000
  *            HCLK(Hz)                       = 100000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 400
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED2 on */
  // BSP_LED_On(LED2);
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

