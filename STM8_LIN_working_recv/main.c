/**
  ******************************************************************************
  * @file    UART1_HyperTerminal_Interrupt\main.c
  * @author  MCD Application Team
  * @version  V2.2.0
  * @date     30-September-2014
  * @brief   This file contains the main function for UART1 using interrupts in 
  *          communication example.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
//#include "stm8s_eval.h"

volatile unsigned char pid = 0x12; //0x02;
volatile int b = 3;
volatile int x = 0;
volatile unsigned char id = 0;
volatile unsigned char pid_crc = 0;
volatile unsigned char fr1 = 0;
volatile unsigned char fr2 = 0;
volatile unsigned char fr3 = 0;
volatile unsigned char fr4 = 0;
volatile unsigned char fr5 = 0;
volatile unsigned char fr6 = 0;
volatile unsigned char fr7 = 0;
volatile unsigned char fr8 = 0;
volatile unsigned char crc = 0;

unsigned char retract = 0; // retract commands: 0 - idle, 1 - open, 2 - close

__IO uint32_t current_millis = 0; //--IO: volatile read/write 
volatile uint32_t idle_millis = 0;
uint32_t last_millis = 0;

/**
  * @addtogroup UART1_HyperTerminal_Interrupt
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void GPIO_Config(void);
static void CLK_Config(void);
static void UART1_Config(void);
static void TIM4_Config(void);
void Delay(uint32_t nCount);
unsigned char Parity( unsigned char pid );
int isNthBitSet (unsigned char c, int n);
/* Private functions ---------------------------------------------------------*/
void p_LIN_wait_us(uint32_t n)
{
  volatile uint32_t p,t;

  // kleine Pause
  for(p=0;p<n;p++) {
    for(t=0;t<1;t++); // ca 56us when p = 2
  }
}

uint32_t millis(void)
{
	return current_millis;
}


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
  /* Clock configuration -----------------------------------------*/
  CLK_Config();  

  /* GPIO configuration -----------------------------------------*/
  GPIO_Config();  

  /* UART1 configuration -----------------------------------------*/
  UART1_Config(); 
  
  /* TIM4 configuration -----------------------------------------*/  
  TIM4_Config();
  
  GPIO_WriteLow(GPIOC,GPIO_PIN_3);
  GPIO_WriteLow(GPIOA,GPIO_PIN_3);
  //GPIO_WriteLow(GPIOB,GPIO_PIN_5);
  GPIO_WriteHigh(GPIOD,GPIO_PIN_4);
  while (1)
  {
    
    if (idle_millis >= 100) {
      x = 0;
      //need to reset frame receiving status
    }
    
    uint32_t currentTime = millis();
    if (currentTime - last_millis >= 15) {
      //GPIO_WriteReverse(GPIOB,GPIO_PIN_5);
    
      last_millis = currentTime;
        // need to reset idle time in order to restore connection
      //idle_millis = 0; 
    }
    
 
    if (x == 0) {
      //GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
      //GPIO_WriteHigh(GPIOD,GPIO_PIN_3);
      
      //GPIO_WriteLow(GPIOD,GPIO_PIN_3);
    }
    
    if (x == 1) {
      idle_millis = 0;          //reset idle check
      //GPIO_WriteHigh(GPIOC,GPIO_PIN_3);
      
      if((fr1 & 128) != 0) {      //ceck if bit 7 is set
        //GPIO_WriteHigh(GPIOC,GPIO_PIN_3);
      }
      if((fr1 & 64) != 0) {       //ceck if bit 6 is set пусть будет команда на раскрытие
        retract = 1;
        //GPIO_WriteHigh(GPIOC,GPIO_PIN_3);
      }
      if((fr1 & 32) != 0) {       //ceck if bit 5 is set а это пусть будет команда на складывание
        retract = 2;
        //GPIO_WriteHigh(GPIOA,GPIO_PIN_3);
      }
      if((fr1 & 16) != 0) {       //ceck if bit 4 is set
        //GPIO_WriteLow(GPIOB,GPIO_PIN_5);
      }
      if((fr1 & 8) != 0) {        //ceck if bit 3 is set
        //GPIO_WriteLow(GPIOB,GPIO_PIN_5);
      }
      if((fr1 & 4) != 0) {        //ceck if bit 2 is set
        //GPIO_WriteLow(GPIOB,GPIO_PIN_5);
      }
      
      if((fr1 & 2) != 0) {        //ceck if bit 1 is set
        //GPIO_WriteLow(GPIOB,GPIO_PIN_5);
        //GPIO_WriteHigh(GPIOC,GPIO_PIN_3);
      }
      else{
        //GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
        //GPIO_WriteLow(GPIOC,GPIO_PIN_3);
      }

      if((fr1 & 1) == 1) {        //ceck if bit 0 is set
        //GPIO_WriteLow(GPIOB,GPIO_PIN_5);
      }
      else{
        //GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
      }
      //Delay(500);
      //GPIO_WriteReverse(GPIOB,GPIO_PIN_5);
      
    }
    
    if (retract == 0){ //команда - сложить зеркало
      GPIO_WriteLow(GPIOA,GPIO_PIN_3);
      GPIO_WriteLow(GPIOC,GPIO_PIN_3);    
    }
    
    // ******** если команда на раскрытие зеркала
    if (retract == 1){ //команда - открыть зеркало
      if(!GPIO_ReadInputPin( GPIOB, GPIO_PIN_4)){ //проверяем физическое положение - если ниизкий уровень, то надо открывать
        GPIO_WriteHigh(GPIOC,GPIO_PIN_3); //открываем противоположное реле
        GPIO_WriteLow(GPIOA,GPIO_PIN_3);
      }
      else { //если физ.положение - выс уровень, значит зеркало открыто, реле закрываем, команду сбрасываем.
        GPIO_WriteLow(GPIOC,GPIO_PIN_3);
        GPIO_WriteLow(GPIOA,GPIO_PIN_3);
        retract = 0;
      }
    }
    // ******** если команда на складывание зеркала
    if (retract == 2){ //команда - сложить зеркало
      if(!GPIO_ReadInputPin( GPIOB, GPIO_PIN_5)){ //проверяем физическое положение - если ниизкий уровень, то надо сложить
        GPIO_WriteHigh(GPIOA,GPIO_PIN_3); //открываем противоположное реле
        GPIO_WriteLow(GPIOC,GPIO_PIN_3); 
      }
      else { //если физ.положение - выс уровень, значит зеркало сложено, реле закрываем, команду сбрасываем.
        GPIO_WriteLow(GPIOA,GPIO_PIN_3);
        GPIO_WriteLow(GPIOC,GPIO_PIN_3); 
        retract = 0;
      }
    }
    
  }
}

/**
  * @brief  Configure system clock to run at 16Mhz
  * @param  None
  * @retval None
  */
static void CLK_Config(void)
{
    /* Initialization of the clock */
    /* Clock divider to HSI/1 */
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
}

/**
  * @brief  Configure LEDs available on the evaluation board
  * @param  None
  * @retval None
  */
static void GPIO_Config(void)
{
    GPIO_DeInit(GPIOA);
    GPIO_DeInit(GPIOB);
    GPIO_DeInit(GPIOC);
    GPIO_DeInit(GPIOD);
/* Initialize LEDs mounted on the Eval board */
    GPIO_Init(GPIOD,GPIO_PIN_4,GPIO_MODE_OUT_PP_HIGH_FAST); //Relay1
    GPIO_Init(GPIOA,GPIO_PIN_3,GPIO_MODE_OUT_PP_HIGH_FAST); //Relay2
    
    GPIO_Init(GPIOD,GPIO_PIN_3,GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(GPIOD,GPIO_PIN_2,GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(GPIOD,GPIO_PIN_1,GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(GPIOC,GPIO_PIN_7,GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(GPIOC,GPIO_PIN_6,GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(GPIOC,GPIO_PIN_5,GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(GPIOC,GPIO_PIN_4,GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(GPIOC,GPIO_PIN_3,GPIO_MODE_OUT_PP_HIGH_FAST);
    
    GPIO_Init(GPIOB,GPIO_PIN_5,GPIO_MODE_OUT_PP_HIGH_FAST);
    
    GPIO_Init(GPIOB,GPIO_PIN_4,GPIO_MODE_IN_PU_NO_IT);
    
    //GPIO_Init(GPIOB,GPIO_PIN_5,GPIO_MODE_IN_PU_NO_IT);
}

/**
  * @brief  Configure UART1 for the communication with HyperTerminal
  * @param  None
  * @retval None
  */
static void UART1_Config(void)
{
  /* EVAL COM (UART) configuration -----------------------------------------*/
  /* USART configured as follow:
        - BaudRate = 19200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Receive and transmit enabled
        - UART Clock disabled
  */
  UART1_Init((uint32_t)19200, UART1_WORDLENGTH_8D,UART1_STOPBITS_1, UART1_PARITY_NO,
                   UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);

  UART1_LINBreakDetectionConfig(UART1_LINBREAKDETECTIONLENGTH_11BITS);
  
  
  
  
  /* Enable the UART Receive interrupt: this interrupt is generated when the UART
    receive data register is not empty - overrun */
  UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);
  
  /* Enable the LIN break interrupt: this interrupt is generated when the UART
    receive break field */
  UART1_ITConfig(UART1_IT_LBDF, ENABLE);
  
  /* Enable the UART Transmit complete interrupt: this interrupt is generated 
     when the UART transmit Shift Register is empty */
  //UART1_ITConfig(UART1_IT_TXE, ENABLE);

  /* Enable UART */
  //UART1_Cmd(ENABLE);
  UART1_LINCmd(ENABLE);
    /* Enable general interrupts */
  enableInterrupts();
  UART1->CR4 &= ((uint8_t)~UART1_FLAG_LBDF); //appears that flag needs to be cleared
  
}

static void TIM4_Config(void)
{
        /* TIM4 configuration:
	- TIM4CLK is set to 16 MHz, the TIM4 Prescaler is equal to 128 so the TIM1 counter
	clock used is 16 MHz / 128 = 125 000 Hz
	- With 125 000 Hz we can generate time base:
	  max time base is 2.048 ms if TIM4_PERIOD = 255 --> (255 + 1) / 125000 = 2.048 ms
	  min time base is 0.016 ms if TIM4_PERIOD = 1   --> (  1 + 1) / 125000 = 0.016 ms
	- In this example we need to generate a time base equal to 1 ms
	so TIM4_PERIOD = (0.001 * 125000 - 1) = 124 */

	/* Time base configuration */
	TIM4_TimeBaseInit(TIM4_PRESCALER_128, 124);
	/* Clear TIM4 update flag */
	TIM4_ClearFlag(TIM4_FLAG_UPDATE);
	/* Enable update interrupt */
	TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);

	/* enable interrupts */
	enableInterrupts();

	/* Enable TIM4 */
	TIM4_Cmd(ENABLE);
}  


int isNthBitSet (unsigned char c, int n) {
    static unsigned char mask[] = {128, 64, 32, 16, 8, 4, 2, 1};
    return ((c & mask[n]) != 0);
}

/**
  * @brief  Delay.
  * @param  nCount
  * @retval None
  */
void Delay(uint32_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}

/**
  * @brief  Odd Parity.
  * @param  b
  * @retval ret parity bit
  */
unsigned char Parity( unsigned char b )
{
unsigned char odd_p;
unsigned char even_p;

odd_p = b & 0x17; //0b00010111

odd_p = odd_p ^ (odd_p>>4);
odd_p = odd_p ^ (odd_p>>2);
odd_p = odd_p ^ (odd_p>>1);
odd_p = odd_p &1;
odd_p = odd_p << 7;

even_p = b & 0x3A; //0b00111010
even_p = even_p ^ (even_p>>4);
even_p = even_p ^ (even_p>>2);
even_p = even_p ^ (even_p>>1);
even_p = even_p << 6;

return b|even_p|odd_p;
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
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

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/*
void LinCommand (void) // Объявляемся
  {
   unsigned int chk=0; // объявляем беззнаковую переменную в 16 бит
   unsigned char chk1; // объявляем беззнаковую переменную в 8 бит
    DataTX(0x3c); // служебные байты, например адрес, в подсчете не участвуют
    chk+=DataTX(0xA6); // 1 байт данных
    chk+=DataTX(0xB4); // 2 байт данных
    chk+=DataTX(0x80); // 3 байт данных
    chk+=DataTX(0x6A); // 4 байт данных
    chk+=DataTX(0x76); // 5 байт данных
    chk+=DataTX(0xE0); // 6 байт данных
    chk+=DataTX(0xFE); // 7 байт данных
    chk+=DataTX(0x00); // 8 байт данных
    chk1=chk>>8; // сдвигаем и пишем во вторую переменную
    DataTX(~(chk1+chk)); // передаем итоговую контрольную сумму
 }

void LIN_UART_ISR(void) {
  uint16_t wert;

  if (USART_GetITStatus(LIN_UART, USART_IT_LBD) == SET) {
    //---------------------------
    // BreakField
    //---------------------------
    // flag loeschen
    USART_ClearITPendingBit(LIN_UART, USART_IT_LBD);

    LIN_SLAVE.mode=RECEIVE_SYNC;
  }


  if (USART_GetITStatus(LIN_UART, USART_IT_RXNE) == SET) {
    // Daten auslesen
    wert=USART_ReceiveData(LIN_UART);

    // check welcher Mode gerade aktiv ist
    if(LIN_SLAVE.mode==RECEIVE_SYNC) {
      //---------------------------
      // SyncField
      //---------------------------
      if(wert==LIN_SYNC_DATA) {
        LIN_SLAVE.mode=RECEIVE_ID;
      }
      else {
        LIN_SLAVE.mode=WAIT_4_BREAK;              
      }
    }
    else if(LIN_SLAVE.mode==RECEIVE_ID) {
      //---------------------------
      // IDField
      //---------------------------
      LIN_SLAVE.mode=ID_OK;
      LIN_FRAME.frame_id=(uint8_t)(wert);      
    }
    else if(LIN_SLAVE.mode==RECEIVE_DATA) {
      //---------------------------
      // DataField
      //---------------------------
      // Daten speichern
      LIN_FRAME.data[LIN_SLAVE.data_ptr]=(uint8_t)(wert);
      // Pointer weiterschalten
      LIN_SLAVE.data_ptr++;
      // check ob alle Daten empfangen sind
      if(LIN_SLAVE.data_ptr>=LIN_FRAME.data_len) {          
        LIN_SLAVE.mode=RECEIVE_CRC;
      }
    }
    else if(LIN_SLAVE.mode==RECEIVE_CRC) { 
      //---------------------------
      // CRCField
      //---------------------------     
      LIN_SLAVE.crc=(uint8_t)(wert);
      LIN_SLAVE.mode=WAIT_4_BREAK; 
    }
  }
}

*/