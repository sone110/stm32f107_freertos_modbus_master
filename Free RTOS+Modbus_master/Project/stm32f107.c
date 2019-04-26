/**
  ******************************************************************************
  * @file    stm32f107.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    11/20/2009
  * @brief   STM32F107 hardware configuration
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32_eth.h"
#include "stm32f107.h"
#include "usart.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PHY_ADDRESS       0x01 /* Relative to STM3210C-EVAL Board */

//#define MII_MODE          /* MII mode for STM3210C-EVAL Board (MB784) (check jumpers setting) */
#define RMII_MODE       /* RMII mode for STM3210C-EVAL Board (MB784) (check jumpers setting) */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void GPIO_Configuration(void);
void ETH_NVIC_Config(void);
void Ethernet_Configuration(void);

/**
  * @brief  Setup STM32 system (clocks, Ethernet, GPIO, NVIC) and STM3210C-EVAL resources.
  * @param  None
  * @retval None
  */
void System_Setup(void)
{
  /* Enable ETHERNET clock  */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC | RCC_AHBPeriph_ETH_MAC_Tx |
                        RCC_AHBPeriph_ETH_MAC_Rx, ENABLE);

  /* Enable GPIOs and ADC1 clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |	RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD  | 
	                       RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO 
						             , ENABLE);

  /* Configure the GPIO ports */
  GPIO_Configuration();

  /* Configure the Ethernet peripheral */
  Ethernet_Configuration();
}

/**
  * @brief  Configures the Ethernet Interface
  * @param  None
  * @retval None
  */
void Ethernet_Configuration(void)
{
	ETH_InitTypeDef ETH_InitStructure;

	/* MII/RMII Media interface selection ------------------------------------------*/
	#ifdef MII_MODE /* Mode MII with STM3210C-EVAL  */
		GPIO_ETH_MediaInterfaceConfig(GPIO_ETH_MediaInterface_MII);

		/* Get HSE clock = 25MHz on PA8 pin (MCO) */
		RCC_MCOConfig(RCC_MCO_HSE);

	#elif defined RMII_MODE  /* Mode RMII with STM3210C-EVAL */
		GPIO_ETH_MediaInterfaceConfig(GPIO_ETH_MediaInterface_RMII);

		/* Set PLL3 clock output to 50MHz (25MHz /5 *10 =50MHz) */
		RCC_PLL3Config(RCC_PLL3Mul_10);
		/* Enable PLL3 */
		RCC_PLL3Cmd(ENABLE);
		/* Wait till PLL3 is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_PLL3RDY) == RESET)
		{}

		/* Get PLL3 clock on PA8 pin (MCO) */
		RCC_MCOConfig(RCC_MCO_PLL3CLK);
	#endif

	/* NVIC configuration */
	ETH_NVIC_Config(); 
	
	/* Reset ETHERNET on AHB Bus */
	ETH_DeInit();

	/* Software reset */
	ETH_SoftwareReset();

	/* Wait for software reset */
	while (ETH_GetSoftwareResetStatus() == SET);
			
	if(ETH_ReadPHYRegister(PHY_ADDRESS, 0x02) != 0x0243)
  {
     NVIC_SystemReset();
  }

	
  
	/* ETHERNET Configuration ------------------------------------------------------*/
	/* Call ETH_StructInit if you don't like to configure all ETH_InitStructure parameter */
	ETH_StructInit(&ETH_InitStructure);

	/* Fill ETH_InitStructure parametrs */
	/*------------------------   MAC   -----------------------------------*/
	ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Enable  ;
	ETH_InitStructure.ETH_LoopbackMode = ETH_LoopbackMode_Disable;
	ETH_InitStructure.ETH_RetryTransmission = ETH_RetryTransmission_Disable;
	ETH_InitStructure.ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable;
	ETH_InitStructure.ETH_ReceiveAll = ETH_ReceiveAll_Disable;
	ETH_InitStructure.ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Enable;
	ETH_InitStructure.ETH_PromiscuousMode = ETH_PromiscuousMode_Disable;
	ETH_InitStructure.ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;
	ETH_InitStructure.ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;
	#ifdef CHECKSUM_BY_HARDWARE
	ETH_InitStructure.ETH_ChecksumOffload = ETH_ChecksumOffload_Enable;
	#endif

	/*------------------------   DMA   -----------------------------------*/  

	/* When we use the Checksum offload feature, we need to enable the Store and Forward mode: 
	the store and forward guarantee that a whole frame is stored in the FIFO, so the MAC can insert/verify the checksum, 
	if the checksum is OK the DMA can handle the frame otherwise the frame is dropped */
	ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame = ETH_DropTCPIPChecksumErrorFrame_Enable; 
	ETH_InitStructure.ETH_ReceiveStoreForward = ETH_ReceiveStoreForward_Enable;         
	ETH_InitStructure.ETH_TransmitStoreForward = ETH_TransmitStoreForward_Enable;     

	ETH_InitStructure.ETH_ForwardErrorFrames = ETH_ForwardErrorFrames_Disable;       
	ETH_InitStructure.ETH_ForwardUndersizedGoodFrames = ETH_ForwardUndersizedGoodFrames_Disable;   
	ETH_InitStructure.ETH_SecondFrameOperate = ETH_SecondFrameOperate_Enable;                                                          
	ETH_InitStructure.ETH_AddressAlignedBeats = ETH_AddressAlignedBeats_Enable;      
	ETH_InitStructure.ETH_FixedBurst = ETH_FixedBurst_Enable;                
	ETH_InitStructure.ETH_RxDMABurstLength = ETH_RxDMABurstLength_32Beat;          
	ETH_InitStructure.ETH_TxDMABurstLength = ETH_TxDMABurstLength_32Beat;                                                                 
	ETH_InitStructure.ETH_DMAArbitration = ETH_DMAArbitration_RoundRobin_RxTx_2_1;

	/* Configure Ethernet */
	  if(ETH_Init(&ETH_InitStructure, PHY_ADDRESS))
  {
    printf("ETH_ReadPHYRegister(PHY_ADDRESS, 0x00) = 0x%x\n",ETH_ReadPHYRegister(PHY_ADDRESS, 0x00)) ; 
		printf("ETH_ReadPHYRegister(PHY_ADDRESS, 0x01) = 0x%x\n",ETH_ReadPHYRegister(PHY_ADDRESS, 0x01)) ; 
		printf("ETH_ReadPHYRegister(PHY_ADDRESS, 0x02) = 0x%x\n",ETH_ReadPHYRegister(PHY_ADDRESS, 0x02)) ; 
		printf("ETH_ReadPHYRegister(PHY_ADDRESS, 0x03) = 0x%x\n",ETH_ReadPHYRegister(PHY_ADDRESS, 0x03)) ; 
		printf("ETH_ReadPHYRegister(PHY_ADDRESS, 0x04) = 0x%x\n",ETH_ReadPHYRegister(PHY_ADDRESS, 0x04)) ; 
		printf("ETH_ReadPHYRegister(PHY_ADDRESS, 0x05) = 0x%x\n",ETH_ReadPHYRegister(PHY_ADDRESS, 0x05)) ; 
		printf("ETH_ReadPHYRegister(PHY_ADDRESS, 0x06) = 0x%x\n",ETH_ReadPHYRegister(PHY_ADDRESS, 0x06)) ; 
		printf("ETH_ReadPHYRegister(PHY_ADDRESS, 0x07) = 0x%x\n",ETH_ReadPHYRegister(PHY_ADDRESS, 0x07)) ; 
  }
  else
  {
    //STM_EVAL_BEEPOn();
    //STM_EVAL_LEDOff(LED2);
    //LCD_DisplayStringLine(Line5, "XX ETH Link ERROR XX");
    printf(" ---> 提示: 网口连接失败 Ethernet_Configuration() ETH_ERROR");
    printf("\r\n #网口连接失败, 请检查网线连接是否正常! 连接好网线后请复位系统!\r\n ");
    
    //STM_EVAL_BEEPOff();
    //Delay_ARMJISHU(20000000);
    if(!ETH_Init(&ETH_InitStructure, PHY_ADDRESS))
    {
      //Delay_ARMJISHU(100000000);
			printf("\r\n 自动复位系统!!! \r\n ");
			NVIC_SystemReset();
    }
  }

	/* Enable the Ethernet Rx Interrupt */
	ETH_DMAITConfig(ETH_DMA_IT_NIS | ETH_DMA_IT_R, ENABLE);

}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* ETHERNET pins configuration */
  /* AF Output Push Pull:
  - ETH_MII_MDIO / ETH_RMII_MDIO: PA2
  - ETH_MII_MDC / ETH_RMII_MDC: PC1
  - ETH_MII_TXD2: PC2
  - ETH_MII_TX_EN / ETH_RMII_TX_EN: PB11
  - ETH_MII_TXD0 / ETH_RMII_TXD0: PB12
  - ETH_MII_TXD1 / ETH_RMII_TXD1: PB13
 */

  /* Configure PA2 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure PC1, PC2 and PC3 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure PB5, PB8, PB11, PB12 and PB13 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /**************************************************************/
  /*               For Remapped Ethernet pins                   */
  /*************************************************************/
  /* Input (Reset Value):
  - ETH_MII_RX_CLK / ETH_RMII_REF_CLK: PA1
  - ETH_MII_RX_DV / ETH_RMII_CRS_DV: PD8
  - ETH_MII_RXD0 / ETH_RMII_RXD0: PD9
  - ETH_MII_RXD1 / ETH_RMII_RXD1: PD10
 */

  /* ETHERNET pins remapp in STM3210C-EVAL board: RX_DV and RxD[3:0] */
  GPIO_PinRemapConfig(GPIO_Remap_ETH, ENABLE);

  /* Configure PA1 as input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Configure PD8, PD9, PD10, PD11 and PD12 as input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure); /**/

  /* ADC Channel14 config ----------------------------------------*/
  /* Relative to STM3210D-EVAL Board   */
  /* Configure PC.00 (ADC Channel10) as analog input -------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* MCO pin configuration--------------------------------------- */
  /* Configure MCO (PA8) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
  * @brief  Configures the ADC.
  * @param  None
  * @retval None
  */

/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void ETH_NVIC_Config(void)
{
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
  
  /* Enable the Ethernet global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = ETH_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);    
  
}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
