/**
  ******************************************************************************
  * @file    Wifi/WiFi_Client_Server/src/main.c
  * @author  MCD Application Team
  * @brief   This file provides main program functions
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private defines -----------------------------------------------------------*/
/* Update SSID and PASSWORD with own Access point settings */
#define SSID     "CenturyLink4234"
#define PASSWORD "fuckyoutom"
#define LCD_USE
#define CONNECTION_TRIAL_MAX          10

/* Private variables ---------------------------------------------------------*/
const uint8_t RemoteIP[] = {192, 168, 0, 64};
uint8_t RxData [500];

uint8_t* TxData[638] = "STM32 : Hello!\n";
uint16_t RxLen;
uint8_t  MAC_Addr[6];
uint8_t  IP_Addr[4];

#ifdef __GNUC__
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

/* E1.31 Public Constants */
const uint16_t E131_DEFAULT_PORT = 5568;
const uint8_t E131_DEFAULT_PRIORITY = 0x64;

/* E1.31 Private Constants */
const uint16_t _E131_PREAMBLE_SIZE = 0x0010;
const uint16_t _E131_POSTAMBLE_SIZE = 0x0000;
const uint8_t _E131_ACN_PID[] = { 0x41, 0x53, 0x43, 0x2d, 0x45, 0x31, 0x2e, 0x31, 0x37, 0x00, 0x00, 0x00 };
const uint32_t _E131_ROOT_VECTOR = 0x00000004;
const uint32_t _E131_FRAME_VECTOR = 0x00000002;
const uint8_t _E131_DMP_VECTOR = 0x02;
const uint8_t _E131_DMP_TYPE = 0xa1;
const uint16_t _E131_DMP_FIRST_ADDR = 0x0000;
const uint16_t _E131_DMP_ADDR_INC = 0x0001;
const int E131_OPT_TERMINATED = 6;
const int E131_OPT_PREVIEW = 7;

typedef union {
  PACK(struct {
    PACK(struct { /* ACN Root Layer: 38 bytes */
      uint16_t preamble_size;    /* Preamble Size */
      uint16_t postamble_size;   /* Post-amble Size */
      uint8_t  acn_pid[12];      /* ACN Packet Identifier */
      uint16_t flength;          /* Flags (high 4 bits) & Length (low 12 bits) */
      uint32_t vector;           /* Layer Vector */
      uint8_t  cid[16];          /* Component Identifier (UUID) */
    }) root;

    PACK(struct { /* Framing Layer: 77 bytes */
      uint16_t flength;          /* Flags (high 4 bits) & Length (low 12 bits) */
      uint32_t vector;           /* Layer Vector */
      uint8_t  source_name[64];  /* User Assigned Name of Source (UTF-8) */
      uint8_t  priority;         /* Packet Priority (0-200, default 100) */
      uint16_t reserved;         /* Reserved (should be always 0) */
      uint8_t  seq_number;       /* Sequence Number (detect duplicates or out of order packets) */
      uint8_t  options;          /* Options Flags (bit 7: preview data, bit 6: stream terminated) */
      uint16_t universe;         /* DMX Universe Number */
    }) frame;

    PACK(struct { /* Device Management Protocol (DMP) Layer: 523 bytes */
      uint16_t flength;          /* Flags (high 4 bits) / Length (low 12 bits) */
      uint8_t  vector;           /* Layer Vector */
      uint8_t  type;             /* Address Type & Data Type */
      uint16_t first_addr;       /* First Property Address */
      uint16_t addr_inc;         /* Address Increment */
      uint16_t prop_val_cnt;     /* Property Value Count (1 + number of slots) */
      uint8_t  prop_val[513];    /* Property Values (DMX start code + slots data) */
    }) dmp;
  });
} e131_packet_t;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

/* Private functions ---------------------------------------------------------*/

uint8_t* serialize(e131_packet_t* packet)
{
	return (uint8_t*)(packet);
}

/* Set the state of a framing option in an E1.31 packet */
int e131_set_option(e131_packet_t* packet, const int option, const bool state) {
    if (packet == NULL) {
        errno = EINVAL;
        return -1;
    }
    packet->frame.options ^= (-state ^ packet->frame.options) & (1 << (option % 8));
    return 0;
}

/* Initialize an E1.31 packet using a universe and a number of slots */
int e131_pkt_init(e131_packet_t* packet, const uint16_t universe, const uint16_t num_slots) {
    if (packet == NULL || universe < 1 || universe > 63999 || num_slots < 1 || num_slots > 512) {
        errno = EINVAL;
        return -1;
    }

    // compute packet layer lengths
    uint16_t prop_val_cnt = num_slots + 1;
    uint16_t dmp_length = prop_val_cnt +
        sizeof packet->dmp - sizeof packet->dmp.prop_val;
    uint16_t frame_length = sizeof packet->frame + dmp_length;
    uint16_t root_length = sizeof packet->root.flength +
        sizeof packet->root.vector + sizeof packet->root.cid + frame_length;

    // clear packet
    memset(packet, 0, sizeof * packet);

    // set Root Layer values
    packet->root.preamble_size = htons(_E131_PREAMBLE_SIZE);
    packet->root.postamble_size = htons(_E131_POSTAMBLE_SIZE);
    memcpy(packet->root.acn_pid, _E131_ACN_PID, sizeof packet->root.acn_pid);
    packet->root.flength = htons(0x7000 | root_length);
    packet->root.vector = htonl(_E131_ROOT_VECTOR);

    // set Framing Layer values
    packet->frame.flength = htons(0x7000 | frame_length);
    packet->frame.vector = htonl(_E131_FRAME_VECTOR);
    packet->frame.priority = E131_DEFAULT_PRIORITY;
    packet->frame.universe = htons(universe);

    // set Device Management Protocol (DMP) Layer values
    packet->dmp.flength = htons(0x7000 | dmp_length);
    packet->dmp.vector = _E131_DMP_VECTOR;
    packet->dmp.type = _E131_DMP_TYPE;
    packet->dmp.first_addr = htons(_E131_DMP_FIRST_ADDR);
    packet->dmp.addr_inc = htons(_E131_DMP_ADDR_INC);
    packet->dmp.prop_val_cnt = htons(prop_val_cnt);

    memcpy(&packet.frame.source_name, "E1.31 Test Client", 18);
    e131_set_option(&packet, E131_OPT_PREVIEW, true))

    return 0;
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  int32_t Socket = -1;
  uint16_t Datalen;
  uint16_t Trials = CONNECTION_TRIAL_MAX;
  uint8_t modulestr[20] = {0};

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  /* Configure LED4 */
  BSP_LED_Init(LED4);
  BSP_LCD_InitEx(LCD_ORIENTATION_LANDSCAPE_ROT180);
  BSP_LCD_DisplayOn();
  BSP_LCD_Clear(LCD_COLOR_WHITE);

  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_BLUE);

  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  BSP_LCD_FillRect(0, 0, BSP_LCD_GetXSize(), (BSP_LCD_GetYSize() -200));

  BSP_LCD_SetFont(&Font12);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DisplayStringAt(10, 10, (uint8_t *)"ES-WIFI Module in TCP", CENTER_MODE );
  BSP_LCD_DisplayStringAt(10, 23, (uint8_t *)"Client mode demonstration", CENTER_MODE );
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_DisplayStringAtLine(4, (uint8_t *)"TCP Client Instructions:");
  BSP_LCD_DisplayStringAtLine(6, (uint8_t *)"1-Make sure your Phone connected ");
  BSP_LCD_DisplayStringAtLine(7, (uint8_t *)"to the network that you configured");
  BSP_LCD_DisplayStringAtLine(8, (uint8_t *)"using Configuration AccessPoint.");
  BSP_LCD_DisplayStringAtLine(10,(uint8_t *)"2-Create a server by using the ");
  BSP_LCD_DisplayStringAtLine(11,(uint8_t *)"android application TCP Server");
  BSP_LCD_DisplayStringAtLine(12,(uint8_t *)"with port(8002).");
  BSP_LCD_DisplayStringAtLine(14,(uint8_t *)"3-Get the Network Name or IP Address");
  BSP_LCD_DisplayStringAtLine(15,(uint8_t *)"of your Android from the step 2.");

  HAL_Delay(2000);
  /*Initialize  WIFI module */
  if(WIFI_Init() ==  WIFI_STATUS_OK)
  {
    BSP_LCD_Clear(LCD_COLOR_WHITE);
    BSP_LCD_SetBackColor(LCD_COLOR_BLUE);

    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_FillRect(0, 0, BSP_LCD_GetXSize(), (BSP_LCD_GetYSize() -200));

    BSP_LCD_SetFont(&Font12);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DisplayStringAt(10, 10, (uint8_t *)"ES-WIFI Module in TCP", CENTER_MODE );
    BSP_LCD_DisplayStringAt(10, 23, (uint8_t *)"Client mode demonstration", CENTER_MODE );

    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_DisplayStringAtLine(7, (uint8_t *)"ES-WIFI Module Initialized.");
    if(WIFI_GetMAC_Address(MAC_Addr) == WIFI_STATUS_OK)
    {
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      BSP_LCD_DisplayStringAtLine(9, (uint8_t *)"Es-wifi module MAC Address:");
      sprintf((char*)modulestr,"%X:%X:%X:%X:%X:%X.", MAC_Addr[0],
              MAC_Addr[1],
              MAC_Addr[2],
              MAC_Addr[3],
              MAC_Addr[4],
              MAC_Addr[5]);

      BSP_LCD_DisplayStringAtLine(10, (uint8_t *) modulestr);
    }
    else
    {
      BSP_LCD_SetTextColor(LCD_COLOR_RED);
      BSP_LCD_DisplayStringAtLine(9, (uint8_t *)"ERROR : CANNOT get MAC address");
      BSP_LED_On(LED4);
    }

    if( WIFI_Connect(SSID, PASSWORD, WIFI_ECN_WPA2_PSK) == WIFI_STATUS_OK)
    {
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      BSP_LCD_DisplayStringAtLine(11, (uint8_t *)"Es-wifi module connected");
      if(WIFI_GetIP_Address(IP_Addr) == WIFI_STATUS_OK)
      {
        BSP_LCD_DisplayStringAtLine(12, (uint8_t *)"es-wifi module got IP Address :");
        sprintf((char*)modulestr,"%d.%d.%d.%d",  IP_Addr[0],
                IP_Addr[1],
                IP_Addr[2],
                IP_Addr[3]);

        BSP_LCD_DisplayStringAtLine(13, (uint8_t *) modulestr);

        BSP_LCD_DisplayStringAtLine(14, (uint8_t *)"Trying to connect to Server:");
        sprintf((char*)modulestr,"%d.%d.%d.%d:8002",  RemoteIP[0],
                RemoteIP[1],
                RemoteIP[2],
                RemoteIP[3]);

        BSP_LCD_DisplayStringAtLine(15, (uint8_t *) modulestr);

        while (Trials--)
        {
          //changed TCP to UDP, here. Hopefully this works.
          if( WIFI_OpenClientConnection(0, WIFI_UDP_PROTOCOL, "UDP_CLIENT", (char *)RemoteIP, E131_DEFAULT_PORT, 0) == WIFI_STATUS_OK)
          {
            BSP_LCD_DisplayStringAtLine(16, (uint8_t *)"UDP Connection opened successfully.");
            Socket = 0;
          }
        }
        if(!Trials)
        {
          BSP_LCD_SetTextColor(LCD_COLOR_RED);
          BSP_LCD_DisplayStringAtLine(14, (uint8_t *)"ERROR : Cannot open Connection");
          BSP_LED_On(LED4);
        }
      }
      else
      {
        BSP_LCD_SetTextColor(LCD_COLOR_RED);
        BSP_LCD_DisplayStringAtLine(12, (uint8_t *)"ERROR: es-wifi module CANNOT get IP address");
        BSP_LED_On(LED4);
      }
    }
    else
    {
      BSP_LCD_SetTextColor(LCD_COLOR_RED);
      BSP_LCD_DisplayStringAtLine(11, (uint8_t *)"ERROR:es-wifi module NOT connected\n");
      BSP_LED_On(LED4);
    }
  }
  else
  {
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    BSP_LCD_DisplayStringAtLine(7, (uint8_t *)"ERROR : WIFI Module cannot be initialized.\n");
    BSP_LED_On(LED4);
  }

  while(1)
  {
    if(Socket != -1)
    {
	  if(WIFI_SendData(Socket, TxData, sizeof(TxData), &Datalen) != WIFI_STATUS_OK)
	  {
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_DisplayStringAtLine(8, (uint8_t *)"ERROR : Failed to send Data.");
	  }
    }
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 100000000
  *            HCLK(Hz)                       = 100000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 200
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
   RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);

  if(ret != HAL_OK)
  {
    while(1);
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
  if(ret != HAL_OK)
  {
    while(1);
  }
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
