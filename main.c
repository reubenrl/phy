/***************************************************************************//**
 * @file main.c
 * @brief EZRadio trx example with auto acknowledge option enabled.
 *
 * This example shows how to easily implement a trx code with auto acknowledge
 * option for your controller using EZRadio or EZRadioPRO devices.
 *
 * @version 4.4.0
 *******************************************************************************
 * @section License
 * <b>Copyright 2015 Silicon Labs, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "spidrv.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "gpiointerrupt.h"
#include "rtcdriver.h"

#include "ezradio_cmd.h"
#include "ezradio_api_lib.h"
#include "ezradio_plugin_manager.h"

#include "bsp.h"
#include "uart_dbg_print.h"
#include "cmd_msg.h"
#include "ustimer.h"


#if defined(WBRD200_VER1)

#include "logger_interface.h"
#include "ee_m24c08.h"
#include "ezr_temperature.h"

uint8_t	strRxLogger[32];

static volatile bool rtsEvent = false;
/* Uart CTS callback functionns. */
static void GPIO_PD6_IRQHandler( uint8_t pin );

#elif defined(WBRD100_VER1)
#include "measure.h"
/* Push button callback functionns. */
static void GPIO_PB0_IRQHandler( uint8_t pin );
#endif

#if (defined EZRADIO_VARIABLE_DATA_START)
#define APP_PKT_DATA_START EZRADIO_VARIABLE_DATA_START
#else
#define APP_PKT_DATA_START 1u
#endif

static void appPacketTransmittedCallback ( EZRADIODRV_Handle_t handle, Ecode_t status );
static void appPacketReceivedCallback ( EZRADIODRV_Handle_t handle, Ecode_t status );
static void appPacketCrcErrorCallback ( EZRADIODRV_Handle_t handle, Ecode_t status );
static void appAutoAckTransmittedCallback ( EZRADIODRV_Handle_t handle, Ecode_t status );

#if !defined(__CROSSWORKS_ARM) && defined(__GNUC__)
/* sniprintf does not process floats, but occupy less flash memory ! */
#define snprintf    sniprintf
#endif

/* Defines the number of packets to send for one press of PB1.
 * Sends infinite number of packets if defined to 0xFFFF. */
#define APP_TX_PKT_SEND_NUM   0xFFFF

/* Length of the actual data in the Tx packet */
#define APP_TX_PKT_DATA_LENGTH   2u

/* Length of the actual data in the ACK packet */
#define APP_AUTO_ACK_PKT_DATA_LENGTH   3u

/* Rx packet data array */
static uint8_t radioRxPkt[EZRADIO_FIFO_SIZE];

/* Tx packet data array, initialized with the default payload in the generated header file */
static uint8_t radioTxPkt[EZRADIO_FIFO_SIZE] = RADIO_CONFIG_DATA_CUSTOM_PAYLOAD;

/* Auto ack packet data array */
static uint8_t radioAutoAckPkt[RADIO_CONFIG_DATA_MAX_PACKET_LENGTH];

#if (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_PACKET_LENGTH_ENABLE == TRUE)
/* Default length configuration for normal transmission */
static EZRADIODRV_PacketLengthConfig_t radioTxLengthConf =
    { ezradiodrvTransmitLenghtDefault, RADIO_CONFIG_DATA_MAX_PACKET_LENGTH, RADIO_CONFIG_DATA_FIELD_LENGTH };
#endif //#if (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_PACKET_LENGTH_ENABLE == TRUE)

/* Data counter in transmitted packet */
static volatile uint16_t appDataCntr = 0;

/* Packet counter */
static volatile uint16_t appTxPktCntr = 0;

/* Sign tx active state */
static volatile bool appTxActive = false;

/* RTC frequency */
#define APP_RTC_FREQ_HZ 2u
/* RTC timeout */
#define APP_RTC_TIMEOUT_MS ( 1000u / APP_RTC_FREQ_HZ )

/* RTC set time is expired */
static volatile bool rtcTick = false;

/** Timer used to issue time elapsed interrupt. */
static RTCDRV_TimerID_t rtcTickTimer;

/**************************************************************************//**
 * @brief Setup GPIO interrupt for pushbuttons.
 *****************************************************************************/
static void GpioSetup(void)
{
  /* Enable GPIO clock */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Initialize GPIO interrupt */
  GPIOINT_Init();

#if defined(WBRD200_VER1)
  /* Configure PD6 as input and enable interrupt */
  GPIO_PinModeSet(GPIO_RADIO_CTS_PORT, GPIO_RADIO_CTS_PIN, gpioModeInputPull, 0);
  GPIO_IntConfig(GPIO_RADIO_CTS_PORT, GPIO_RADIO_CTS_PIN, false, true, true);
  GPIOINT_CallbackRegister( GPIO_RADIO_CTS_PIN, GPIO_PD6_IRQHandler );

#elif defined(WBRD100_VER1)
  /* Configure PB0 as input and enable interrupt */
   GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInputPull, 1);
   GPIO_IntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, false, true, true);
   GPIOINT_CallbackRegister( BSP_GPIO_PB0_PIN, GPIO_PB0_IRQHandler );

   /* Configure PB1 as input and enable interrupt */
   //GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, gpioModeInputPull, 1);
   //GPIO_IntConfig(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, false, true, true);
   //GPIOINT_CallbackRegister( BSP_GPIO_PB1_PIN, GPIO_PB1_IRQHandler );

   GPIO_PinModeSet(GPIO_TCXO_PORT, GPIO_TCXO_PIN, gpioModePushPull, 0);

   GPIO_PinModeSet(GPIO_MAIN_POWER_PORT, GPIO_MAIN_POWER_PIN, gpioModePushPull, 1);

   // GPS I/O Config.
   GPIO_PinModeSet(GPIO_GPS_IGNITION_PORT, GPIO_GPS_IGNITION_PIN, gpioModePushPull, 0);
   GPIO_PinModeSet(GPIO_GPS_PWR_ENA_PORT, GPIO_GPS_PWR_ENA_PIN, gpioModePushPull, 0);
   GPIO_PinModeSet(GPIO_GPS_SYS_ON_RDY_PORT, GPIO_GPS_SYS_ON_RDY_PIN, gpioModeInput, 0);

#else

  /* Configure PB1 as input and enable interrupt */
  GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, gpioModeInputPull, 1);
  GPIO_IntConfig(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, false, true, true);
  GPIOINT_CallbackRegister( BSP_GPIO_PB1_PIN, GPIO_PB1_IRQHandler );
#endif
}

#if defined(WBRD200_VER1)
/**************************************************************************//**
 * @brief GPIO Interrupt handler (PD6)
 *        UART-CTS Send data buffer to logger.
 *****************************************************************************/
static void GPIO_PD6_IRQHandler( uint8_t pin )
{
  (void)pin;
  rtsEvent = true;
}

#elif defined(WBRD100_VER1)
/**************************************************************************//**
 * @brief GPIO Interrupt handler (PB1)
 *        Switches between analog and digital clock modes.
 *****************************************************************************/
static void GPIO_PB0_IRQHandler( uint8_t pin )
{
  (void)pin;

  /* Check if already transmitting some packets,
   * send one otherwise. */
  if ( !appTxPktCntr )
  {
    appTxPktCntr += 1;
  }
}

#endif

/**************************************************************************//**
 * @brief GPIO Interrupt handler (PB0)
 *        Increments the time by one minute.
 *****************************************************************************/
void RTC_App_IRQHandler()
{
  rtcTick = true;
}

uint32_t rtcTickCounter = 0;

/**************************************************************************//**
 * @brief  Main function of the example.
 *****************************************************************************/
int main(void)
{
  /* EZRadio driver init data and handler */
  EZRADIODRV_HandleData_t appRadioInitData = EZRADIODRV_INIT_DEFAULT;
  EZRADIODRV_Handle_t appRadioHandle = &appRadioInitData;

  /* EZRadio response structure union */
  ezradio_cmd_reply_t ezradioReply;

  /* Chip errata */
  CHIP_Init();

  /* HFXO 48MHz, divided by 1 */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

  /* Setup GPIO for pushbuttons. */
  GpioSetup();

  /* Set RTC to generate interrupt 250ms. */
  RTCDRV_Init();
  if (ECODE_EMDRV_RTCDRV_OK !=
      RTCDRV_AllocateTimer( &rtcTickTimer) )
  {
    while (1);
  }
  if (ECODE_EMDRV_RTCDRV_OK !=
      RTCDRV_StartTimer(rtcTickTimer, rtcdrvTimerTypePeriodic, APP_RTC_TIMEOUT_MS,
                        (RTCDRV_Callback_t)RTC_App_IRQHandler, NULL ) )
  {
    while (1);
  }

  BSP_LedsInit();
  BSP_LedSet(1);

  //cmdlisttest();

  /* Print header */
   printDbg("\nEZRadio TRx with ACK.");
#if defined(WBRD200_VER1)
   printDbg("\nWBRD200_VER1 Card.");
  if(ee_m24c0Test()) printDbg("eeprom test successful.");
  else printDbg("eeprom test failed.");
#endif

  //printDbg("ezr_temperatureGetTemp:%f", ezr_temperatureGetTemp());

  // TCXO power ON
  GPIO_PinOutSet(GPIO_TCXO_PORT, GPIO_TCXO_PIN);


  /* Configure packet transmitted callback. */
  appRadioInitData.packetTx.userCallback = &appPacketTransmittedCallback;
#if (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_PACKET_LENGTH_ENABLE == TRUE)
  /* Store length for variable length packet */
  radioTxPkt[EZRADIO_LENGTH_WORD_START + RADIO_CONFIGURATION_DATA_PKT_LENGTH_SIZE - 1] = APP_TX_PKT_DATA_LENGTH;

  /* Set the length of the variable length to the actual data */
#if (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_LENGTH_FIELD == 1)
  radioTxLengthConf.fieldLen.f1 = APP_TX_PKT_DATA_LENGTH;
#elif (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_LENGTH_FIELD == 2)
  radioTxLengthConf.fieldLen.f2 = APP_TX_PKT_DATA_LENGTH;
#elif (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_LENGTH_FIELD == 3)
  radioTxLengthConf.fieldLen.f3 = APP_TX_PKT_DATA_LENGTH;
#elif (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_LENGTH_FIELD == 4)
  radioTxLengthConf.fieldLen.f4 = APP_TX_PKT_DATA_LENGTH;
#elif (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_LENGTH_FIELD == 5)
  radioTxLengthConf.fieldLen.f5 = APP_TX_PKT_DATA_LENGTH;
#endif

#endif //#if (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_PACKET_LENGTH_ENABLE == TRUE)

  /* Configure packet received buffer and callback. */
  appRadioInitData.packetRx.userCallback = &appPacketReceivedCallback;
  appRadioInitData.packetRx.pktBuf = radioRxPkt;

  /* Configure packet received with CRC error callback. */
  appRadioInitData.packetCrcError.userCallback = &appPacketCrcErrorCallback;

  /* Configure auto ack packet buffer and callback. */
  appRadioInitData.autoAck.pktBuf = radioAutoAckPkt;
  appRadioInitData.autoAck.userCallback = &appAutoAckTransmittedCallback;

  /*Store auto ack packet data */
  radioAutoAckPkt[APP_PKT_DATA_START]     = 'P';
  radioAutoAckPkt[APP_PKT_DATA_START + 1] = 'H';
  radioAutoAckPkt[APP_PKT_DATA_START + 2] = 'Y';

#if (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_PACKET_LENGTH_ENABLE == TRUE)
  /* Auto ack data length differs from the normal packet length, so handle that accordingly. */
  appRadioInitData.autoAck.pktMode = ezradiodrvAutoAckPktCustom;
  appRadioInitData.autoAck.lenConfig.lenMode = ezradiodrvTransmitLenghtCustomFieldLen;

  /* Store length for variable length packet */
  radioAutoAckPkt[EZRADIO_LENGTH_WORD_START + RADIO_CONFIGURATION_DATA_PKT_LENGTH_SIZE - 1] = APP_AUTO_ACK_PKT_DATA_LENGTH;

  /* Set the length of the variable length to the auto ack data */
#if (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_LENGTH_FIELD == 1)
  appRadioInitData.autoAck.lenConfig.fieldLen.f1 = APP_AUTO_ACK_PKT_DATA_LENGTH;
#elif (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_LENGTH_FIELD == 2)
  appRadioInitData.autoAck.lenConfig.fieldLen.f2 = APP_AUTO_ACK_PKT_DATA_LENGTH;
#elif (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_LENGTH_FIELD == 3)
  appRadioInitData.autoAck.lenConfig.fieldLen.f3 = APP_AUTO_ACK_PKT_DATA_LENGTH;
#elif (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_LENGTH_FIELD == 4)
  appRadioInitData.autoAck.lenConfig.fieldLen.f4 = APP_AUTO_ACK_PKT_DATA_LENGTH;
#elif (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_LENGTH_FIELD == 5)
  appRadioInitData.autoAck.lenConfig.fieldLen.f5 = APP_AUTO_ACK_PKT_DATA_LENGTH;
#endif

#endif //#if (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_PACKET_LENGTH_ENABLE == TRUE)

  /* Initialize EZRadio device. */
  ezradioInit( appRadioHandle );

  /* Enable auto acknowledge feature. */
  ezradioEnableAutoAck( &(appRadioHandle->autoAck) );


  /* Print EZRadio device number. */
  ezradio_part_info(&ezradioReply);
  printDbg("Device: Si%04x\n", ezradioReply.PART_INFO.PART);

  /* Print instructions. */
  printDbg("Press PB0 to send\n  one packet.");
#if (APP_TX_PKT_SEND_NUM == 0xFFFF)
  printDbg("Press PB1 to send\n  unlimited packets.");
#else
  printDbg(" Press PB1 to send\n  %d packets.\n", APP_TX_PKT_SEND_NUM);
#endif

  /* Reset radio fifos and start reception. */
  ezradioResetTRxFifo();
  ezradioStartRx( appRadioHandle );

   //changeSignal = true;
//  	  appTxPktCntr = 0xFFFF;
  /* Enter infinite loop that will take care of ezradio plugin manager, packet transmission
   * and auto acknowledge. */
  while (1)
  {
    /* Run radio plug-in manager */
    ezradioPluginManager( appRadioHandle );

    if (rtcTick)
    {
      rtcTick = false;
      rtcTickCounter++;

 //     BSP_LedClear(0);

#if defined(WBRD200_VER1)
      if(rtsEvent){
    	  rtsEvent = false;
    	 // appTxPktCntr = true,
    	  printDbg("rtsEvent Occurred.");

    	  logger_interfaceSendBuffer();

    	  uint16_t lenRxLooger = logger_interfaceGetBuffer(strRxLogger, 32u, 2);
    	  if(lenRxLooger){
    		  strRxLogger[lenRxLooger] = '\0';
    		  printDbg("rtsAck:%s", strRxLogger);
    	  }else
    		  printDbg("rtsAck failed.");

    	  /* Read ezradio modem status (ie. contains the RSSI value) */
    	  //ezradio_get_modem_status(EZRADIO_CMD_GET_MODEM_STATUS_ARG_MODEM_CLR_PEND_MASK, &ezradioReply);
/*
          ezradio_part_info(&ezradioReply);
          printDbg("TEMP_ADC:%04x\n", ezradioReply.GET_ADC_READING.TEMP_ADC);
          printDbg("BATTERY_ADC:%04x\n", ezradioReply.GET_ADC_READING.BATTERY_ADC);
*/


      }
      if(!(rtcTickCounter % 4u)) BSP_LedToggle(0); // blinking every 1s
#elif defined(WBRD100_VER1)
      if(!(rtcTickCounter % 1000u)){// 100s


#if defined(BSP_GPS_MODULE)
    	  GPIO_PinOutSet(GPIO_GPS_PWR_ENA_PORT, GPIO_GPS_PWR_ENA_PIN);
    	  USTIMER_Init();
    	  USTIMER_Delay(1000u*1000u); // 10seconds
    	  USTIMER_DeInit();

    	  GPIO_PinOutSet(GPIO_GPS_IGNITION_PORT, GPIO_GPS_IGNITION_PIN);
    	  USTIMER_Init();
    	  USTIMER_Delay(10u*1000u); // 10 miliseconds
    	  USTIMER_DeInit();
    	  GPIO_PinOutClear(GPIO_GPS_IGNITION_PORT, GPIO_GPS_IGNITION_PIN);

    	  USTIMER_Init();
    	  USTIMER_Delay(3000u*1000u); // 30seconds
    	  USTIMER_DeInit();

#else
		printDbg("Battery Voltage: %d mV.", measureBatteryVoltage_mV());
		printDbg("ADC Input Voltage: %d mV.", measureSensor_mV());
		printDbg("Dendrometer: %d micra.", measureDendrometerSensor_um());
#endif

      }


#if defined(SENSOR_TRANSMITTER)
 //     if(!(rtcTickCounter % 8u)){// transmitter every 2s
 //   	  appTxPktCntr = true;
 //   	  appTxActive = false;
 //     }

//      if(!(rtcTickCounter % 500u)){// 125s
 //   	  BSP_LedClear(0);
 //   	  while(1){// auto off
 //   		  GPIO_PinOutClear(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN);
 //   	  }
 //     }
#endif

#endif


      /* Send a packet if requested */
      if (appTxPktCntr)
      {
#if defined(SENSOR_TRANSMITTER)
    	  BSP_LedSet(1);
#endif
        /* Try to send the packet */
        if ( !appTxActive )
        {
          /* Sing tx active state */
          appTxActive = true;

          /* Add data cntr as the data to be sent to the packet */
          radioTxPkt[APP_PKT_DATA_START]   = (uint8_t)( ((uint16_t)appDataCntr) >> 8 );
          radioTxPkt[APP_PKT_DATA_START+1] = (uint8_t)( ((uint16_t)appDataCntr) & 0x00FF );

          /* Note: The following line issues the auto acknowledge feature to skip
           *       one session.
           *        - Should be used for links where both nodes transmits ACK packets,
           *          in order to skip auto ACK for received ACK packets.
           *        - Should be commented out if the receiver node does not send back
           *          auto acknowledge packets. */
          ezradioSkipAutoAck( &(appRadioHandle->autoAck) );

          /* Transmit packet */
#if (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_PACKET_LENGTH_ENABLE == FALSE)
          ezradioStartTransmitConfigured( appRadioHandle, radioTxPkt );
#else
          ezradioStartTransmitCustom( appRadioHandle, radioTxLengthConf, radioTxPkt );
#endif

          printDbg("<--Data TX: %05d", appDataCntr);

          /* Increase data counter */
          appDataCntr++;

          /* Decrease number of requested packets,
           * if not configured to infinite. */
          if (appTxPktCntr != 0xFFFF)
          {
            /* Decrease request counter */
            if (appTxPktCntr)
            {
              appTxPktCntr--;
            }
          }
        }
        else
        {
#if defined(SENSOR_TRANSMITTER)
          printDbg("---Data TX: need to wait");
#endif
        }
      }
    }
  }
}


/**************************************************************************//**
 * @brief  Packet transmitted callback of the application.
 *
 * @param[in] handle EzRadio plugin manager handler.
 * @param[in] status Callback status.
 *****************************************************************************/
static void appPacketTransmittedCallback ( EZRADIODRV_Handle_t handle, Ecode_t status )
{
  if ( status == ECODE_EMDRV_EZRADIODRV_OK )
  {
    /* Sign tx passive state */
    appTxActive = false;

    /* Change to RX state */
    ezradioStartRx( handle );
#if defined(SENSOR_TRANSMITTER)
    BSP_LedClear(0);
    BSP_LedClear(1);;
#endif
  }
}

/**************************************************************************//**
 * @brief  Packet received callback of the application.
 *
 * @param[in] handle EzRadio plugin manager handler.
 * @param[in] status Callback status.
 *****************************************************************************/
static void appPacketReceivedCallback ( EZRADIODRV_Handle_t handle, Ecode_t status )
{
  //Silent warning.
  (void)handle;

  if ( status == ECODE_EMDRV_EZRADIODRV_OK )
  {

    /* Read out and print received packet data:
     *  - print 'ACK' in case of ACK was received
     *  - print the data if some other data was received. */
    if ( (radioRxPkt[APP_PKT_DATA_START] == 'P') &&
         (radioRxPkt[APP_PKT_DATA_START + 1] == 'H') &&
         (radioRxPkt[APP_PKT_DATA_START + 2] == 'Y') )
    {
#if defined(SENSOR_TRANSMITTER)
    BSP_LedSet(0);
#endif
      printDbg("-->Data RX: ACK");

    }
    else
    {
      uint16_t rxData;

      rxData =  (uint16_t)(radioRxPkt[APP_PKT_DATA_START]) << 8;
      rxData += (uint16_t)(radioRxPkt[APP_PKT_DATA_START+1]);

      printDbg("-->Data RX: %05d", rxData);
    }
  }
}

/**************************************************************************//**
 * @brief  Packet received with CRC error callback of the application.
 *
 * @param[in] handle EzRadio plugin manager handler.
 * @param[in] status Callback status.
 *****************************************************************************/
static void appPacketCrcErrorCallback ( EZRADIODRV_Handle_t handle, Ecode_t status )
{
  if ( status == ECODE_EMDRV_EZRADIODRV_OK )
  {
    printDbg("-->Pkt  RX: CRC Error");

    /* Change to RX state */
    ezradioStartRx( handle );
  }
}

/**************************************************************************//**
 * @brief  Packet transmitted callback of the application.
 *
 * @param[in] handle EzRadio plugin manager handler.
 * @param[in] status Callback status.
 *****************************************************************************/
static void appAutoAckTransmittedCallback ( EZRADIODRV_Handle_t handle, Ecode_t status )
{
  if ( status == ECODE_EMDRV_EZRADIODRV_OK )
  {
    printDbg("<--Data TX: ACK \n");

    ezradioStartRx( handle );

  }
}
