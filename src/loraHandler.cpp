/**
 * @file lora_handler.cpp
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief Initialization, event handlers and task for LoRaWan
 * @version 0.1
 * @date 2020-08-15
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "main.h"
#include "config.cpp"

/** DIO1 GPIO pin for RAK4631 */
#define PIN_LORA_DIO_1 47

/** Max size of the data to be transmitted. */
#define LORAWAN_APP_DATA_BUFF_SIZE 64
/** Number of trials for the join request. */
#define JOINREQ_NBTRIALS 8

/** Lora application data buffer. */
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];
/** Lora application data structure. */
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0};

// LoRaWan event handlers
/** LoRaWan callback when join network finished */
static void lorawan_has_joined_handler(void);
/** LoRaWan callback when data arrived */
static void lorawan_rx_handler(lmh_app_data_t *app_data);
/** LoRaWan callback after class change request finished */
static void lorawan_confirm_class_handler(DeviceClass_t Class);
/** LoRaWan Function to send a package */
bool sendLoRaFrame(void);

/** Semaphore used by SX126x IRQ handler to wake up LoRaWan task */
SemaphoreHandle_t loraEvent = NULL;

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
 * 
 * Set structure members to
 * LORAWAN_ADR_ON or LORAWAN_ADR_OFF to enable or disable adaptive data rate
 * LORAWAN_DEFAULT_DATARATE OR DR_0 ... DR_5 for default data rate or specific data rate selection
 * LORAWAN_PUBLIC_NETWORK or LORAWAN_PRIVATE_NETWORK to select the use of a public or private network
 * JOINREQ_NBTRIALS or a specific number to set the number of trials to join the network
 * LORAWAN_DEFAULT_TX_POWER or a specific number to set the TX power used
 * LORAWAN_DUTYCYCLE_ON or LORAWAN_DUTYCYCLE_OFF to enable or disable duty cycles
 *                   Please note that ETSI mandates duty cycled transmissions. 
 */
static lmh_param_t lora_param_init = {LORAWAN_ADR_OFF, DR_3, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_DEFAULT_TX_POWER, LORAWAN_DUTYCYCLE_ON};

/** Structure containing LoRaWan callback functions, needed for lmh_init() */
static lmh_callback_t lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                    lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler};



/** Flag whether to use OTAA or ABP network join method */
bool doOTAA = true;

/** LoRa task handle */
TaskHandle_t loraTaskHandle;
/** GPS reading task */
void loraTask(void *pvParameters);

/**
 * @brief SX126x interrupt handler
 * Called when DIO1 is set by SX126x
 * Gives loraEvent semaphore to wake up LoRaWan handler task
 * 
 */
void loraIntHandler(void)
{
  // SX126x set IRQ
  if (loraEvent != NULL)
  {
    // Wake up LoRa task
    xSemaphoreGive(loraEvent);
  }
}

/**
 * @brief Initialize LoRa HW and LoRaWan MAC layer
 * 
 * @return int8_t result
 *  0 => OK
 * -1 => SX126x HW init failure
 * -2 => LoRaWan MAC initialization failure
 * -3 => Subband selection failure
 * -4 => LoRaWan handler task start failure
 */
int8_t initLoRaWan(void)
{
  // Create the LoRaWan event semaphore
  loraEvent = xSemaphoreCreateBinary();
  // Initialize semaphore
  xSemaphoreGive(loraEvent);

  // Initialize LoRa chip.
  if (lora_rak4630_init() != 0)
  {
    return -1;
  }

  // Setup the EUIs and Keys
  lmh_setDevEui(nodeDeviceEUI);
  lmh_setAppEui(nodeAppEUI);
  lmh_setAppKey(nodeAppKey);

  // Initialize LoRaWan
  if (lmh_init(&lora_callbacks, lora_param_init, doOTAA) != 0)
  {
    return -2;
  }

  // For some regions we might need to define the sub band the gateway is listening to
  // This must be called AFTER lmh_init()
  if (!lmh_setSubBandChannels(1))
  {
    return -3;
  }

  // In deep sleep we need to hijack the SX126x IRQ to trigger a wakeup of the nRF52
  attachInterrupt(PIN_LORA_DIO_1, loraIntHandler, RISING);

  // Start the task that will handle the LoRaWan events
#ifndef MAX_SAVE
  Serial.println("Starting LoRaWan task");
#endif
  if (!xTaskCreate(loraTask, "LORA", 2048, NULL, TASK_PRIO_LOW, &loraTaskHandle))
  {
    return -4;
  }

  // Start Join procedure
#ifndef MAX_SAVE
  Serial.println("Start network join request");
#endif
  lmh_join();

  return 0;
}

/**
 * @brief Independent task to handle LoRa events
 * 
 * @param pvParameters Unused
 */
void loraTask(void *pvParameters)
{
  while (1)
  {
    if (lmh_join_status_get() == LMH_SET)
    { // Switch off the indicator lights
      digitalWrite(LED_CONN, LOW);
    }
    // Only if semaphore is available we need to handle LoRa events.
    // Otherwise we sleep here until an event occurs
    if (xSemaphoreTake(loraEvent, portMAX_DELAY) == pdTRUE)
    {
      // Switch off the indicator lights
      digitalWrite(LED_CONN, HIGH);

      // Handle Radio events with special process command!!!!
      Radio.IrqProcessAfterDeepSleep();
    }
  }
}

/**
 * @brief LoRa function for handling HasJoined event.
 */
static void lorawan_has_joined_handler(void)
{
  if (doOTAA)
  {
    #ifndef MAX_SAVE
        Serial.printf("OTAA joined and got dev address %08X\n", lmh_getDevAddr());
    #endif
  }
  else
  {
#ifndef MAX_SAVE
    Serial.println("ABP joined");
#endif
  }
  
  // Default is Class A, where the SX1262 transceiver is in sleep mode unless a package is sent
  // If switched to Class C the power consumption is higher because the SX1262 chip remains in RX mode

  // lmh_class_request(CLASS_C);

  // Now we are connected, start the timer that will wakeup the loop frequently
  taskWakeupTimer.begin(SLEEP_TIME, periodicWakeup);
  taskWakeupTimer.start();
}

/**
 * @brief Function for handling LoRaWan received data from Gateway
 *
 * @param app_data  Pointer to rx data
 */
static void lorawan_rx_handler(lmh_app_data_t *app_data)
{
#ifndef MAX_SAVE
  Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d\n",
    app_data->port, app_data->buffsize, app_data->rssi, app_data->snr);
#endif
  switch (app_data->port)
  {
  case 3:
    // Port 3 switches the class
    if (app_data->buffsize == 1)
    {
      switch (app_data->buffer[0])
      {
      case 0:
        lmh_class_request(CLASS_A);
#ifndef MAX_SAVE
        Serial.println("Request to switch to class A");
#endif
        break;

      case 1:
        lmh_class_request(CLASS_B);
#ifndef MAX_SAVE
        Serial.println("Request to switch to class B");
#endif
        break;

      case 2:
        lmh_class_request(CLASS_C);
#ifndef MAX_SAVE
        Serial.println("Request to switch to class C");
#endif
        break;

      default:
        break;
      }
    }

    // Send LoRaWan handler back to sleep
    xSemaphoreTake(loraEvent, 10);
    break;
  case LORAWAN_APP_PORT:
    // Copy the data into loop data buffer
    memcpy(rcvdLoRaData, app_data->buffer, app_data->buffsize);
    rcvdDataLen = app_data->buffsize;
    eventType = 0;
    // Notify task about the event
    if (taskEvent != NULL)
    {
#ifndef MAX_SAVE
      Serial.println("Waking up loop task");
#endif
      xSemaphoreGive(taskEvent);
    }

    // Send LoRa handler back to sleep
    xSemaphoreTake(loraEvent, 10);
  }
}

/**
 * @brief Callback for class switch confirmation
 * 
 * @param Class The new class
 */
static void lorawan_confirm_class_handler(DeviceClass_t Class)
{
#ifndef MAX_SAVE
  Serial.printf("switch to class %c done\n", "ABC"[Class]);
#endif

  // Informs the server that switch has occurred ASAP
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = LORAWAN_APP_PORT;
  lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);

  // Send LoRa handler back to sleep
  xSemaphoreTake(loraEvent, 10);
}

/**
 * @brief Send a LoRaWan package
 * 
 * @return result of send request
 */
bool sendLoRaFrame(float Distance float BatteryLevel)
{
  if (lmh_join_status_get() != LMH_SET)
  {
    //Not joined, try again later
    #ifndef MAX_SAVE
        Serial.println("Did not join network, skip sending frame");
    #endif
    // Send LoRa handler back to sleep
    xSemaphoreTake(loraEvent, 10);
    return false;
  }

  m_lora_app_data.port = LORAWAN_APP_PORT;

  //write temperature in a state we can send it
  int8_t envDistanceHigh = Distance/255;
  int8_t envDistanceLow = Distance-(envDistanceHigh*255);
  //same for battery
  int8_t envBatteryHigh = BatteryLevel/255;
  int8_t envBatteryLow = BatteryLevel-(envBatteryHigh*255);
  int8_t envGasHigh = GasLevel/255;

  uint8_t buff[4]; // reserve 9 bytes in memory 
  
  buff[0] = envDistanceHigh //high
  buff[1] = envDistanceLow; //low
  buff[2] = envBatteryHigh;
  buff[3] = envBatteryLow;

  //copy the buffer we just built into the lora buffer
  memcpy(m_lora_app_data_buffer,buff,sizeof(buff));
  m_lora_app_data.buffsize = sizeof(buff);

  //send the uplink
  lmh_error_status error = lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);

  // Send LoRa handler back to sleep
  xSemaphoreTake(loraEvent, 10);

  return (error == 0);
}
