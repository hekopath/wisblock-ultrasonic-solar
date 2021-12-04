/**
 * @file main.cpp
 * @author BFGNeil (neil@bfgNeil.io)
 * @brief Wisblock Soil Moisture & Env Sensor
 * @version 1.1
 * @date 2021-08-14
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "main.h"

/** Semaphore used by events to wake up loop task */
SemaphoreHandle_t taskEvent = NULL;

/** Timer to wakeup task frequently and send message */
SoftwareTimer taskWakeupTimer;

/** Buffer for received LoRaWan data */
uint8_t rcvdLoRaData[256];
/** Length of received data */
uint8_t rcvdDataLen = 0;

#include <Wire.h>

/**
 * @brief Flag for the event type
 * -1 => no event
 * 0 => LoRaWan data received
 * 1 => Timer wakeup
 * 2 => tbd
 * ...
 */
uint8_t eventType = -1;

/**
 * @brief Timer event that wakes up the loop task frequently
 * 
 * @param unused 
 */
void periodicWakeup(TimerHandle_t unused)
{
  // Switch on blue LED to show we are awake
  digitalWrite(LED_CONN, HIGH);
  eventType = 1;
  // Give the semaphore, so the loop task will wake up
  xSemaphoreGiveFromISR(taskEvent, pdFALSE);
}

/**
 * @brief Arduino setup function. Called once after power-up or reset
 * 
 */
void setup(void)
{
  // Create the LoRaWan event semaphore
  taskEvent = xSemaphoreCreateBinary();
  // Initialize semaphore
  xSemaphoreGive(taskEvent);

  // Initialize the built in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize battery reading
  initReadVBAT();

  // Initialize the connection status LED
  pinMode(LED_CONN, OUTPUT);
  digitalWrite(LED_CONN, LOW);

  #ifndef MAX_SAVE
    // Initialize Serial for debug output
    Serial.begin(115200);

    time_t timeout = millis();
    // On nRF52840 the USB serial is not available immediately
    while (!Serial)
    {
      if ((millis() - timeout) < 5000)
      {
        delay(100);
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      }
      else
      {
        break;
      }
    }
  #endif

  digitalWrite(LED_BUILTIN, LOW);

  // initialize Hygro / Temp sensor
  Wire.begin();
#define TRIG WB_IO6
#define ECHO WB_IO4
#define PD   WB_IO5   //power done control （=1 power done，=0 power on）

#define TIME_OUT  24125 //max measure distance is 4m,the velocity of sound is 331.6m/s in 0℃,TIME_OUT=4*2/331.6*1000000=24215us

float ratio = 346.6/1000/2;   //velocity of sound =331.6+0.6*25℃(m/s),(Indoor temperature about 25℃)
long int duration_time();  //measure high level time

Serial.println("========================");
   Serial.println("    RAK12007 test");
   Serial.println("========================");



  // Initialize LoRaWan and start join request
  int8_t loraInitResult = initLoRaWan();

  #ifndef MAX_SAVE
    if (loraInitResult != 0)
    {
      switch (loraInitResult)
      {
      case -1:
        Serial.println("SX126x init failed");
        break;
      case -2:
        Serial.println("LoRaWan init failed");
        break;
      case -3:
        Serial.println("Subband init error");
        break;
      case -4:
        Serial.println("LoRa Task init error");
        break;
      default:
        Serial.println("LoRa init unknown error");
        break;
      }

      // Without working LoRa we just stop here
      while (1)
      {
        Serial.println("Nothing I can do, just loving you");
        delay(5000);
      }
    }
    Serial.println("LoRaWan init success");
  #endif

  // Take the semaphore so the loop will go to sleep until an event happens
  xSemaphoreTake(taskEvent, 10);
}

/**
 * @brief Arduino loop task. Called in a loop from the FreeRTOS task handler
 * 
 */
void loop(void)
{
  // Switch off blue LED to show we go to sleep
  digitalWrite(LED_BUILTIN, LOW);

  // Sleep until we are woken up by an event
  if (xSemaphoreTake(taskEvent, portMAX_DELAY) == pdTRUE)
  {
    // Switch on blue LED to show we are awake
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500); // Only so we can see the blue LED

    // Check the wake up reason
    switch (eventType)
    {
    case 0: // Wakeup reason is package downlink arrived
      #ifndef MAX_SAVE
            Serial.println("Received package over LoRaWan");
      #endif
      if (rcvdLoRaData[0] > 0x1F)
      {
        #ifndef MAX_SAVE
                Serial.printf("%s\n", (char *)rcvdLoRaData);
        #endif
      }
      else
      {
        #ifndef MAX_SAVE
                for (int idx = 0; idx < rcvdDataLen; idx++)
                {
                  Serial.printf("%X ", rcvdLoRaData[idx]);
                }
                Serial.println("");
        #endif
      }

      break;
    case 1:
    { // Wakeup reason is timer
      #ifndef MAX_SAVE
            Serial.println("Timer wakeup");
      #endif
      // Wake up by timer expiration

      // Read the Distance
      long int duration, mm;
      digitalWrite(LED_BLUE,HIGH);
      duration = duration_time();
      if(duration > 0)
     {
     mm = duration*ratio; //Test distance = (high level time×velocity of sound (340M/S) / 2,
     digitalWrite(LED_BLUE,LOW);     
     Serial.print(mm);
     Serial.print("mm");
     Serial.println();
    }
   else
   {
     Serial.println("Out of range");  
     } 
   
  delay(100);
  }
long int duration_time()
{
   long int respondTime;
   pinMode(TRIG, OUTPUT);
   digitalWrite(TRIG, HIGH);
   delayMicroseconds(12);   //pull high time need over 10us 
   digitalWrite(TRIG, LOW);  
   pinMode(ECHO, INPUT);
   respondTime = pulseIn(ECHO, HIGH); // microseconds 
   delay(33);
   if(RAK4631_BOARD)
   {
     respondTime = respondTime*0.7726; // Time calibration factor is 0.7726,to get real time microseconds for 4631board
   }
   Serial.printf("respond time is %d\r\n",respondTime);

   if((respondTime>0)&&(respondTime < TIME_OUT))  //ECHO pin max timeout is 33000us according it's datasheet 
   {
    return respondTime;
   }
   else
   {
     return -1;  
   }   
}
     else
      {
      #ifndef MAX_SAVE
              Serial.printf("Error reading T & H\r\n");
      #endif
      }
      
      BatteryLevel = readVBAT();
      
      #ifndef MAX_SAVE
            Serial.printf("Distance %f\r\n", mm);
            Serial.printf("Battery Level %f\r\n", BatteryLevel);
      #endif

      // Send the data package
      if (sendLoRaFrame(Distance, BatteryLevel))
      {
        #ifndef MAX_SAVE
                Serial.println("LoRaWan package sent successfully");
        #endif
      }
      else
      {
        #ifndef MAX_SAVE
                Serial.println("LoRaWan package send failed");
        #endif
      }
    }
    break;
    default:
      #ifndef MAX_SAVE
            Serial.println("This should never happen ;-)");
      #endif
      break;
    }
    // Go back to sleep
    xSemaphoreTake(taskEvent, 10);
  }
}
