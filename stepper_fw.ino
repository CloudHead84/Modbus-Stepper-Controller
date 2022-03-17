// -------------------------------------------------------------------------------------------
// ------------------ INCLUDE ----------------------------------------------------------------
// -------------------------------------------------------------------------------------------
#include <SPI.h>
#include <NativeEthernet.h>
#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "TeensyThreads.h"



unsigned char modbus_buffer[100];
int processModbusMessage(unsigned char *buffer, int bufferSize);

#include "modbus.h"

extern uint16_t mb_holding_regs[MAX_HOLD_REGS];

// -------------------------------------------------------------------------------------------
// ------------------ GLOBAL VARS ------------------------------------------------------------
// -------------------------------------------------------------------------------------------

// Ethernet and Modbus
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 118);
EthernetServer ethServer(502);


// Stepper
#define STEPPER1_DIR_PIN 1
#define STEPPER1_STEP_PIN 2
#define STEPPER1_ENABLE_PIN 0

AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);

int threadid;
bool modbus_connected;
bool PowerOn;
int actualpos;
int targetpos;


// -------------------------------------------------------------------------------------------
// ------------------ STEPPER THREAD ------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
void thread_stepper() {
  while (1) {

    if (!stepper1.isRunning()) {
      if (modbus_connected) {
        stepper1.moveTo(targetpos); 
      }
    }
     stepper1.run();

     if (PowerOn) {
        stepper1.enableOutputs();
     }else{
        stepper1.disableOutputs();
     }
     actualpos = stepper1.currentPosition();
     threads.yield();
  }
}


// -------------------------------------------------------------------------------------------
// ------------------ SETUP ------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
void setup() {

  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  Serial.println("Ethernet Modbus TCP Example");

  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  Serial.println("Started Ethernet!");
  
  // start the server
  ethServer.begin();
  Serial.println("Started Ethernet Server!");
   
  // start the Modbus TCP server

  // stepper
    stepper1.setPinsInverted(false,false,true);  
    stepper1.setEnablePin(STEPPER1_ENABLE_PIN);
    stepper1.setMaxSpeed(2000.0);
    stepper1.setAcceleration(1000.0);
    stepper1.setMinPulseWidth(40);

  threads.setMicroTimer(10);
  threadid = threads.addThread(thread_stepper,1);


  
}



void updateIO()
{

      targetpos = mb_holding_regs[0];
      mb_input_regs[0]  = actualpos;
             PowerOn    = mb_coils[0];
      
}



// -------------------------------------------------------------------------------------------
// ------------------ LOOP -------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
void loop() {
      // listen for incoming clients
  EthernetClient client = ethServer.available();
  
   if (!client)
        return;
    
    Serial.println("new client!");

    while(client.connected())
    {
        // Wait until the client sends some data
        while(!client.available())
        {
            delay(1);
            if (!client.connected())
                return;
        }
        
        int i = 0;
        while(client.available())
        {
            modbus_connected = true;
            modbus_buffer[i] = client.read();
            i++;
            if (i == 100)
                break;
        }

        //DEBUG
        /*
        Serial.print("Received MB frame: ");
        PrintHex(modbus_buffer, i);
        */
        
        updateIO();
        unsigned int return_length = processModbusMessage(modbus_buffer, i);
        client.write((const uint8_t *)modbus_buffer, return_length);
        updateIO();
        delay(1);
    }
    
    Serial.println("Client disonnected");
    modbus_connected = false;
    delay(1);
}
