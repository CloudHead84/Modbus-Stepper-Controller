// -------------------------------------------------------------------------------------------
// ------------------ INCLUDE ----------------------------------------------------------------
// -------------------------------------------------------------------------------------------
#include <SPI.h>
#include <NativeEthernet.h>
#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "TeensyThreads.h"
#include "modbus.h"
#include "stepper_utils.h"

// -------------------------------------------------------------------------------------------
// ------------------ GLOBAL VARS ------------------------------------------------------------
// -------------------------------------------------------------------------------------------

// Ethernet
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 0, 118);
EthernetServer ethServer(502);

// Modbus
unsigned char modbus_buffer[100];
int processModbusMessage(unsigned char *buffer, int bufferSize);
bool modbus_connected;

extern bool mb_discrete_input[MAX_DISCRETE_INPUT];
extern bool mb_coils[MAX_COILS];
extern uint16_t mb_input_regs[MAX_INP_REGS];
extern uint16_t mb_holding_regs[MAX_HOLD_REGS];

// Stepper 1
#define STEPPER1_DIR_PIN 1
#define STEPPER1_STEP_PIN 2
#define STEPPER1_ENABLE_PIN 0

AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
REF_AXIS axis[1];


// threads
int threadid;




void updateIO()
{
    //INs
    axis[0].IN_xLifeBit = mb_coils[0];
    axis[0].IN_xPowerOnReq = mb_coils[1];
    axis[0].IN_xStartReq = mb_coils[2];
    axis[0].IN_xStopReq = mb_coils[3];      
    axis[0].IN_iModeReq = (int)mb_holding_regs[0];   
    axis[0].IN_diTargetPosition = create_long(mb_holding_regs[1],mb_holding_regs[2]);   
    axis[0].IN_rTargetSpeed = word_to_float(mb_holding_regs[3]);   
    axis[0].IN_rTargetAcc = word_to_float(mb_holding_regs[4]);
    axis[0].IN_rTargetDec = word_to_float(mb_holding_regs[5]);   
    
    //OUTs        
    mb_discrete_input[0] = axis[0].OUT_xLifeBitFB;
    mb_discrete_input[1] = axis[0].OUT_xPowered;    
    mb_discrete_input[2] = axis[0].OUT_xRunning;       
    mb_input_regs[0] = create_words(axis[0].OUT_diActualPosition,false);
    mb_input_regs[1] = create_words(axis[0].OUT_diActualPosition,true);
    mb_input_regs[2] = (int)axis[0].OUT_rActualSpeed;
  
    axis[0].OUT_wCheckSum = (word)mb_holding_regs[0] + (word)mb_holding_regs[1] + (word)mb_holding_regs[2] + (word)mb_holding_regs[3] + (word)mb_holding_regs[4] + (word)mb_holding_regs[5];
    mb_input_regs[3] = axis[0].OUT_wCheckSum;         
}


// -------------------------------------------------------------------------------------------
// ------------------ STEPPER THREAD ---------------------------------------------------------
// -------------------------------------------------------------------------------------------
void thread_stepper() {
  while (1) {


switch (axis[0].IN_iModeReq) {
  
    // velocity -------------------------------------------------------------------------------
    case 30:
    
    if (axis[0].IN_xStartReq ) 
        {
        stepper1.setMaxSpeed(axis[0].IN_rTargetSpeed);  
        stepper1.setAcceleration(1000.0);  
        stepper1.setSpeed(axis[0].IN_rTargetSpeed);
    }
    stepper1.runSpeed();
    break;

    // absolute -------------------------------------------------------------------------------
    case 40:
    
    if (axis[0].IN_xStartReq ) 
        {
        stepper1.setAcceleration(axis[0].IN_rTargetAcc);  
        stepper1.setMaxSpeed(axis[0].IN_rTargetSpeed);
        stepper1.moveTo(axis[0].IN_diTargetPosition); 
    }
    stepper1.run();
    break;
    
    // relative -------------------------------------------------------------------------------
    case 50:
    
    if (axis[0].IN_xStartReq ) 
        {
        stepper1.setAcceleration(axis[0].IN_rTargetAcc);  
        stepper1.setMaxSpeed(axis[0].IN_rTargetSpeed);
        stepper1.move(axis[0].IN_diTargetPosition); 
    }
    stepper1.run();
    break;
    default:
    // Statement(s)
    break; // Wird nicht benötigt, wenn Statement(s) vorhanden sind
}




    
    if (axis[0].IN_xStopReq) 
    {
        stepper1.stop();
    }


    if (axis[0].IN_xPowerOnReq) 
    {
       stepper1.enableOutputs();
       axis[0].OUT_xPowered = true;
    }else{
       stepper1.disableOutputs();
       axis[0].OUT_xPowered = false;
    }



    
     axis[0].OUT_diActualPosition = stepper1.currentPosition(); 
     axis[0].OUT_rActualSpeed = stepper1.speed();
     axis[0].OUT_xRunning = stepper1.isRunning();  
     axis[0].OUT_xLifeBitFB = axis[0].IN_xLifeBit;
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
