// -------------------------------------------------------------------------------------------
// ------------------ INCLUDE ----------------------------------------------------------------
// -------------------------------------------------------------------------------------------
#include <NativeEthernet.h>
#include <Arduino.h>
#include "TeensyThreads.h"
#include "modbus.h"
#include "stepper_utils.h"
#include <AsyncStepper.hpp>



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
#define STEPPER1_ENABLE_PIN 0
#define STEPPER1_SWITCH_PIN 3

const int stepper_dir_pin = 1;
const int stepper_step_pin = 2;

const int stepper_steps = 800;



AsyncStepper stepper1(stepper_steps, stepper_dir_pin, stepper_step_pin);

REF_AXIS axis[1];


// threads
int threadid;

long AbsPos;


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
    mb_discrete_input[3] = axis[0].OUT_xAck;   
    mb_discrete_input[4] = axis[0].OUT_xSwitch;       
    mb_input_regs[0] = create_words(axis[0].OUT_diActualPosition,false);
    mb_input_regs[1] = create_words(axis[0].OUT_diActualPosition,true);
    mb_input_regs[2] = (int)axis[0].OUT_rActualSpeed;
  
    axis[0].OUT_wCheckSum = (word)mb_holding_regs[0] + (word)mb_holding_regs[1] + (word)mb_holding_regs[2] + (word)mb_holding_regs[3] + (word)mb_holding_regs[4] + (word)mb_holding_regs[5];
    mb_input_regs[3] = axis[0].OUT_wCheckSum;         

    axis[0].OUT_xLifeBitFB = axis[0].IN_xLifeBit;
}


// -------------------------------------------------------------------------------------------
// ------------------ STEPPER THREAD ---------------------------------------------------------
// -------------------------------------------------------------------------------------------
void thread_stepper() {
  while (1) {

 
  if (not axis[0].IN_xStartReq and axis[0].OUT_xAck) 
  {
        axis[0].OUT_xAck = false;                  
  }


    
switch (axis[0].IN_iModeReq) {
  
    // homing-------------------------------------------------------------------------------
    case 20:

     if (not digitalRead(STEPPER1_SWITCH_PIN))
      {
        stepper1.Stop();
      }   
    
    else if (axis[0].IN_xStartReq and not axis[0].OUT_xAck) 
        {
        axis[0].OUT_xAck = true;

        if (axis[0].IN_rTargetSpeed > 0 )
        {
        stepper1.SetSpeed(axis[0].IN_rTargetSpeed);  
        stepper1.SetAcceleration(axis[0].IN_rTargetAcc, axis[0].IN_rTargetDec); 
        stepper1.RotateContinuous(AsyncStepper::CW);  
        } 
        else    
        {
        stepper1.SetSpeed(axis[0].IN_rTargetSpeed * -1);  
        stepper1.SetAcceleration(axis[0].IN_rTargetAcc, axis[0].IN_rTargetDec); 
        stepper1.RotateContinuous(AsyncStepper::CCW); 
        }
                    
    }
    break;    
    
       case 21:

     if (digitalRead(STEPPER1_SWITCH_PIN))
      {
        stepper1.Stop();
        stepper1.SetAbsoluteStep(axis[0].IN_diTargetPosition); 
      }   
    
    else if (axis[0].IN_xStartReq and not axis[0].OUT_xAck) 
        {
        axis[0].OUT_xAck = true;

        if (axis[0].IN_rTargetSpeed > 0 )
        {
        stepper1.SetSpeed(axis[0].IN_rTargetSpeed);  
        stepper1.SetAcceleration(axis[0].IN_rTargetAcc, axis[0].IN_rTargetDec); 
        stepper1.RotateContinuous(AsyncStepper::CW);  
        } 
        else    
        {
        stepper1.SetSpeed(axis[0].IN_rTargetSpeed * -1);  
        stepper1.SetAcceleration(axis[0].IN_rTargetAcc, axis[0].IN_rTargetDec); 
        stepper1.RotateContinuous(AsyncStepper::CCW); 
        }
                    
    }
    break;  
    
    
    // velocity -------------------------------------------------------------------------------
    case 30:
    
    if (axis[0].IN_xStartReq and not axis[0].OUT_xAck) 
        {
        axis[0].OUT_xAck = true;

        if (axis[0].IN_rTargetSpeed > 0 )
        {
        stepper1.SetSpeed(axis[0].IN_rTargetSpeed);  
        stepper1.SetAcceleration(axis[0].IN_rTargetAcc, axis[0].IN_rTargetDec); 
        stepper1.RotateContinuous(AsyncStepper::CW);  
        } 
        else    
        {
        stepper1.SetSpeed(axis[0].IN_rTargetSpeed * -1);  
        stepper1.SetAcceleration(axis[0].IN_rTargetAcc, axis[0].IN_rTargetDec); 
        stepper1.RotateContinuous(AsyncStepper::CCW); 
        }
                    
    }
    break;

    // absolute -------------------------------------------------------------------------------
    case 40:
    
    if (axis[0].IN_xStartReq and not axis[0].OUT_xAck) 
        {
          
        axis[0].OUT_xAck = true;
        stepper1.SetSpeed(axis[0].IN_rTargetSpeed);  
        stepper1.SetAcceleration(axis[0].IN_rTargetAcc, axis[0].IN_rTargetDec); 


        AbsPos = axis[0].IN_diTargetPosition - stepper1.GetAbsoluteStep();

        
        if (AbsPos > 0 )
        {
        stepper1.Rotate(AbsPos,AsyncStepper::CW);
        }
        else
        { 
         stepper1.Rotate(AbsPos,AsyncStepper::CCW);
        }
        
    }
    break;
    
    // relative -------------------------------------------------------------------------------
    case 50:
    
    if (axis[0].IN_xStartReq and not axis[0].OUT_xAck) 
        {
          
        axis[0].OUT_xAck = true;
        stepper1.SetSpeed(axis[0].IN_rTargetSpeed);  
        stepper1.SetAcceleration(axis[0].IN_rTargetAcc, axis[0].IN_rTargetDec); 
        if (axis[0].IN_diTargetPosition > 0 )
        {
        stepper1.Rotate(axis[0].IN_diTargetPosition,AsyncStepper::CW);
        }
        else
        { 
         stepper1.Rotate(axis[0].IN_diTargetPosition,AsyncStepper::CCW);
        }
       }
    
    break;

    
    default:
    // Statement(s)
    break; // Wird nicht benötigt, wenn Statement(s) vorhanden sind
}




    
    if (axis[0].IN_xStopReq and stepper1.GetCurrentSpeed() != 0 and not axis[0].OUT_xAck) 
    {
        axis[0].OUT_xAck = true;  
        stepper1.SetAcceleration(axis[0].IN_rTargetAcc, axis[0].IN_rTargetDec); 
        stepper1.Break();
    }


    if (axis[0].IN_xPowerOnReq) 
    {
       digitalWrite(STEPPER1_ENABLE_PIN, LOW);
       axis[0].OUT_xPowered = true;
    }
    else
    {
       digitalWrite(STEPPER1_ENABLE_PIN, HIGH);
       axis[0].OUT_xPowered = false;
       Serial.println("power off");
    }




     axis[0].OUT_xSwitch = not digitalRead(STEPPER1_SWITCH_PIN);
     axis[0].OUT_diActualPosition = stepper1.GetAbsoluteStep(); 
     axis[0].OUT_rActualSpeed = stepper1.GetCurrentSpeed();
     axis[0].OUT_xRunning = axis[0].OUT_rActualSpeed != 0;  
     stepper1.Update();
     threads.yield();
  }
}


// -------------------------------------------------------------------------------------------
// ------------------ SETUP ------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
  void setup() {

    pinMode(STEPPER1_ENABLE_PIN, OUTPUT);
    digitalWrite(STEPPER1_ENABLE_PIN, HIGH);  
    
    pinMode(STEPPER1_SWITCH_PIN, INPUT_PULLUP);

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

  threads.setMicroTimer(10);
  threads.addThread(thread_stepper);


  
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
